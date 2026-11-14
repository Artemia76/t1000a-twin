"""Cœur du jumeau numérique T1000-A : CPU GR5515 (Unicorn), mémoire, horloge virtuelle, interruptions.

Principes
---------
* **Horloge virtuelle unique** (`Twin.cycles`, 64 MHz) : avance avec les instructions exécutées (≈ 1 cycle
  chacune), avec les lectures d'horloge, et par sauts quand le firmware dort. DWT, SysTick, minuteur AON et
  calendrier en dérivent tous.
* **Tranches** : le CPU tourne par paquets de `SLICE` instructions (`emu_start(count=…)`) ; entre deux
  tranches, les événements des modèles échus sont traités et les rappels du firmware injectés.
* **Interruptions asynchrones** : un rappel du firmware (minuteur, E/S, UART, BLE) est appelé entre deux
  instructions quelconques si PRIMASK le permet, avec sa propre pile sous celle du code interrompu, puis le
  contexte est restauré — comme une vraie interruption. La graine `seed` décale les points d'injection pour
  explorer d'autres entrelacements, de façon reproductible.
* **Émulation de haut niveau** : les pilotes `app_*`, `ble_*`, `hal_flash_*` du SDK sont remplacés par
  les modèles Python des composants (crochets par adresse, aucun crochet global par instruction).
* **Redémarrages** : un reset (chien de garde, AIRCR, `hal_nvic_system_reset`) relance le CPU en gardant la
  Flash ; les composants externes (LR1110, GNSS, téléphone) gardent leur propre état.
"""
import ctypes, heapq, inspect, random, re, struct
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, \
    UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UC_HOOK_INTR, UC_HOOK_MEM_UNMAPPED, UC_PROT_ALL
from unicorn.arm_const import *
from elftools.elf.elffile import ELFFile

from .layouts import Layouts
from .paths import rom_syms


F_CPU = 64_000_000
ROM_BASE, ROM_SIZE = 0x00000000, 0x00080000
RAM_SIZE = 0x00040000
RAM_A, RAM_B = 0x30000000, 0x00800000          # deux vues de la même RAM
FLASH_BASE, FLASH_SIZE = 0x01000000, 0x00100000
APP_BASE = 0x0101B000                           # application (après le bootloader Seeed)
PERIPH_BASE, PERIPH_SIZE = 0xA0000000, 0x00100000
SYS_BASE, SYS_SIZE = 0xE0000000, 0x00100000
EM_BASE, EM_SIZE = 0xB0000000, 0x00020000       # mémoire d'échange BLE (tampon NVDS g_nvds_buf, pile BLE)
TRAMP = 0x0FFFFFF0                              # adresse de retour fictive (call / rappels injectés)
SECTOR = 0x1000
AIRCR = 0xE000ED0C
DFU_FLAG = 0x010E4000

SLICE = 10_000                                  # instructions par tranche (≈ 156 µs)
COST_CALL = 60                                  # cycles comptés pour un pilote remplacé par un modèle
COST_CLOCK = 16                                 # cycles comptés par lecture d'horloge
COST_IRQ = 120                                  # entrée + sortie d'interruption
QUIET_LOOPS = 8                                 # tours de boucle « à vide » avant d'accélérer
SPIN_SKIP = 64_000                              # saut maximal par tour accéléré (1 ms)

PASS = object()
_U32 = struct.Struct('<I')                                 # un crochet qui renvoie PASS laisse s'exécuter la fonction réelle


class Stop(Exception):
    """Arrêt de l'exécution (faute, boucle, cible atteinte…)."""


class Reset(Stop):
    """Le firmware a demandé (ou provoqué) un redémarrage."""


class _Sleep(Exception):
    pass


class _Break(Exception):
    pass


def ms(n):
    return int(n * F_CPU // 1000)


class Twin:
    def __init__(self, elf_path, flash=None, seed=0, name='t1000a', trace=False, turbo=True):
        self.name = name
        self.elf_path = elf_path
        self.trace_enabled = trace
        self.rng = random.Random(seed)
        self.seed = seed
        self.L = Layouts(elf_path)

        self.elf = ELFFile(open(elf_path, 'rb'))
        self.sym, self.var = {}, {}
        for s in self.elf.get_section_by_name('.symtab').iter_symbols():
            t = s['st_info']['type']
            if t == 'STT_FUNC' and s['st_value']:
                self.sym.setdefault(s.name, s['st_value'] & ~1)
            elif t == 'STT_OBJECT' and s['st_value']:
                self.var.setdefault(s.name, s['st_value'])
        self.addr2sym = {}
        for k, v in self.sym.items():
            self.addr2sym.setdefault(v, k)
        self._sorted_funcs = sorted(self.addr2sym)
        self.rom = {}
        for line in open(rom_syms()):
            m = re.match(r'\s*(\w+)\s*=\s*(0x[0-9a-fA-F]+)', line)
            if m:
                self.rom[int(m.group(2), 16) & ~1] = m.group(1)
        self.rom_names = set(self.rom.values())

        # Flash : image fournie (persistance) ou effacée + segments de l'ELF
        if flash is None:
            self.flash = bytearray(b'\xff' * FLASH_SIZE)
        else:
            self.flash = bytearray(flash)
            assert len(self.flash) == FLASH_SIZE
        self.program_elf()

        # Temps et événements
        self.cycles = 0
        self._heap, self._seq = [], 0
        self._pending = []                      # rappels firmware à injecter : (nom, adr, args, blobs)
        self._in_irq = False
        self._slice_base, self._slice_extra = 0, 0
        self.log_list = []                      # (cycles, type, détails)
        self.rom_calls = {}
        self.calls = []                         # fonctions tracées (trace())
        self.resets = []                        # (cycles, raison)
        self.state = 'off'                      # off / run / dfu / halted
        self.stop_reason = None

        # Registres de crochets (réappliqués à chaque reconstruction du CPU)
        self.intercept = {}                     # adr -> fonction Python
        self.rom_handlers = {}                  # nom ROM -> fonction
        self.breakpoints = {}                   # adr -> raison
        self.traced = set()
        self.reset_listeners = []               # appelés à chaque redémarrage du MCU
        self.pwr_mode = 0
        self.idle_mode_value = None
        self._sleep_req = False
        self._dwt_frozen = 0
        self._cal = None                        # (cycles au démarrage du calendrier)
        self.aon_reload, self.aon_t0 = 0xFFFFFFFF, 0
        self.uc, self._running, self._hooked, self._skip_bp = None, False, set(), None
        self.asleep, self.sleeps, self.slept = False, 0, 0
        # Accélération des boucles d'attente actives : voir _pwr_schedule
        self.turbo = turbo
        self._model_calls = self._calls_at_sched = 0
        self._quiet = 0
        self._yield = False
        self.spin_skipped = 0
        from collections import Counter
        self.pc_samples = Counter()

        self._install_base_models()
        self._build_cpu()

    # ================================================================== symboles
    def lookup(self, name, table=None):
        """Symbole C, ou C++ décoré (_Z<len>name…, _ZL<len>name… pour un symbole statique)."""
        table = self.sym if table is None else table
        if name in table:
            return name
        pref = re.compile(r'_ZL?%d%s' % (len(name), re.escape(name)))
        names = [n for n in table if pref.match(n)]
        if len(names) != 1:
            raise KeyError(f'{name} : {names or "introuvable"}')
        return names[0]

    def addr(self, name):
        return self.sym[self.lookup(name)]

    def vaddr(self, name):
        return self.var[self.lookup(name, self.var)]

    def has(self, name):
        try:
            self.lookup(name)
            return True
        except KeyError:
            return False

    def where(self, pc):
        import bisect
        i = bisect.bisect_right(self._sorted_funcs, pc) - 1
        if i < 0:
            return '?'
        a = self._sorted_funcs[i]
        return f'{self.addr2sym[a]}+{pc - a:#x}'

    # ================================================================== Flash
    def program_elf(self):
        """Écrit les segments de l'ELF en Flash (comme un DFU réussi)."""
        for seg in self.elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                lma = seg['p_paddr']
                if FLASH_BASE <= lma < FLASH_BASE + FLASH_SIZE:
                    data = seg.data()
                    self.flash[lma - FLASH_BASE:lma - FLASH_BASE + len(data)] = data

    def patch(self, addr, data):
        """Modifie l'image en Flash (mutation du firmware pour les tests), y compris dans le CPU."""
        o = addr - FLASH_BASE
        self.flash[o:o + len(data)] = data
        if self.uc is not None:
            self._sync_flash(addr & ~0xFFF, ((addr + len(data) + 0xFFF) & ~0xFFF) - (addr & ~0xFFF))

    def flash_bytes(self, addr, n):
        return bytes(self.flash[addr - FLASH_BASE:addr - FLASH_BASE + n])

    def _sync_flash(self, addr=FLASH_BASE, size=FLASH_SIZE):
        o = addr - FLASH_BASE
        self.uc.mem_write(addr, bytes(self.flash[o:o + size]))

    def _check_flash_range(self, addr, size, op):
        if not (FLASH_BASE <= addr and addr + size <= FLASH_BASE + FLASH_SIZE):
            raise Stop(f'{op} hors Flash : {addr:#x}+{size:#x} ({self.where(self.lr())})')
        if addr < APP_BASE and op != 'lecture':
            raise Stop(f'{op} DANS LE BOOTLOADER : {addr:#x}+{size:#x} ({self.where(self.lr())})')

    def _f_read(self, addr, buf, size):
        self._check_flash_range(addr, size, 'lecture')
        self.uc.mem_write(buf, self.flash_bytes(addr, size))
        self.cycles_add(size // 4)
        return size

    def _f_write(self, addr, buf, size):
        self._check_flash_range(addr, size, 'écriture')
        data = self.uc.mem_read(buf, size)
        o = addr - FLASH_BASE
        for i, b in enumerate(data):
            self.flash[o + i] &= b                     # NOR : on ne peut que passer des 1 à 0
        self._sync_flash(addr, size)
        self.log('flash_write', addr=addr, data=bytes(data))
        self.cycles_add(size * 64)                     # ≈ 1 µs par octet
        return size

    def _f_erase(self, addr, size):
        self._check_flash_range(addr, size, 'effacement')
        start = addr & ~(SECTOR - 1)
        end = (addr + size + SECTOR - 1) & ~(SECTOR - 1)
        self.flash[start - FLASH_BASE:end - FLASH_BASE] = b'\xff' * (end - start)
        self._sync_flash(start, end - start)
        self.log('flash_erase', addr=start, size=end - start)
        self.cycles_add(ms(30) * ((end - start) // SECTOR))   # ≈ 30 ms par secteur
        return 1

    # ================================================================== temps
    def now(self):
        """Temps virtuel courant (cycles), y compris pendant une tranche."""
        return self._slice_base + self._slice_extra if self._running else self.cycles

    def now_ms(self):
        return self.now() * 1000 // F_CPU

    def now_us(self):
        return self.now() * 1_000_000 // F_CPU

    def cycles_add(self, n):
        if self._running:
            self._slice_extra += n
        else:
            self.cycles += n

    def schedule(self, delay_cycles, fn, *args):
        """Programme fn(*args) (code Python d'un modèle) dans `delay_cycles` cycles. Renvoie un jeton annulable."""
        self._seq += 1
        ev = [self.now() + max(0, int(delay_cycles)), self._seq, fn, args, True]
        heapq.heappush(self._heap, ev)
        return ev

    def schedule_ms(self, delay_ms, fn, *args):
        return self.schedule(ms(delay_ms), fn, *args)

    @staticmethod
    def cancel(ev):
        if ev:
            ev[4] = False

    def _next_event(self):
        while self._heap and not self._heap[0][4]:
            heapq.heappop(self._heap)
        return self._heap[0][0] if self._heap else None

    # ================================================================== interruptions
    def irq(self, name, fn_addr, args=(), blobs=()):
        """Demande l'appel asynchrone d'un rappel du firmware.
        `blobs` : octets copiés sur la pile de l'interruption ; leur adresse remplace None dans `args`."""
        self._pending.append((name, fn_addr, tuple(args), tuple(blobs)))

    def _irq_enabled(self):
        return not (self.uc.reg_read(UC_ARM_REG_PRIMASK) & 1)

    def _inject_pending(self):
        while self._pending and not self._in_irq and self.state == 'run' and self._irq_enabled():
            name, fn, args, blobs = self._pending.pop(0)
            self._call_isr(name, fn, args, blobs)

    def _call_isr(self, name, fn, args, blobs):
        uc = self.uc
        ctx = uc.context_save()
        sp = (uc.reg_read(UC_ARM_REG_SP) - 0x80) & ~7          # cadre d'exception + marge
        ptrs = []
        for b in blobs:
            if isinstance(b, tuple):                            # (taille, fn(adresse) -> octets)
                size, build = b
                sp = (sp - size) & ~7
                uc.mem_write(sp, bytes(build(sp)))
            else:
                sp = (sp - len(b)) & ~7
                uc.mem_write(sp, bytes(b))
            ptrs.append(sp)
        it = iter(ptrs)
        regs = [next(it) if a is None else a for a in args]
        for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), regs):
            uc.reg_write(r, v & 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_SP, sp)
        uc.reg_write(UC_ARM_REG_LR, TRAMP | 1)
        self._in_irq = True
        self.cycles_add(COST_IRQ)
        try:
            self._exec_until_tramp(fn, f'interruption {name}')
        finally:
            self._in_irq = False
        uc.context_restore(ctx)

    def _exec_until_tramp(self, fn, what, limit=20_000_000):
        done = 0
        start = fn | 1
        while True:
            r = self._exec(start, min(SLICE * 10, limit - done), until=TRAMP)
            if r == 'until':
                return
            done += SLICE * 10
            if done >= limit:
                raise Stop(f'{what} : {limit} instructions sans retour ({self.where(self.pc())})')
            start = self.pc() | 1
            self._dispatch_due()

    # ================================================================== CPU
    def _build_cpu(self):
        self.uc = uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(ROM_BASE, ROM_SIZE, UC_PROT_ALL)
        uc.mem_write(ROM_BASE, b'\x70\x47' * (ROM_SIZE // 2))      # « bx lr » partout
        self._ram = ctypes.create_string_buffer(RAM_SIZE)
        uc.mem_map_ptr(RAM_A, RAM_SIZE, UC_PROT_ALL, ctypes.addressof(self._ram))
        uc.mem_map_ptr(RAM_B, RAM_SIZE, UC_PROT_ALL, ctypes.addressof(self._ram))
        uc.mem_map(FLASH_BASE, FLASH_SIZE, UC_PROT_ALL)
        self._periph = ctypes.create_string_buffer(PERIPH_SIZE)
        self._sys = ctypes.create_string_buffer(SYS_SIZE)
        uc.mem_map_ptr(PERIPH_BASE, PERIPH_SIZE, UC_PROT_ALL, ctypes.addressof(self._periph))
        uc.mem_map_ptr(SYS_BASE, SYS_SIZE, UC_PROT_ALL, ctypes.addressof(self._sys))
        uc.mem_map(EM_BASE, EM_SIZE, UC_PROT_ALL)
        uc.mem_map(TRAMP & ~0xFFF, 0x1000, UC_PROT_ALL)
        uc.mem_write(TRAMP, b'\x00\xbf' * 8)                       # nop
        self._sync_flash()
        self._rom_boot_state()
        self._running = False

        uc.hook_add(UC_HOOK_CODE, self._on_rom, begin=ROM_BASE, end=ROM_BASE + ROM_SIZE - 1)
        for a in set(self.intercept) | set(self.breakpoints) | self.traced:
            uc.hook_add(UC_HOOK_CODE, self._on_addr, begin=a, end=a)
        self._hooked = set(self.intercept) | set(self.breakpoints) | self.traced
        # Crochets mémoire limités aux registres modélisés (le reste est de la mémoire simple)
        for a in (0xE0001004, 0xE000E010, 0xE000E018):
            uc.hook_add(UC_HOOK_MEM_READ, self._on_sys_read, begin=a, end=a + 3)
        for a in (AIRCR, 0xE0001004):
            uc.hook_add(UC_HOOK_MEM_WRITE, self._on_sys_write, begin=a, end=a + 3)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_flash_write_direct, begin=FLASH_BASE, end=FLASH_BASE + FLASH_SIZE - 1)
        uc.hook_add(UC_HOOK_INTR, self._on_intr)
        uc.hook_add(UC_HOOK_MEM_UNMAPPED, self._on_unmapped)
        for a in [self.TIMER_LOAD, self.CAL_CTL] + [w for w, _ in self.MIRRORS]:
            uc.hook_add(UC_HOOK_MEM_WRITE, self._on_periph_write, begin=a, end=a + 3)
        for a in [self.TIMER_VAL] + list(self.READ_RULES):
            uc.hook_add(UC_HOOK_MEM_READ, self._on_periph_read, begin=a, end=a + 3)

        sp, reset = struct.unpack('<II', self.flash_bytes(APP_BASE, 8))
        uc.reg_write(UC_ARM_REG_SP, sp)
        uc.reg_write(UC_ARM_REG_MSP, sp)
        uc.reg_write(UC_ARM_REG_PC, reset | 1)

    def _hook_addr(self, a):
        if self.uc is not None and a not in self._hooked:
            self.uc.hook_add(UC_HOOK_CODE, self._on_addr, begin=a, end=a)
            self._hooked.add(a)

    def _rom_boot_state(self):
        """État laissé en RAM par la ROM (et le bootloader) avant de sauter dans l'application.
        Valeurs reprises du firmware Seeed, qui initialise les mêmes handles (cf. 0x01035B98)."""
        w = self.w32
        xq = 0x00801F28                       # g_xqspi_handle
        w(xq + 0x00, 0xA000D000)              # p_instance = XQSPI
        w(xq + 0x04, 1)                       # work_mode = XIP
        w(xq + 0x08, 1)                       # cache_mode
        w(xq + 0x0C, 0xEB)                    # read_cmd (quad I/O)
        w(xq + 0x10, 0)                       # baud_rate
        ex = 0x00801F64                       # g_exflash_handle
        w(ex + 0x00, xq)                      # p_xqspi
        w(ex + 0x14, 0x001460C8)              # flash_id (valeur plausible)
        w(ex + 0x18, FLASH_SIZE)              # flash_size
        w(0xA000DC10, 1)                      # XQSPI : drapeau XIP actif

    # ------------------------------------------------------------------ accès
    def _buf(self, a):
        if RAM_A <= a < RAM_A + RAM_SIZE:
            return self._ram, a - RAM_A
        if RAM_B <= a < RAM_B + RAM_SIZE:
            return self._ram, a - RAM_B
        if PERIPH_BASE <= a < PERIPH_BASE + PERIPH_SIZE:
            return self._periph, a - PERIPH_BASE
        if SYS_BASE <= a < SYS_BASE + SYS_SIZE:
            return self._sys, a - SYS_BASE
        return None, 0

    def r32(self, a):
        b, o = self._buf(a)
        if b is not None:
            return _U32.unpack_from(b, o)[0]
        return struct.unpack('<I', self.uc.mem_read(a, 4))[0]

    def w32(self, a, v):
        b, o = self._buf(a)
        if b is not None:
            _U32.pack_into(b, o, v & 0xFFFFFFFF)
        else:
            self.uc.mem_write(a, struct.pack('<I', v & 0xFFFFFFFF))

    def read(self, a, n):
        return bytes(self.uc.mem_read(a, n))

    def write(self, a, data):
        self.uc.mem_write(a, bytes(data))

    def cstr(self, a, maxlen=256):
        b = self.read(a, maxlen)
        return b.split(b'\0')[0].decode('utf-8', 'replace')

    def pc(self):
        return self.uc.reg_read(UC_ARM_REG_PC)

    def lr(self):
        return self.uc.reg_read(UC_ARM_REG_LR)

    def arg(self, i):
        """i-ème argument (0..) de la fonction interceptée (au point d'entrée)."""
        if i < 4:
            return self.uc.reg_read((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)[i])
        return self.r32(self.uc.reg_read(UC_ARM_REG_SP) + 4 * (i - 4))

    # ------------------------------------------------------------------ crochets
    def on(self, name, fn):
        """Remplace la fonction `name` du firmware par fn(*args) (arguments AAPCS ; renvoie r0, None ou PASS)."""
        if not self.has(name):
            if name in self.rom_names:                  # fonction en ROM : remplacée au niveau de la ROM
                self.on_rom(name, fn)
                return True
            return False
        a = self.addr(name)
        n = len(inspect.signature(fn).parameters)
        self.intercept[a] = (fn, n, name)
        self._hook_addr(a)
        return True

    def on_rom(self, name, fn):
        n = len(inspect.signature(fn).parameters)
        self.rom_handlers[name] = (fn, n)

    def trace(self, *names):
        for n in names:
            if self.has(n):
                a = self.addr(n)
                self.traced.add(a)
                self._hook_addr(a)

    def break_at(self, name, reason=None):
        a = self.addr(name)
        self.breakpoints[a] = reason or f'atteint {name}'
        self._hook_addr(a)

    def _return(self, value):
        uc = self.uc
        if value is not None:
            uc.reg_write(UC_ARM_REG_R0, value & 0xFFFFFFFF)
            if value > 0xFFFFFFFF or value < -0x80000000:
                uc.reg_write(UC_ARM_REG_R1, (value >> 32) & 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR) | 1)

    def _on_addr(self, uc, addr, size, _):
        if addr in self.breakpoints and not self._in_irq:
            if self._skip_bp == addr:
                self._skip_bp = None
            else:
                self._skip_bp = addr
                raise _Break(self.breakpoints[addr])
        if addr in self.traced:
            self.calls.append(self.addr2sym.get(addr, hex(addr)))
        h = self.intercept.get(addr)
        if h:
            fn, n, name = h
            if fn != self._pwr_schedule:
                self._model_calls += 1
            self.cycles_add(COST_CALL)
            ret = fn(*[self.arg(i) for i in range(n)])
            if ret is not PASS:
                self._return(ret)

    def _on_rom(self, uc, addr, size, _):
        name = self.rom.get(addr, f'rom_{addr:#x}')
        self.rom_calls[name] = self.rom_calls.get(name, 0) + 1
        h = self.rom_handlers.get(name)
        self._model_calls += 1
        self.cycles_add(COST_CALL)
        if h:
            fn, n = h
            self._return(fn(*[self.arg(i) for i in range(n)]))
        else:
            self._return(0)

    def _on_flash_write_direct(self, uc, access, addr, size, value, _):
        raise Stop(f'écriture directe en Flash XIP à {addr:#x} (pc={self.pc():#x} {self.where(self.pc())})')

    def _on_unmapped(self, uc, access, addr, size, value, _):
        pc, lr = self.pc(), self.lr()
        raise Stop(f'FAUTE accès non cartographié {addr:#x} (pc={pc:#x} {self.where(pc)}, '
                   f'lr={lr:#x} {self.where(lr & ~1)})')

    def _on_intr(self, uc, intno, _):
        pc = self.pc()
        raise Stop(f'exception/interruption {intno} à pc={pc:#x} ({self.where(pc)})')

    # ------------------------------------------------------------------ registres modélisés
    MIRRORS = {
        (0xA000DC0C, 0x1): (0xA000DC10, 0x1),     # XQSPI XIP : CTRL3.EN_REQ -> STAT.EN_OUT
    }
    READ_RULES = {
        # MCU_SUB RNG2 : calibration terminée (bit16) avec un compte non nul (bits 4..15)
        0xA000E238: lambda v: (v | 0x10000 | (0x100 << 4)) if v & 3 else v,
    }
    CAL_CTL, PAD_CTL1, TIMER_VAL, TIMER_LOAD = 0xA000C518, 0xA000C55C, 0xA000C594, 0xA000C590

    def _on_periph_read(self, uc, access, addr, size, value, _):
        a = addr & ~3
        if a == self.TIMER_VAL:               # AON TIMER_VAL, multiplexé par AON_PAD_CTL1[31:30]
            self.cycles_add(COST_CLOCK)
            pb = self._periph
            sel = (_U32.unpack_from(pb, self.PAD_CTL1 - PERIPH_BASE)[0] >> 30) & 3
            now = self.now()
            if sel == 0 and pb[self.CAL_CTL - PERIPH_BASE][0] & 1:   # calendrier : croissant à 32 768 Hz
                if self._cal is None:
                    self._cal = now
                v = (now - self._cal) * 32768 // F_CPU
            else:                                             # minuteur de veille décroissant
                v = self.aon_reload - (now - self.aon_t0) * 32768 // F_CPU
            _U32.pack_into(pb, a - PERIPH_BASE, v & 0xFFFFFFFF)
            return
        f = self.READ_RULES.get(a)
        if f:
            self.w32(a, f(self.r32(a)))

    def _on_periph_write(self, uc, access, addr, size, value, _):
        if addr == self.TIMER_LOAD:
            self.aon_reload, self.aon_t0 = value, self.now()
        elif addr == self.CAL_CTL and not (value & 1):
            self._cal = None
        for (waddr, wmask), (saddr, smask) in self.MIRRORS.items():
            if addr == waddr:
                cur = self.r32(saddr)
                self.w32(saddr, (cur | smask) if value & wmask else (cur & ~smask))

    def _on_sys_read(self, uc, access, addr, size, value, _):
        if addr == 0xE0001004:                       # DWT->CYCCNT : n'avance que si TRCENA et CYCCNTENA
            self.cycles_add(COST_CLOCK)
            if (self.r32(0xE000EDFC) >> 24) & 1 and self.r32(0xE0001000) & 1:
                v = (self.now() - self._dwt_frozen) & 0xFFFFFFFF
            else:
                v = self.r32(addr)
            uc.mem_write(addr, struct.pack('<I', v))
        elif addr == 0xE000E018:                     # SysTick->VAL descend
            self.cycles_add(COST_CLOCK)
            load = (self.r32(0xE000E014) & 0xFFFFFF) + 1
            self.w32(addr, (load - 1 - self.now() % load) & 0xFFFFFF)
        elif addr == 0xE000E010:                     # SysTick->CTRL : COUNTFLAG
            self.w32(addr, self.r32(addr) | 0x10000)

    def _on_sys_write(self, uc, access, addr, size, value, _):
        if addr == AIRCR and (value >> 16) == 0x05FA and value & 0x4:
            raise Reset('SCB->AIRCR SYSRESETREQ')
        if addr == 0xE0001004:                       # écriture de CYCCNT
            self._dwt_frozen = (self.now() - value) & 0xFFFFFFFF

    # ------------------------------------------------------------------ modèles de base
    def _install_base_models(self):
        self.on('hal_flash_init', lambda: 1)
        self.on('hal_flash_read', self._f_read)
        self.on('hal_flash_write', self._f_write)
        self.on('hal_flash_write_r', self._f_write)
        self.on('hal_flash_erase', self._f_erase)
        self.on('hal_nvic_system_reset', self._sw_reset)
        self.on('pwr_mgmt_mode_set', self._pwr_mode_set)
        self.on('pwr_mgmt_schedule', self._pwr_schedule)
        self.on_rom('hal_flash_sector_size', lambda: SECTOR)
        self.on_rom('hal_flash_get_info', self._rom_flash_info)
        try:
            self.idle_mode_value = self.L.enum('PMR_MGMT_IDLE_MODE')
        except KeyError:
            self.idle_mode_value = None

    def _rom_flash_info(self, p_id, p_size):
        if p_id:
            self.w32(p_id, 0x001460C8)
        if p_size:
            self.w32(p_size, FLASH_SIZE)
        return None

    def _sw_reset(self):
        raise Reset('hal_nvic_system_reset')

    def _pwr_mode_set(self, mode):
        self.pwr_mode = mode
        return 0

    def _pwr_schedule(self):
        """pwr_mgmt_schedule : en mode veille, le cœur dort jusqu'au prochain événement."""
        if self.idle_mode_value is not None and self.pwr_mode == self.idle_mode_value and not self._in_irq:
            raise _Sleep()                      # le CPU reste dans pwr_mgmt_schedule jusqu'à une interruption
        # Mode actif : la boucle principale tourne à vide. Après QUIET_LOOPS tours sans aucun appel aux
        # modèles (ni E/S, ni radio, ni Flash, ni BLE), le temps saute au prochain événement (au plus
        # SPIN_SKIP) : les échéances et interruptions restent exactes, seul le nombre de tours change.
        if self._model_calls == self._calls_at_sched:
            self._quiet += 1
        else:
            self._quiet = 0
            self._calls_at_sched = self._model_calls
        if self.turbo and self._quiet >= QUIET_LOOPS and not self._in_irq:
            now = self.now()
            t = self._next_event()
            target = now + SPIN_SKIP if t is None else min(t, now + SPIN_SKIP)
            if target > now:
                self._slice_extra += target - now
                self.spin_skipped += target - now
                self._yield = True
                self.uc.emu_stop()
        return None

    # ================================================================== exécution
    def _exec(self, start, count, until=0):
        """Exécute au plus `count` instructions depuis `start`. Renvoie 'count', 'until', 'sleep' ou lève."""
        self._slice_base, self._slice_extra = self.cycles, 0
        self._running = True
        self._yield = False
        res = 'count'
        try:
            self.uc.emu_start(start | 1, until or 0xFFFFFFFF, count=count)
            if until and self.pc() == until:
                res = 'until'
            elif self._yield:
                res = 'yield'
        except _Sleep:
            res = 'sleep'
        except _Break as e:
            res = 'break'
            self.stop_reason = str(e)
        except Reset:
            raise
        except Stop:
            raise
        except UcError as e:
            pc = self.pc()
            raise Stop(f'FAUTE {e} à pc={pc:#x} ({self.where(pc)}), lr={self.lr():#x} ({self.where(self.lr() & ~1)})')
        finally:
            self._running = False
            done = count if res == 'count' else 0
            self.cycles = self._slice_base + max(done, self._slice_extra)
        return res

    def _dispatch_due(self):
        while True:
            t = self._next_event()
            if t is None or t > self.cycles:
                return
            ev = heapq.heappop(self._heap)
            ev[2](*ev[3])

    def power_on(self):
        self.state = 'run'
        self.log('power_on')

    def reset(self, reason):
        """Redémarrage du MCU (la Flash est conservée)."""
        self.resets.append((self.cycles, reason))
        self.log('reset', reason=reason)
        self._pending.clear()
        self._heap = [e for e in self._heap if getattr(e[2], 'survives_reset', False)]
        heapq.heapify(self._heap)
        self.pwr_mode = 0
        self._cal = None
        self.asleep = False
        self._build_cpu()
        for f in self.reset_listeners:
            f()
        if self.flash_bytes(DFU_FLAG, 1) == b'\x01':
            self.state = 'dfu'                 # le bootloader Seeed reste en DFU : l'application ne démarre plus
            self.log('dfu_mode')

    def run(self, ms_=None, until=None, max_insn=None, reset='restart', until_cycles=None):
        """Fait avancer le jumeau de `ms_` millisecondes virtuelles (ou jusqu'à until(twin) vrai, ou jusqu'à un
        point d'arrêt). reset='restart' : un redémarrage relance le MCU ; 'raise' : lève Reset.
        Renvoie la raison d'arrêt : 'temps', 'condition', 'arrêt: …', 'dfu'."""
        if self.state == 'off':
            self.power_on()
        end = self.cycles + ms(ms_) if ms_ is not None else until_cycles
        budget = max_insn
        while True:
            if self.state in ('dfu', 'halted'):
                if end is not None:
                    self.cycles = max(self.cycles, end)
                return self.state
            if until is not None and until(self):
                return 'condition'
            if end is not None and self.cycles >= end:
                return 'temps'
            try:
                if self.asleep:
                    if not self._sleep_until(end):
                        return 'temps'
                    continue
                self._dispatch_due()
                self._inject_pending()
                n = SLICE
                if self.seed:
                    n = self.rng.randint(SLICE // 2, SLICE)
                t = self._next_event()
                if t is not None:
                    n = max(1, min(n, t - self.cycles))
                if end is not None:
                    n = max(1, min(n, end - self.cycles))
                if budget is not None:
                    if budget <= 0:
                        hot = ', '.join(self.where(a) for a, _ in self.pc_samples.most_common(4))
                        raise Stop(f'BOUCLE : {max_insn} instructions sans atteindre la cible ; zones chaudes : {hot}')
                    n = min(n, budget)
                    budget -= n
                r = self._exec(self.pc(), n)
                pc = self.pc()
                self.pc_samples[pc] += 1
                if r == 'count' and self.read(pc & ~1, 2) == b'\xfe\xe7':
                    # « b . » : boucle sur elle-même, rien ne change jusqu'au prochain événement (ou à la fin)
                    t = self._next_event()
                    target = min(x for x in (t, end) if x is not None) if (t is not None or end is not None) else None
                    if target is None:
                        raise Stop(f'blocage définitif dans « b . » à {self.where(pc)} sans aucun événement')
                    if target > self.cycles:
                        self.spin_skipped += target - self.cycles
                        self.cycles = target
                if r == 'break':
                    return 'arrêt: ' + self.stop_reason
                if r == 'sleep':
                    self.asleep = True
                    self.sleeps += 1
            except Reset as e:
                if reset == 'raise':
                    raise
                self.reset(str(e))

    def _sleep_until(self, end):
        """Le cœur dort dans pwr_mgmt_schedule : le temps saute d'événement en événement jusqu'à ce qu'une
        interruption soit à servir (réveil) ou jusqu'à `end`. Renvoie True au réveil."""
        while True:
            if self._pending:
                self._wake()
                return True
            t = self._next_event()
            if t is None:
                if end is None:
                    raise Stop('veille sans aucun événement programmé : le firmware ne se réveillerait jamais')
                self.slept += end - self.cycles
                self.cycles = end
                return False
            if end is not None and t > end:
                self.slept += end - self.cycles
                self.cycles = end
                return False
            if t > self.cycles:
                self.slept += t - self.cycles
                self.cycles = t
            self._dispatch_due()

    def _wake(self):
        self.asleep = False
        self._return(None)                      # sortie de pwr_mgmt_schedule

    def call(self, name, *args, max_insn=5_000_000):
        """Appelle une fonction du firmware (synchrone, hors interruptions) et renvoie r0."""
        uc = self.uc
        ctx = uc.context_save()
        for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            uc.reg_write(r, v & 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_LR, TRAMP | 1)
        a = self.addr(name) if isinstance(name, str) else name
        self._exec_until_tramp(a, f'appel {name}', limit=max_insn)
        r0 = uc.reg_read(UC_ARM_REG_R0)
        uc.context_restore(ctx)
        return r0

    # ================================================================== journal
    def log(self, kind, **data):
        self.log_list.append((self.now(), kind, data))
        if self.trace_enabled:
            print(f'[{self.name} {self.now() / F_CPU * 1000:10.3f} ms] {kind} {data}')

    def events(self, kind):
        return [(t, d) for t, k, d in self.log_list if k == kind]
