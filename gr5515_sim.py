#!/usr/bin/env python3
"""Émulateur partiel GR5515 (Cortex-M4) pour valider nos firmwares T1000-A avant de les flasher.

Ce qui est émulé :
  - le cœur Cortex-M4 (Unicorn), depuis le vecteur de réinitialisation de l'image ;
  - la RAM (256 Ko, visible à 0x30000000 et à 0x00800000 comme sur la puce) ;
  - la Flash XIP (1 Mo à 0x01000000), avec la sémantique NOR (écriture = ET bit à bit,
    effacement par secteur de 4 Ko) — via l'interception de hal_flash_read/write/erase ;
  - les périphériques (0xA0000000) et l'espace système (0xE0000000) comme mémoire simple,
    avec compteurs de temps (DWT, SysTick) qui avancent pour ne pas bloquer les temporisations.

Ce qui ne l'est PAS (la ROM de la puce n'est pas disponible) :
  - la ROM : chaque appel vers 0x00000000–0x0007FFFF est remplacé par un bouchon qui renvoie 0
    (ou par un gestionnaire Python pour quelques fonctions connues) et journalisé ;
  - la pile BLE, les interruptions, le comportement électrique des broches, les durées réelles.

Il valide donc NOTRE code et l'enchaînement du démarrage du SDK : absence de faute mémoire, de boucle
infinie, de saut hors image, et la logique de sécurité (compteur de démarrages, passage en DFU).
"""
import ctypes, re, struct, sys
from collections import Counter
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, \
    UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UC_HOOK_INTR, UC_HOOK_MEM_UNMAPPED, UC_PROT_ALL
from unicorn.arm_const import *

from twin.paths import rom_syms

ROM_BASE, ROM_SIZE = 0x00000000, 0x00080000
RAM_SIZE = 0x00040000
RAM_A, RAM_B = 0x30000000, 0x00800000          # deux vues de la même RAM
FLASH_BASE, FLASH_SIZE = 0x01000000, 0x00100000
PERIPH_BASE, PERIPH_SIZE = 0xA0000000, 0x00100000
SYS_BASE, SYS_SIZE = 0xE0000000, 0x00100000
RET_MAGIC = 0x0FFFFFF0                          # adresse de retour fictive pour call()
SECTOR = 0x1000
AIRCR = 0xE000ED0C


class Stop(Exception):
    pass


class Reset(Stop):
    pass


class GR5515Sim:
    def __init__(self, elf_path, flash_preset=None, verbose=False):
        self.verbose = verbose
        self.log = []
        self.rom_calls = Counter()
        self.events = []                        # (type, détails) : reset, dfu_flag, led…
        self.flash = bytearray(b'\xff' * FLASH_SIZE)
        self.button = 0
        self.stop_at = {}                       # adresse -> raison d'arrêt
        self.insn_count = 0
        self.total_insn = 0
        self.pc_hist = Counter()

        self.rom = {}
        for line in open(rom_syms()):
            m = re.match(r'\s*(\w+)\s*=\s*(0x[0-9a-fA-F]+)', line)
            if m:
                self.rom[int(m.group(2), 16) & ~1] = m.group(1)

        self.uc = uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(ROM_BASE, ROM_SIZE, UC_PROT_ALL)
        uc.mem_write(ROM_BASE, b'\x70\x47' * (ROM_SIZE // 2))      # « bx lr » partout
        self._ram = ctypes.create_string_buffer(RAM_SIZE)
        uc.mem_map_ptr(RAM_A, RAM_SIZE, UC_PROT_ALL, ctypes.addressof(self._ram))
        uc.mem_map_ptr(RAM_B, RAM_SIZE, UC_PROT_ALL, ctypes.addressof(self._ram))
        uc.mem_map(FLASH_BASE, FLASH_SIZE, UC_PROT_ALL)
        uc.mem_map(PERIPH_BASE, PERIPH_SIZE, UC_PROT_ALL)
        uc.mem_map(SYS_BASE, SYS_SIZE, UC_PROT_ALL)
        uc.mem_map(RET_MAGIC & ~0xFFF, 0x1000, UC_PROT_ALL)
        uc.mem_write(RET_MAGIC, b'\x00\xbf' * 8)                    # nop

        # Image : segments chargés à leur adresse de chargement (LMA) en Flash
        self.elf = ELFFile(open(elf_path, 'rb'))
        for seg in self.elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                lma = seg['p_paddr']
                if FLASH_BASE <= lma < FLASH_BASE + FLASH_SIZE:
                    data = seg.data()
                    self.flash[lma - FLASH_BASE:lma - FLASH_BASE + len(data)] = data
        self.sym = {}
        for s in self.elf.get_section_by_name('.symtab').iter_symbols():
            if s['st_info']['type'] == 'STT_FUNC' and s['st_value']:
                self.sym[s.name] = s['st_value'] & ~1
        self.addr2sym = {v: k for k, v in self.sym.items()}
        self.var = {}
        for sy in self.elf.get_section_by_name('.symtab').iter_symbols():
            if sy['st_info']['type'] == 'STT_OBJECT' and sy['st_value']:
                self.var[sy.name] = sy['st_value']
        self.traced = set()                     # fonctions dont on journalise l'entrée
        self.calls = []
        self.idle_count = 0
        self.idle_limit = None
        self.on_idle = None
        for addr, data in (flash_preset or {}).items():
            self.flash[addr - FLASH_BASE:addr - FLASH_BASE + len(data)] = data
        self._sync_flash()
        self._rom_boot_state()

        # Interceptions par nom de fonction de l'ELF
        self.intercept = {
            'hal_flash_init': lambda: 1,
            'hal_flash_read': self._f_read,
            'hal_flash_write': self._f_write,
            'hal_flash_write_r': self._f_write,
            'hal_flash_erase': self._f_erase,
            'app_io_read_pin': self._io_read,
            'app_io_write_pin': self._io_write,
            'hal_nvic_system_reset': self._reset,
            # Pile BLE en ROM : non émulable, on note l'appel et on renvoie « succès »
            'ble_stack_init': lambda: self.events.append(('ble_stack_init (non émulé)',)) or 0,
        }
        # Fonctions ROM émulées (par nom de la table de symboles ROM)
        self.rom_handlers = {
            'hal_flash_sector_size': lambda: SECTOR,
            'hal_flash_get_info': self._rom_flash_info,
        }
        self.intercept_addr = {self.sym[n]: f for n, f in self.intercept.items() if n in self.sym}
        self.trace_addr = set()

        uc.hook_add(UC_HOOK_CODE, self._on_code)
        uc.hook_add(UC_HOOK_MEM_READ, self._on_sys_read, begin=SYS_BASE, end=SYS_BASE + SYS_SIZE - 1)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_sys_write, begin=SYS_BASE, end=SYS_BASE + SYS_SIZE - 1)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_flash_write_direct, begin=FLASH_BASE, end=FLASH_BASE + FLASH_SIZE - 1)
        uc.hook_add(UC_HOOK_INTR, self._on_intr)
        uc.hook_add(UC_HOOK_MEM_UNMAPPED, self._on_unmapped)
        uc.hook_add(UC_HOOK_MEM_WRITE, self._on_periph_write, begin=PERIPH_BASE, end=PERIPH_BASE + PERIPH_SIZE - 1)
        uc.hook_add(UC_HOOK_MEM_READ, self._on_periph_read, begin=PERIPH_BASE, end=PERIPH_BASE + PERIPH_SIZE - 1)
        self.cyc = 0
        self.cal_cyc, self._cal_prev = 0, 0          # temps du calendrier, en cycles de 64 MHz
        self.aon_reload, self.aon_elapsed = 0xFFFFFFFF, 0

    def _rom_boot_state(self):
        """État laissé en RAM par la ROM (et le bootloader) avant de sauter dans l'application.
        Valeurs reprises du firmware Seeed, qui initialise les mêmes handles (cf. 0x01035B98)."""
        w = lambda a, v: self.uc.mem_write(a, struct.pack('<I', v))
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

    # ---------------------------------------------------------------- Flash simulée
    def _sync_flash(self, addr=FLASH_BASE, size=FLASH_SIZE):
        o = addr - FLASH_BASE
        self.uc.mem_write(addr, bytes(self.flash[o:o + size]))

    def flash_bytes(self, addr, n):
        return bytes(self.flash[addr - FLASH_BASE:addr - FLASH_BASE + n])

    def _check_flash_range(self, addr, size, op):
        if not (FLASH_BASE <= addr and addr + size <= FLASH_BASE + FLASH_SIZE):
            raise Stop(f'{op} hors Flash : {addr:#x}+{size:#x}')
        if addr < 0x0101B000 and op != 'lecture':
            raise Stop(f'{op} DANS LE BOOTLOADER : {addr:#x}+{size:#x}')

    def _f_read(self, addr, buf, size):
        self._check_flash_range(addr, size, 'lecture')
        self.uc.mem_write(buf, self.flash_bytes(addr, size))
        return size

    def _f_write(self, addr, buf, size):
        self._check_flash_range(addr, size, 'écriture')
        data = self.uc.mem_read(buf, size)
        o = addr - FLASH_BASE
        for i, b in enumerate(data):
            self.flash[o + i] &= b                     # NOR : on ne peut que passer des 1 à 0
        self._sync_flash(addr, size)
        self.events.append(('flash_write', addr, bytes(data)))
        return size

    def _f_erase(self, addr, size):
        self._check_flash_range(addr, size, 'effacement')
        start = addr & ~(SECTOR - 1)
        end = (addr + size + SECTOR - 1) & ~(SECTOR - 1)
        self.flash[start - FLASH_BASE:end - FLASH_BASE] = b'\xff' * (end - start)
        self._sync_flash(start, end - start)
        self.events.append(('flash_erase', start, end - start))
        return 1

    def _on_flash_write_direct(self, uc, access, addr, size, value, _):
        raise Stop(f'écriture directe en Flash XIP à {addr:#x} (pc={uc.reg_read(UC_ARM_REG_PC):#x})')

    # ---------------------------------------------------------------- E/S
    def _rom_flash_info(self, p_id, p_size):
        if p_id:
            self.uc.mem_write(p_id, struct.pack('<I', 0x001460C8))
        if p_size:
            self.uc.mem_write(p_size, struct.pack('<I', FLASH_SIZE))
        return None

    def _io_read(self, typ, pin):
        if typ == 3 and pin == 0x1:      # APP_IO_TYPE_AON, AON0 = bouton
            return self.button
        return 0

    def _io_write(self, typ, pin, state):
        self.events.append(('io', typ, pin, state))
        return 0

    def _reset(self, *a):
        raise Reset('hal_nvic_system_reset')

    # ---------------------------------------------------------------- hooks
    def _on_code(self, uc, addr, size, _):
        self.insn_count += 1
        self.total_insn += 1
        if self.insn_count & 0x3FF == 0:
            self.pc_hist[addr & ~0x3F] += 1
        if addr in self.stop_at:
            raise Stop(self.stop_at[addr])
        if addr in self.trace_addr:
            self.calls.append(self.addr2sym[addr])
        if addr in self.intercept_addr:
            args = [uc.reg_read(r) for r in (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)]
            f = self.intercept_addr[addr]
            n = f.__code__.co_argcount - (1 if hasattr(f, '__self__') else 0)
            ret = f(*args[:n])
            self._return(ret)
            return
        if addr < ROM_SIZE:
            name = self.rom.get(addr, f'rom_{addr:#x}')
            self.rom_calls[name] += 1
            h = self.rom_handlers.get(name)
            if h:
                args = [uc.reg_read(r) for r in (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)]
                self._return(h(*args[:h.__code__.co_argcount - (1 if hasattr(h, '__self__') else 0)]))
            else:
                self._return(0)

    def _return(self, value):
        uc = self.uc
        if value is not None:
            uc.reg_write(UC_ARM_REG_R0, value & 0xFFFFFFFF)
        uc.reg_write(UC_ARM_REG_PC, uc.reg_read(UC_ARM_REG_LR) | 1)

    # Modèle minimal de périphériques : registres d'état qui suivent un registre de commande.
    # (adresse écrite, masque) -> (adresse d'état, masque)
    MIRRORS = {
        (0xA000DC0C, 0x1): (0xA000DC10, 0x1),     # XQSPI XIP : CTRL3.EN_REQ -> STAT.EN_OUT
    }

    # Registres d'état lus en boucle : valeur renvoyée = f(valeur courante)
    READ_RULES = {
        # MCU_SUB RNG2 : calibration terminée (bit16) avec un compte non nul (bits 4..15)
        0xA000E238: lambda v: (v | 0x10000 | (0x100 << 4)) if v & 3 else v,
    }

    def _on_periph_read(self, uc, access, addr, size, value, _):
        if addr & ~3 == 0xA000C594:          # AON TIMER_VAL, multiplexé par AON_PAD_CTL1[31:30]
            sel = (struct.unpack('<I', uc.mem_read(0xA000C55C, 4))[0] >> 30) & 3
            cal_en = struct.unpack('<I', uc.mem_read(0xA000C518, 4))[0] & 1
            if sel == 0 and cal_en:          # calendrier (activé par notre code) : croissant à 32 768 Hz
                self.cyc = (self.cyc + 1000) & 0xFFFFFFFF        # le temps avance à chaque lecture
                self.cal_cyc += (self.cyc - self._cal_prev) & 0xFFFFFFFF
                self._cal_prev = self.cyc
                uc.mem_write(0xA000C594, struct.pack('<I', (self.cal_cyc * 32768 // 64_000_000) & 0xFFFFFFFF))
                return
            ticks = (self.total_insn - self.aon_elapsed) // 2000     # minuteur décroissant (≈ 64 MHz / 32 kHz)
            uc.mem_write(0xA000C594, struct.pack('<I', (self.aon_reload - ticks) & 0xFFFFFFFF))
            return
        f = self.READ_RULES.get(addr & ~3)
        if f:
            a = addr & ~3
            if isinstance(f, tuple) and f[0] == 'copy':
                uc.mem_write(a, bytes(uc.mem_read(f[1], 4)))
                return
            v = struct.unpack('<I', uc.mem_read(a, 4))[0]
            uc.mem_write(a, struct.pack('<I', f(v) & 0xFFFFFFFF))

    def _on_periph_write(self, uc, access, addr, size, value, _):
        if addr == 0xA000C590:                # AON TIMER_VALUE : rechargement du compteur
            self.aon_reload, self.aon_elapsed = value, self.total_insn
        for (waddr, wmask), (saddr, smask) in self.MIRRORS.items():
            if addr == waddr:
                cur = struct.unpack('<I', uc.mem_read(saddr, 4))[0]
                cur = (cur | smask) if value & wmask else (cur & ~smask)
                uc.mem_write(saddr, struct.pack('<I', cur))

    def _on_sys_read(self, uc, access, addr, size, value, _):
        if addr == 0xE0001004:                       # DWT->CYCCNT : n'avance que si TRCENA et CYCCNTENA
            demcr = struct.unpack('<I', uc.mem_read(0xE000EDFC, 4))[0]
            ctrl = struct.unpack('<I', uc.mem_read(0xE0001000, 4))[0]
            if (demcr >> 24) & 1 and ctrl & 1:
                self.cyc = (self.cyc + 1000) & 0xFFFFFFFF
            uc.mem_write(addr, struct.pack('<I', self.cyc))
        elif addr == 0xE000E018:                     # SysTick->VAL descend
            self.cyc = (self.cyc + 1000) & 0xFFFFFFFF
            uc.mem_write(addr, struct.pack('<I', (0xFFFFFF - self.cyc) & 0xFFFFFF))
        elif addr == 0xE000E010:                     # SysTick->CTRL : COUNTFLAG
            uc.mem_write(addr, struct.pack('<I', 0x10005))

    def _on_sys_write(self, uc, access, addr, size, value, _):
        if addr == AIRCR and (value >> 16) == 0x05FA and value & 0x4:
            raise Reset('SCB->AIRCR SYSRESETREQ')

    def _on_unmapped(self, uc, access, addr, size, value, _):
        pc, lr = uc.reg_read(UC_ARM_REG_PC), uc.reg_read(UC_ARM_REG_LR)
        raise Stop(f'FAUTE accès non cartographié {addr:#x} (pc={pc:#x} {self.where(pc)}, '
                   f'lr={lr:#x} {self.where(lr & ~1)})')

    def _on_intr(self, uc, intno, _):
        pc = uc.reg_read(UC_ARM_REG_PC)
        raise Stop(f'exception/interruption {intno} à pc={pc:#x} ({self.where(pc)})')

    # ---------------------------------------------------------------- exécution
    def trace(self, *names):
        for n in names:
            self.trace_addr.add(self.sym[n])

    def w32(self, addr, v):
        self.uc.mem_write(addr, struct.pack('<I', v & 0xFFFFFFFF))

    def r32(self, addr):
        return struct.unpack('<I', self.uc.mem_read(addr, 4))[0]

    def _idle(self):
        """pwr_mgmt_schedule (mise en veille de la boucle principale) : on simule ici le temps
        qui passe et l'interruption du temporisateur 50 ms du firmware."""
        self.idle_count += 1
        if self.on_idle:
            self.on_idle(self)
        if self.idle_limit is not None and self.idle_count >= self.idle_limit:
            self.idle_ctx = self.uc.context_save()      # pour reprendre avec resume()
            raise Stop(f'{self.idle_count} passages en veille')
        return None

    def run_main_loop(self, idles, max_insn=50_000_000):
        """Démarre (ou continue) et laisse tourner la boucle principale pendant `idles` mises en veille."""
        self.intercept_addr[self.sym['pwr_mgmt_schedule']] = self._idle
        self.idle_count = 0
        self.idle_limit = idles
        return self.boot(max_insn=max_insn, stop_symbol=None)

    def resume(self, idles, max_insn=50_000_000):
        """Reprend la boucle principale là où run_main_loop/resume s'est arrêté (sans redémarrer)."""
        self.uc.context_restore(self.idle_ctx)
        self.idle_count = 0
        self.idle_limit = idles
        return self._run(self.uc.reg_read(UC_ARM_REG_LR) | 1, max_insn)

    def where(self, pc):
        best = max((a for a in self.addr2sym if a <= pc), default=None)
        return f'{self.addr2sym[best]}+{pc - best:#x}' if best is not None else '?'

    def boot(self, max_insn=20_000_000, stop_symbol='pwr_mgmt_schedule'):
        """Démarre depuis le vecteur de réinitialisation jusqu'à stop_symbol (boucle principale)."""
        sp, reset = struct.unpack('<II', self.flash_bytes(0x0101B000, 8))
        self.uc.reg_write(UC_ARM_REG_SP, sp)
        self.uc.reg_write(UC_ARM_REG_MSP, sp)
        if stop_symbol:
            self.stop_at[self.sym[stop_symbol]] = f'atteint {stop_symbol}'
        return self._run(reset, max_insn)

    def call(self, name, *args, max_insn=5_000_000):
        """Appelle une fonction du firmware et renvoie r0 (ou lève Reset/Stop)."""
        uc = self.uc
        for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            uc.reg_write(r, v)
        uc.reg_write(UC_ARM_REG_LR, RET_MAGIC | 1)
        self.stop_at[RET_MAGIC] = 'retour'
        res = self._run(self.sym[name], max_insn)
        del self.stop_at[RET_MAGIC]
        if res != 'retour':
            raise Stop(res)
        return uc.reg_read(UC_ARM_REG_R0)

    def _run(self, start, max_insn):
        self.insn_count = 0
        try:
            self.uc.emu_start(start | 1, 0xFFFFFFFF, count=max_insn)
        except Reset:
            raise
        except Stop as e:
            return str(e)
        except UcError as e:
            pc = self.uc.reg_read(UC_ARM_REG_PC)
            raise Stop(f'FAUTE {e} à pc={pc:#x} ({self.where(pc)})')
        hot = ', '.join(f'{self.where(a)}' for a, _ in self.pc_hist.most_common(3))
        raise Stop(f'BOUCLE : {max_insn} instructions sans atteindre la cible ; zones chaudes : {hot}')
