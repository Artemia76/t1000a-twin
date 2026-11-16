"""Modèle comportemental de l'émetteur-récepteur Semtech LR1110 (partie radio LoRa).

Interface électrique (T1000-A) : SPI (NSS = GPIO6), NRESET = GPIO12, BUSY = GPIO13 (sortie du LR1110),
DIO9 (ligne d'IRQ) → inverseur à collecteur ouvert → AON1 : le GR5515 voit AON1 = NOT irq.

Sources : « UM » = manuel utilisateur LR1110 Rev 2.3 (Semtech, avril 2026) ; « mesuré » = sonde `lr` du
firmware debug sur le boîtier (variants/t1000-a/LrProbe.cpp, 2026-11-16, LR1110 firmware 03.07 ; détail dans
docs/materiel-t1000a.md du dépôt T1000A). Les durées mesurées sont à ±30 µs.

| Comportement | Modèle | Source |
|---|---|---|
| Protocole SPI | Stat1, Stat2, Irq[31:0] pendant chaque transaction ; réponse lue par une 2ᵉ transaction (Stat1 = CMD_DAT puis données) | UM §3.1, §3.2, §3.4 |
| Statut de commande | CMD_OK ; CMD_PERR (opcode inconnu) ; CMD_FAIL (SetRegMode, SetTcxoMode, SetDioAsRfSwitch hors STBY_RC ; SetPacketType hors STBY/FS ; SetRxDutyCycle sans type de paquet) | UM §3.4.2, §4.2.1, §5.3.1, §6.3.2, §7.2.6, §8.1.1 |
| Commande envoyée pendant BUSY haut | perdue (violation) | mesuré |
| BUSY après une commande simple | 45 µs | mesuré : 30 à 60 µs |
| Démarrage de l'oscillateur (TCXO) depuis STBY_RC | 5 ms avant XOSC, FS, RX, TX, écoute cyclique | mesuré : SetStandby(XOSC), SetFs ≈ 5 ms |
| CalibImage / Calibrate | 6,5 ms / 31 ms | mesuré |
| Reset (NRESET) | BUSY haut 237 ms, ResetStatus 2 | UM §2.1.6, tableau 2-15 |
| SetSleep | BUSY haut, DIO et MISO en haute impédance ; réveil par front de NSS seulement, la transaction est perdue ; ResetStatus 5 | UM §2.1.5, mesuré |
| Réveil d'un sommeil chaud | 640 µs ; configuration et tampons conservés | mesuré (manuel : < 1 ms) |
| Réveil d'un sommeil froid | 13,3 ms ; contexte perdu, IRQ Error (bit 23) | mesuré, UM §2.1.4 |
| Mode de repli (0x0213) | après TX_DONE et RX_DONE (RX simple, avec délai, écoute cyclique) ; défaut STBY_RC | UM §2.1.11, §7.2.5 |
| Expiration en RX | toujours STBY_RC | UM §7.2.5, mesuré |
| Délai RX arrêté | à l'en-tête, ou au préambule après StopTimeoutOnPreamble(1) | UM §7.2.2, §7.2.7 |
| Écoute cyclique (0x0214) | fenêtre RX (BUSY bas) puis sommeil (BUSY haut) de SleepPeriod + 5,2 ms ; ResetStatus 6 | UM §7.2.6, mesuré : 36,5 ms bas / 64,7 ms haut pour 36,8 / 59,5 ms |
| Écoute cyclique : préambule | reste en RX jusqu'à la fin de la trame, sinon fin de fenêtre 2 × rx + sommeil après la détection | UM §7.2.6 |
| Écoute cyclique : RX_DONE | mode de repli (STBY_RC), fin du cycle | UM §7.2.5, §7.2.6, mesuré |
| Écoute cyclique : NSS pendant le sommeil | réveil 640 µs, STBY_RC, ResetStatus 5, fin du cycle ; transaction perdue | UM §7.2.6, mesuré |
| Écoute cyclique : SetStandby pendant une fenêtre | fin du cycle | UM §7.2.6 |
| Captation d'une trame en écoute cyclique | il faut 8 symboles de son préambule dans une fenêtre | UM §8.2.2.1 (8 symboles minimum) |
| Tampons | TX et RX distincts | mesuré, UM §8.9 |
| Version | 0x22, LR1110, firmware 03.07 | mesuré |

Ce qui reste simplifié : RSSI/SNR fournis par l'éther, temps d'antenne par la formule Semtech, registres mémoire
(valeurs écrites relues), CAD sans seuils. Les commandes inconnues sont journalisées (`unknown_cmds`).
"""
import math, struct

CMD_OK, CMD_DAT, CMD_PERR, CMD_FAIL = 2, 3, 1, 0
MODE_SLEEP, MODE_STBY_RC, MODE_STBY_XOSC, MODE_FS, MODE_RX, MODE_TX = 0, 1, 2, 3, 4, 5
IRQ_TX_DONE, IRQ_RX_DONE, IRQ_PREAMBLE, IRQ_HEADER_VALID = 1 << 2, 1 << 3, 1 << 4, 1 << 5
IRQ_HEADER_ERR, IRQ_CRC_ERR, IRQ_CAD_DONE, IRQ_CAD_DETECTED, IRQ_TIMEOUT = 1 << 6, 1 << 7, 1 << 8, 1 << 9, 1 << 10
IRQ_ERROR = 1 << 23
RESET_NONE, RESET_POR, RESET_NRESET, RESET_NSS_WAKE, RESET_RTC = 0, 1, 2, 5, 6
BW_KHZ = {0x00: 7.8, 0x08: 10.4, 0x01: 15.6, 0x09: 20.8, 0x02: 31.25, 0x0A: 41.7, 0x03: 62.5,
          0x04: 125.0, 0x05: 250.0, 0x06: 500.0}
CR_DENOM = {1: 5, 2: 6, 3: 7, 4: 8, 5: 5, 6: 6, 7: 8}

# Durées (µs) — voir le tableau du docstring
BUSY_US = 45                    # commande simple
XOSC_START_US = 5000            # TCXO configuré : STBY_RC → XOSC
XTAL_START_US = 43              # sans TCXO (UM tableau 2-15)
CALIB_IMAGE_US, CALIBRATE_US = 6500, 31000
RESET_US = 237_000
WAKE_WARM_US, WAKE_COLD_US = 640, 13_300
DUTY_RESTART_US = 5200          # fin de sommeil du cycle : redémarrage (TCXO) avant la fenêtre suivante
TX_RAMP_US = 142                # STBY_XOSC → TX (UM tableau 2-15)

# Commandes dont la réponse se lit dans la transaction suivante : opcode -> fonction(params) -> octets
READ_CMDS = {0x0101, 0x010D, 0x0106, 0x010A, 0x0119, 0x011A, 0x0120, 0x0125, 0x0126, 0x0127, 0x0201, 0x0202,
             0x0203, 0x0204, 0x0205, 0x0230, 0x0320, 0x0406, 0x0407, 0x0123}
# Réglages acceptés sans effet sur le modèle
NO_EFFECT = {0x010E, 0x0116, 0x012A, 0x0128, 0x0208, 0x0212, 0x0215, 0x021B, 0x0224, 0x0225, 0x0229, 0x0206,
             0x020D}
STBY_RC_ONLY = {0x0110, 0x0112, 0x0117}     # CMD_FAIL hors STBY_RC


def time_on_air_us(n, sf, bw_khz, cr, preamble=16, explicit=True, crc=True, ldro=None):
    """Durée d'émission LoRa (µs), formule de la note d'application Semtech AN1200.13."""
    tsym = (1 << sf) / (bw_khz * 1000.0)
    if ldro is None:
        ldro = tsym * 1000 >= 16.0
    de = 1 if ldro else 0
    ih = 0 if explicit else 1
    num = 8 * n - 4 * sf + 28 + 16 * (1 if crc else 0) - 20 * ih
    den = 4 * (sf - 2 * de)
    payload_sym = 8 + max(math.ceil(num / den) * (CR_DENOM.get(cr, 5)), 0)
    t = (preamble + 4.25 + payload_sym) * tsym
    return int(t * 1e6)


class LR1110:
    def __init__(self, tw, gpio, spi, nss=6, nreset=12, busy=13, irq_pin=0x21, irq_inverted=True,
                 fw=(0x03, 0x07), name=None):
        self.tw, self.gpio = tw, gpio
        self.name = name or f'{tw.name}.lr1110'
        self.busy_pin, self.irq_pin, self.irq_inv, self.nreset = busy, irq_pin, irq_inverted, nreset
        self.fw = fw
        self.rng = __import__('random').Random(f'{tw.seed}-{tw.name}-lr1110')
        self.eui = bytes(self.rng.randrange(256) for _ in range(8))
        self.ether = None
        self.unknown_cmds = []
        self.violations = []                # protocole non respecté (commande perdue…) : (cycles, texte)
        self.commands = []                  # (cycles, opcode, paramètres) — trace complète
        self.lost = []                      # (cycles, opcode, raison) : transactions ignorées par la puce
        self.tx_packets = []
        self.rx_packets = []
        self.rx_missed = 0                  # trames manquées pendant le sommeil du cycle d'écoute
        self.duty_runs = 0                  # passages en écoute cyclique
        self.trace_cmds = False
        self.noise_floor = -110.0
        self.busy = False
        self.busy_ev = None
        self._power_on_reset()
        self.reset_status = RESET_POR
        spi.attach(nss, self)
        gpio.watch(nreset, self._on_nreset)
        gpio.drive(busy, 0)
        self._update_dio(force=True)

    # ------------------------------------------------------------------ état
    def _power_on_reset(self):
        """Contexte perdu (reset, sommeil froid) ; l'éther et les compteurs de test sont conservés."""
        self.mode = MODE_STBY_RC
        self.in_reset = False
        self.irq = 0
        self.dio_mask = 0
        self.cmd_status = CMD_OK
        self.pending = None                 # réponse à lire
        self.packet_type = 0
        self.freq = 915_000_000
        self.sf, self.bw, self.cr, self.ldro = 7, 0x04, 1, 0
        self.preamble, self.explicit, self.payload_len, self.crc_on, self.invert_iq = 8, True, 255, True, False
        self.sync_word = 0x12
        self.power, self.boosted = 0, False
        self.tcxo = None
        self.fallback = MODE_STBY_RC
        self.stop_on_preamble = False
        self.tx_buf = b''
        self.rx_buf = bytearray(256)
        self.rx_len, self.rx_off = 0, 0
        self.pkt_rssi, self.pkt_snr = -80.0, 8.0
        self.regs = {}
        self.op_ev = None
        self.rx_timeout_ev = None
        self.rx_continuous = False
        self.rx_single = False
        self.receiving = None               # paquet en cours de réception (verrouillé sur son préambule)
        self.duty = None                    # écoute cyclique : {'rx', 'sl', 't0'} en cycles CPU
        self.duty_ev = None
        self.retention = True               # dernier SetSleep (pour le réveil)
        self.stats = [0, 0, 0, 0]
        self._sel = False

    def params(self):
        return dict(freq=self.freq, sf=self.sf, bw=self.bw, cr=self.cr, sync=self.sync_word,
                    invert_iq=self.invert_iq, preamble=self.preamble, explicit=self.explicit,
                    crc=self.crc_on, ldro=self.ldro)

    def _xosc_us(self):
        """Temps de démarrage de l'oscillateur 32 MHz depuis STBY_RC (0 s'il tourne déjà)."""
        if self.mode in (MODE_STBY_XOSC, MODE_FS, MODE_RX, MODE_TX):
            return 0
        return XOSC_START_US if self.tcxo else XTAL_START_US

    # ------------------------------------------------------------------ broches
    def _set_busy(self, level, duration_us=None):
        self.tw.cancel(self.busy_ev)
        self.busy_ev = None
        if not level and self.mode == MODE_SLEEP:
            level = 1                                   # en sommeil (y compris celui du cycle) : BUSY haut
        self.busy = bool(level)
        self.gpio.drive(self.busy_pin, 1 if level else 0)
        if level and duration_us is not None:
            self.busy_ev = self.tw.schedule(duration_us * 64, self._set_busy, 0)

    def _update_dio(self, force=False):
        active = bool(self.irq & self.dio_mask)
        level = (0 if active else 1) if self.irq_inv else (1 if active else 0)
        if force or self.gpio.ext.get(self.irq_pin) != level:
            self.gpio.drive(self.irq_pin, level)

    def _raise_irq(self, bits):
        self.irq |= bits
        self._update_dio()

    def _on_nreset(self, level):
        if not level:
            self.in_reset = True
            self._abort_radio()
            self._set_busy(1)
        elif self.in_reset:
            self._power_on_reset()
            self.reset_status = RESET_NRESET
            self._update_dio()
            self._set_busy(1, duration_us=RESET_US)     # démarrage du firmware interne
            self.tw.log('lr1110_reset')

    # ------------------------------------------------------------------ SPI
    def select(self):
        self.mosi = bytearray()
        self.out = bytearray()
        self._reading = False
        if self.in_reset:
            self.violations.append((self.tw.now(), 'NSS bas pendant le reset'))
            self._sel = 'lost'
            return
        if self.mode == MODE_SLEEP:
            # Front descendant de NSS : réveil (redémarrage du firmware) ; la transaction elle-même est perdue.
            in_duty = self.duty is not None
            self._duty_stop()
            cold = not self.retention and not in_duty
            if cold:
                self._power_on_reset()
                self.irq |= IRQ_ERROR
            self.mode = MODE_STBY_RC
            self.reset_status = RESET_NSS_WAKE
            self._sel = 'wake'
            self._set_busy(1, duration_us=WAKE_COLD_US if cold else WAKE_WARM_US)
            self.tw.log('lr1110_wake', duty=in_duty, cold=cold)
            return
        if self.busy:
            self._sel = 'lost'                           # mesuré : commande ignorée
            return
        self._sel = True
        stat1 = ((CMD_DAT if self.pending is not None else self.cmd_status) << 1) | (1 if self.irq else 0)
        stat2 = (self.reset_status << 4) | (self.mode << 1) | 1
        if self.pending is not None:
            self.out = bytearray([stat1]) + self.pending
            self.pending = None
            self._reading = True
        else:
            self.out = bytearray([stat1, stat2]) + struct.pack('>I', self.irq)

    def xfer(self, data):
        n = len(self.mosi)
        self.mosi += data
        if self._sel is not True:
            return bytes(len(data))
        chunk = self.out[n:n + len(data)]
        return bytes(chunk) + bytes(len(data) - len(chunk))

    def deselect(self):
        sel, self._sel = self._sel, False
        if not sel:
            return
        op = self.mosi[0] << 8 | self.mosi[1] if len(self.mosi) >= 2 else 0
        if sel in ('wake', 'lost'):
            if op != 0x0000:                             # NOP (réveil voulu) : rien de perdu
                why = 'réveil' if sel == 'wake' else 'BUSY haut'
                self.lost.append((self.tw.now(), op, why))
                self.violations.append((self.tw.now(), f'commande {op:04x} perdue ({why})'))
            return
        if self._reading or len(self.mosi) < 2 or op == 0x0000:
            return
        self._command(op, bytes(self.mosi[2:]))

    # ------------------------------------------------------------------ commandes
    def _command(self, op, p):
        tw = self.tw
        self.commands.append((tw.now(), op, p))
        if self.trace_cmds:
            tw.log('lr1110_cmd', op=f'{op:04x}', p=p.hex())
        self.cmd_status = CMD_OK
        busy_us = BUSY_US
        resp = None
        u32 = lambda o=0: struct.unpack_from('>I', p, o)[0] if len(p) >= o + 4 else 0
        if op in STBY_RC_ONLY and self.mode != MODE_STBY_RC:
            self.cmd_status = CMD_FAIL
            self._set_busy(1, duration_us=busy_us)
            return
        if op == 0x0101:
            resp = bytes([0x22, 0x01]) + bytes(self.fw)
        elif op == 0x0320:
            resp = b'\x01\x03'
        elif op == 0x0406:
            resp = b'\x02\x01'
        elif op == 0x0407:
            resp = b'\x03'
        elif op == 0x0100:                              # GetStatus : efface l'état de reset (UM §3.4.1)
            self.reset_status = RESET_NONE
        elif op == 0x010D:
            resp = b'\x00\x00'
        elif op in NO_EFFECT or op == 0x0110 or op == 0x0112:
            pass
        elif op == 0x0213:                              # SetRxTxFallbackMode
            self.fallback = {1: MODE_STBY_RC, 2: MODE_STBY_XOSC, 3: MODE_FS}.get(p[0] if p else 1, MODE_STBY_RC)
        elif op == 0x0217:                              # StopTimeoutOnPreamble
            self.stop_on_preamble = bool(p and p[0])
        elif op == 0x0214:                              # SetRxDutyCycle(rxPeriod, sleepPeriod, mode), pas de 30,52 µs
            if self.packet_type == 0:
                self.cmd_status = CMD_FAIL
            else:
                busy_us = self._xosc_us() + BUSY_US
                rx_us = int.from_bytes(p[0:3], 'big') * 1_000_000 // 32768
                sl_us = int.from_bytes(p[3:6], 'big') * 1_000_000 // 32768
                self._abort_radio()
                self.mode = MODE_RX
                self.rx_continuous = self.rx_single = False
                t0 = tw.now() + busy_us * 64
                self.duty = {'rx': rx_us * 64, 'sl': (sl_us + DUTY_RESTART_US) * 64, 't0': t0}
                self.duty_runs += 1
                tw.log('lr1110_rx_duty', rx_ms=rx_us / 1000, sleep_ms=sl_us / 1000)
                self.duty_ev = tw.schedule(busy_us * 64, self._duty_tick)
        elif op == 0x010F:
            busy_us = CALIBRATE_US
            self.mode = MODE_STBY_RC
        elif op == 0x0111:
            busy_us = CALIB_IMAGE_US
            self.mode = MODE_STBY_RC
        elif op == 0x0113:
            self.dio_mask = u32(0)
            self._update_dio()
        elif op == 0x0114:
            self.irq &= ~u32(0)
            self._update_dio()
        elif op == 0x0117:
            self.tcxo = (p[0] if p else 0, u32(0) & 0xFFFFFF if len(p) >= 4 else 0)
        elif op == 0x0118:
            self._abort_radio()
            self._power_on_reset()
            busy_us = RESET_US
        elif op == 0x0119:
            resp = bytes([0x9C])
        elif op == 0x011A:
            resp = struct.pack('>H', 0x0300)
        elif op == 0x011B:                              # SetSleep(config, time)
            self._abort_radio()
            self.retention = bool(p and p[0] & 1)
            self.mode = MODE_SLEEP
            self.tw.log('lr1110_sleep', retention=self.retention)
            self._set_busy(1)
            return
        elif op == 0x011C:                              # SetStandby : met aussi fin au cycle d'écoute
            busy_us = (self._xosc_us() if p and p[0] else 0) + BUSY_US
            self._abort_radio()
            self.mode = MODE_STBY_XOSC if p and p[0] else MODE_STBY_RC
        elif op == 0x011D:
            busy_us = self._xosc_us() + 50
            self._abort_radio()
            self.mode = MODE_FS
        elif op == 0x0120:
            resp = bytes(self.rng.randrange(256) for _ in range(4))
        elif op in (0x0125, 0x0126):
            resp = self.eui
        elif op == 0x0127:
            resp = bytes(4)
        elif op == 0x0105:                              # WriteRegMem32
            a = u32(0)
            for i in range(4, len(p) - 3, 4):
                self.regs[a + i - 4] = u32(i)
        elif op == 0x010C:                              # WriteRegMemMask32
            a, m, v = u32(0), u32(4), u32(8)
            self.regs[a] = (self.regs.get(a, 0) & ~m) | (v & m)
        elif op == 0x0106:                              # ReadRegMem32
            a, n = u32(0), p[4] if len(p) > 4 else 1
            resp = b''.join(struct.pack('>I', self.regs.get(a + 4 * i, 0)) for i in range(n))
        elif op == 0x0109:                              # WriteBuffer8 (tampon TX)
            self.tx_buf = p
        elif op == 0x010A:                              # ReadBuffer8 (tampon RX)
            off, n = (p[0], p[1]) if len(p) >= 2 else (0, 0)
            resp = bytes(self.rx_buf[off:off + n])
        elif op == 0x010B:
            self.rx_buf = bytearray(256)
        elif op == 0x0200:
            self.stats = [0, 0, 0, 0]
        elif op == 0x0201:
            resp = struct.pack('>HHHH', *self.stats)
        elif op == 0x0202:
            resp = bytes([self.packet_type])
        elif op == 0x0203:
            resp = bytes([self.rx_len, self.rx_off])
        elif op == 0x0204:
            r = min(255, max(0, int(-self.pkt_rssi * 2)))
            resp = bytes([r, int(round(self.pkt_snr * 4)) & 0xFF, r])
        elif op == 0x0205:
            resp = bytes([min(255, int(-self.rssi_now() * 2))])
        elif op == 0x0209:
            if self.packet_type == 0:
                self.cmd_status = CMD_FAIL
            else:
                busy_us = self._xosc_us() + BUSY_US
                self._start_rx((p[0] << 16 | p[1] << 8 | p[2]) if len(p) >= 3 else 0)
        elif op == 0x020A:
            if self.packet_type == 0:
                self.cmd_status = CMD_FAIL
            else:
                busy_us = self._xosc_us() + TX_RAMP_US     # BUSY bas une fois le PA monté (UM §2.1.11)
                self._start_tx(busy_us)
        elif op == 0x020B:
            self.freq = u32(0)
        elif op == 0x020E:
            if self.mode not in (MODE_STBY_RC, MODE_STBY_XOSC, MODE_FS):
                self.cmd_status = CMD_FAIL
            else:
                self.packet_type = p[0] if p else 0
        elif op == 0x020F:
            if len(p) >= 4:
                self.sf, self.bw, self.cr, self.ldro = p[0], p[1], p[2], p[3]
        elif op == 0x0210:
            if len(p) >= 6:
                self.preamble = p[0] << 8 | p[1]
                self.explicit = p[2] == 0
                self.payload_len, self.crc_on, self.invert_iq = p[3], bool(p[4]), bool(p[5])
        elif op == 0x0211:
            if p:
                self.power = struct.unpack('b', p[:1])[0]
        elif op == 0x0218:
            self._start_cad()
        elif op == 0x0227:
            self.boosted = bool(p and p[0])
        elif op == 0x022B:
            self.sync_word = p[0] if p else 0x12
        elif op == 0x0230:
            resp = bytes([(0x10 if self.crc_on else 0) | (self.cr & 7)])
        else:
            self.cmd_status = CMD_PERR
            self.unknown_cmds.append((op, p))
            self.tw.log('lr1110_unknown_cmd', op=f'{op:04x}', params=p.hex())
        if op in READ_CMDS:
            self.pending = bytearray(resp or b'')
        self._set_busy(1, duration_us=busy_us)

    # ------------------------------------------------------------------ radio
    def toa_us(self, n):
        return time_on_air_us(n, self.sf, BW_KHZ.get(self.bw, 125.0), self.cr, self.preamble, self.explicit,
                              self.crc_on, bool(self.ldro))

    def symbol_us(self):
        return (1 << self.sf) / (BW_KHZ.get(self.bw, 125.0) * 1000.0) * 1e6

    def rssi_now(self):
        if self.ether:
            fr = self.ether.on_air(self)
            if fr:
                return self.ether.rssi_at(self)
        return self.noise_floor + self.rng.uniform(-3.0, 3.0)      # bruit propre à chaque radio

    def _abort_radio(self):
        self._duty_stop()
        self.tw.cancel(self.op_ev)
        self.tw.cancel(self.rx_timeout_ev)
        self.op_ev = self.rx_timeout_ev = None
        if self.mode == MODE_TX and self.ether:
            self.ether.abort(self)
        self.receiving = None

    def _start_tx(self, ramp_us):
        self._abort_radio()
        self.mode = MODE_TX
        pkt = bytes(self.tx_buf[:self.payload_len] if not self.explicit else self.tx_buf)
        dur = self.toa_us(len(pkt))
        self.tx_packets.append({'t': self.tw.now(), 'data': pkt, 'toa_us': dur, 'power': self.power, **self.params()})
        self.tw.log('lora_tx', len=len(pkt), toa_ms=dur / 1000, freq=self.freq, sf=self.sf, power=self.power)

        def on_air():
            if self.ether:
                self.ether.transmit(self, pkt, dur)
            self.op_ev = self.tw.schedule(dur * 64, self._tx_done)
        self.op_ev = self.tw.schedule(ramp_us * 64, on_air)          # préambule émis après la montée du PA

    def _tx_done(self):
        self.op_ev = None
        self.mode = self.fallback
        self._raise_irq(IRQ_TX_DONE)

    def _start_rx(self, timeout):
        self._abort_radio()
        self.mode = MODE_RX
        self.rx_continuous = timeout == 0xFFFFFF
        self.rx_single = timeout == 0
        if timeout not in (0, 0xFFFFFF):
            self.rx_timeout_ev = self.tw.schedule(timeout * 64_000_000 // 32768, self._rx_timeout)

    def _rx_timeout(self):
        self.rx_timeout_ev = None
        if self.mode == MODE_RX and not self.receiving:
            self.mode = MODE_STBY_RC                     # toujours STBY_RC (UM §7.2.5)
            self._raise_irq(IRQ_TIMEOUT)

    def _start_cad(self):
        self._abort_radio()
        self.mode = MODE_RX
        busy = self.ether.channel_busy(self) if self.ether else False
        dur = int(self.symbol_us() * 2)

        def done():
            self.op_ev = None
            self.mode = MODE_STBY_RC
            self._raise_irq(IRQ_CAD_DONE | (IRQ_CAD_DETECTED if busy else 0))
        self.op_ev = self.tw.schedule(dur * 64, done)

    # ------------------------------------------------------------------ écoute cyclique (SetRxDutyCycle)
    def duty_awake(self, t=None):
        d = self.duty
        if d is None:
            return True
        t = self.tw.now() if t is None else t
        return (t - d['t0']) % (d['rx'] + d['sl']) < d['rx']

    @property
    def duty_share(self):
        d = self.duty
        return d['rx'] / (d['rx'] + d['sl']) if d else 1.0

    def _duty_tick(self):
        """Début et fin de chaque fenêtre d'écoute : RX (BUSY bas) ou sommeil avec rétention (BUSY haut)."""
        self.duty_ev = None
        d = self.duty
        if d is None or self.receiving is not None:
            return
        now = self.tw.now()
        ph = (now - d['t0']) % (d['rx'] + d['sl'])
        awake = ph < d['rx']
        if awake and self.mode == MODE_SLEEP:
            self.reset_status = RESET_RTC                # réveil par la RTC (mesuré)
        self.mode = MODE_RX if awake else MODE_SLEEP
        if not self.busy_ev:
            self._set_busy(0 if awake else 1)
        nxt = (d['rx'] - ph) if awake else (d['rx'] + d['sl'] - ph)
        self.duty_ev = self.tw.schedule(max(1, nxt), self._duty_tick)

    def _duty_stop(self):
        if self.duty is None:
            return
        self.tw.cancel(self.duty_ev)
        self.duty_ev = None
        self.duty = None

    def _duty_catch(self, frame):
        """Instant (cycles) où une fenêtre d'écoute a vu 8 symboles du préambule, ou None (trame manquée)."""
        d = self.duty
        sym = int(self.symbol_us() * 64)
        t0f = self.tw.now()
        t_end = t0f + int(frame['preamble'] * sym)
        need = 8 * sym
        c = d['rx'] + d['sl']
        k = (t0f - d['t0']) // c - 1
        while True:
            ws = d['t0'] + k * c
            if ws > t_end:
                return None
            ov = min(ws + d['rx'], t_end) - max(ws, t0f)
            if ov >= need:
                return max(ws, t0f) + need
            k += 1

    # Appelés par l'éther (dans le temps de ce jumeau)
    def air_start(self, frame):
        """Début d'un paquet sur l'air, reçu avec frame['rssi'] / frame['snr']."""
        if self.mode not in (MODE_RX, MODE_SLEEP) or self.receiving is not None:
            if self.receiving is not None and frame['rssi'] > self.receiving['rssi'] - 6:
                self.receiving['collided'] = True          # pas d'effet de capture : paquet perdu
            return
        if self.mode == MODE_SLEEP and self.duty is None:
            return
        if not self._compatible(frame):
            return
        if self.duty is not None:
            if self._duty_catch(frame) is None:
                self.rx_missed += 1
                self.tw.log('lora_rx_missed', len=len(frame['data']), preamble=frame['preamble'])
                return
            # verrouillée sur le préambule : reste en RX jusqu'à la fin de la trame (UM §7.2.6)
            self.tw.cancel(self.duty_ev)
            self.duty_ev = None
            self.mode = MODE_RX
            if not self.busy_ev:
                self._set_busy(0)
        rx = {'frame': frame, 'rssi': frame['rssi'], 'collided': False, 'header': False}
        self.receiving = rx
        sym = self.symbol_us()
        t_pre = int((min(frame['preamble'], self.preamble) + 4.25) * sym)
        t_hdr = t_pre + int(8 * sym)

        def preamble():
            if self.receiving is rx:
                if self.stop_on_preamble:
                    self.tw.cancel(self.rx_timeout_ev)
                self._raise_irq(IRQ_PREAMBLE)
        def header():
            if self.receiving is rx:
                rx['header'] = True
                self.tw.cancel(self.rx_timeout_ev)
                self._raise_irq(IRQ_HEADER_VALID)
        self.tw.schedule(t_pre * 64, preamble)
        self.tw.schedule(t_hdr * 64, header)
        if self.duty is not None:
            d = self.duty

            def lock_timeout():                         # 2 × rx + sommeil sans en-tête : retour au cycle
                if self.receiving is rx and not rx['header'] and self.duty is d:
                    self.receiving = None
                    self._duty_tick()
            self.tw.schedule(t_pre * 64 + 2 * d['rx'] + d['sl'], lock_timeout)

    def air_end(self, frame):
        rx = self.receiving
        if rx is None or rx['frame'] is not frame:
            return
        self.receiving = None
        data = frame['data']
        self.rx_buf[0:len(data)] = data
        self.rx_len, self.rx_off = len(data), 0
        self.pkt_rssi, self.pkt_snr = frame['rssi'], frame['snr']
        bad = rx['collided'] or frame.get('corrupt')
        self.stats[0] += 1
        if bad:
            self.stats[1] += 1
        self.rx_packets.append({'t': self.tw.now(), 'data': data, 'ok': not bad, 'rssi': frame['rssi'], 'snr': frame['snr']})
        self.tw.log('lora_rx', len=len(data), ok=not bad, rssi=frame['rssi'])
        if self.duty is not None:
            self._duty_stop()                            # fin du cycle, mode de repli (mesuré : STBY_RC)
            self.mode = self.fallback
            if not self.busy_ev:
                self._set_busy(0)
        elif not self.rx_continuous:
            self.mode = self.fallback
            self.tw.cancel(self.rx_timeout_ev)
        self._raise_irq(IRQ_RX_DONE | (IRQ_CRC_ERR if bad else 0))

    def _compatible(self, f):
        return (abs(f['freq'] - self.freq) < 1000 and f['sf'] == self.sf and f['bw'] == self.bw
                and f['sync'] == self.sync_word and f['invert_iq'] == self.invert_iq and self.packet_type == 2)
