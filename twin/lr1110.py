"""Modèle comportemental de l'émetteur-récepteur Semtech LR1110 (partie radio LoRa).

Interface électrique (T1000-A) : SPI (NSS = GPIO6), NRESET = GPIO12, BUSY = GPIO13 (sortie du LR1110),
DIO9 (ligne d'IRQ) → inverseur à collecteur ouvert → AON1 : le GR5515 voit AON1 = NOT irq.

Protocole SPI (fiche technique LR1110, § 3) :
  * pendant toute transaction, le LR1110 renvoie Stat1, Stat2, Irq[31:0] (6 octets), puis des zéros ;
  * une commande à réponse se lit dans une 2ᵉ transaction (NOP) : Stat1 (CmdStatus = DAT) puis les données ;
  * BUSY monte à la fin de chaque commande, le temps de la traiter ; en veille, BUSY reste haut et un
    front descendant de NSS réveille la puce.

Ce qui est modélisé : états (veille, standby, FS, RX, TX), paramètres LoRa, tampons TX/RX, IRQ et DIO,
temps d'antenne exact (formule Semtech), RSSI/SNR fournis par l'éther, nombres aléatoires, registres
mémoire (valeurs écrites relues). Les commandes inconnues sont acceptées et journalisées
(`unknown_cmds`) : un test peut exiger qu'il n'y en ait aucune.
"""
import math, struct

CMD_OK, CMD_DAT, CMD_PERR, CMD_FAIL = 2, 3, 1, 0
MODE_SLEEP, MODE_STBY_RC, MODE_STBY_XOSC, MODE_FS, MODE_RX, MODE_TX = 0, 1, 2, 3, 4, 5
IRQ_TX_DONE, IRQ_RX_DONE, IRQ_PREAMBLE, IRQ_HEADER_VALID = 1 << 2, 1 << 3, 1 << 4, 1 << 5
IRQ_HEADER_ERR, IRQ_CRC_ERR, IRQ_CAD_DONE, IRQ_CAD_DETECTED, IRQ_TIMEOUT = 1 << 6, 1 << 7, 1 << 8, 1 << 9, 1 << 10
BW_KHZ = {0x00: 7.8, 0x08: 10.4, 0x01: 15.6, 0x09: 20.8, 0x02: 31.25, 0x0A: 41.7, 0x03: 62.5,
          0x04: 125.0, 0x05: 250.0, 0x06: 500.0}
CR_DENOM = {1: 5, 2: 6, 3: 7, 4: 8, 5: 5, 6: 6, 7: 8}

# Commandes dont la réponse se lit dans la transaction suivante : opcode -> fonction(params) -> octets
READ_CMDS = {0x0101, 0x010D, 0x0106, 0x010A, 0x0119, 0x011A, 0x0120, 0x0125, 0x0126, 0x0127, 0x0201, 0x0202,
             0x0203, 0x0204, 0x0205, 0x0230, 0x0320, 0x0406, 0x0407, 0x0123}


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
                 fw=(0x04, 0x01), name=None):
        self.tw, self.gpio = tw, gpio
        self.name = name or f'{tw.name}.lr1110'
        self.busy_pin, self.irq_pin, self.irq_inv, self.nreset = busy, irq_pin, irq_inverted, nreset
        self.fw = fw
        self.rng = __import__('random').Random(f'{tw.seed}-{tw.name}-lr1110')
        self.eui = bytes(self.rng.randrange(256) for _ in range(8))
        self.ether = None
        self.unknown_cmds = []
        self.violations = []                # BUSY non respecté, commande pendant le reset…
        self.commands = []                  # (cycles, opcode, paramètres) — trace complète
        self.tx_packets = []
        self.rx_packets = []
        self.trace_cmds = False
        self.noise_floor = -110.0
        self._power_on_reset()
        spi.attach(nss, self)
        gpio.watch(nreset, self._on_nreset)
        gpio.drive(busy, 0)
        self._update_dio(force=True)

    # ------------------------------------------------------------------ état
    def _power_on_reset(self):
        self.mode = MODE_STBY_RC
        self.in_reset = False
        self.reset_status = 1
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
        self.tx_buf = b''
        self.rx_buf = bytearray(256)
        self.rx_len, self.rx_off = 0, 0
        self.pkt_rssi, self.pkt_snr = -80.0, 8.0
        self.regs = {}
        self.busy_ev = self.op_ev = None
        self.busy = False
        self.rx_timeout_ev = None
        self.rx_continuous = False
        self.receiving = None               # paquet en cours de réception (verrouillé sur son préambule)
        self.stats = [0, 0, 0, 0]
        self._sel = False

    def params(self):
        return dict(freq=self.freq, sf=self.sf, bw=self.bw, cr=self.cr, sync=self.sync_word,
                    invert_iq=self.invert_iq, preamble=self.preamble, explicit=self.explicit,
                    crc=self.crc_on, ldro=self.ldro)

    # ------------------------------------------------------------------ broches
    def _set_busy(self, level, duration_us=None):
        self.tw.cancel(self.busy_ev)
        self.busy_ev = None
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
            self.reset_status = 2                       # reset externe (NRESET)
            self._update_dio()
            self._set_busy(1, duration_us=3000)         # démarrage du firmware interne
            self.tw.log('lr1110_reset')

    # ------------------------------------------------------------------ SPI
    def select(self):
        if self.in_reset:
            self.violations.append((self.tw.now(), 'NSS bas pendant le reset'))
        if self.mode == MODE_SLEEP:
            self._sel = 'wake'
            self.mode = MODE_STBY_RC
            self._set_busy(1, duration_us=500)          # réveil
            self.tw.log('lr1110_wake')
            self.out = bytearray()
            self.mosi = bytearray()
            return
        if self.busy:
            self.violations.append((self.tw.now(), 'NSS bas alors que BUSY est haut'))
        self._sel = True
        stat1 = ((CMD_DAT if self.pending is not None else self.cmd_status) << 1) | (1 if self.irq else 0)
        stat2 = (self.reset_status << 4) | (self.mode << 1) | 1
        if self.pending is not None:
            self.out = bytearray([stat1]) + self.pending
            self.pending = None
            self._reading = True
        else:
            self.out = bytearray([stat1, stat2]) + struct.pack('>I', self.irq)
            self._reading = False
        self.mosi = bytearray()

    def xfer(self, data):
        if not self._sel:
            return bytes(len(data))
        n = len(self.mosi)
        self.mosi += data
        chunk = self.out[n:n + len(data)]
        return bytes(chunk) + bytes(len(data) - len(chunk))

    def deselect(self):
        if not self._sel:
            return
        sel, self._sel = self._sel, False
        if sel == 'wake' or self._reading:
            return
        if len(self.mosi) < 2:
            return
        op = self.mosi[0] << 8 | self.mosi[1]
        if op == 0x0000:                                # NOP : lecture d'état seulement
            return
        self._command(op, bytes(self.mosi[2:]))

    # ------------------------------------------------------------------ commandes
    def _command(self, op, p):
        tw = self.tw
        self.commands.append((tw.now(), op, p))
        if self.trace_cmds:
            tw.log('lr1110_cmd', op=f'{op:04x}', p=p.hex())
        self.cmd_status = CMD_OK
        busy_us = 20
        resp = None
        u32 = lambda o=0: struct.unpack_from('>I', p, o)[0] if len(p) >= o + 4 else 0
        if op == 0x0101:
            resp = bytes([0x22, 0x01]) + bytes(self.fw)
        elif op == 0x0320:
            resp = b'\x01\x03'
        elif op == 0x0406:
            resp = b'\x02\x01'
        elif op == 0x0407:
            resp = b'\x03'
        elif op == 0x0100:                              # GetStatus : efface l'état de reset
            self.reset_status = 0
        elif op == 0x010D:
            resp = b'\x00\x00'
        elif op in (0x010E, 0x0110, 0x0112, 0x0116, 0x012A, 0x0128, 0x0208, 0x0212, 0x0213, 0x0215, 0x0217,
                    0x021B, 0x0224, 0x0225, 0x0229, 0x0206, 0x020D, 0x0214):
            pass                                        # réglages sans effet sur le modèle
        elif op == 0x010F:
            busy_us = 5000                              # calibration complète
        elif op == 0x0111:
            busy_us = 2000
        elif op == 0x0113:
            self.dio_mask = u32(0)
            self._update_dio()
        elif op == 0x0114:
            self.irq &= ~u32(0)
            if not self.irq & IRQ_HEADER_VALID and self.receiving and self.receiving.get('locked'):
                pass
            self._update_dio()
        elif op == 0x0117:
            self.tcxo = (p[0] if p else 0, u32(0) & 0xFFFFFF if len(p) >= 4 else 0)
        elif op == 0x0118:
            self._power_on_reset()
            busy_us = 3000
        elif op == 0x0119:
            resp = bytes([0x9C])
        elif op == 0x011A:
            resp = struct.pack('>H', 0x0300)
        elif op == 0x011B:
            self._abort_radio()
            self.mode = MODE_SLEEP
            self.tw.log('lr1110_sleep')
            self._set_busy(1)
            return
        elif op == 0x011C:
            self._abort_radio()
            self.mode = MODE_STBY_XOSC if p and p[0] else MODE_STBY_RC
        elif op == 0x011D:
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
        elif op == 0x0109:                              # WriteBuffer8
            self.tx_buf = p
        elif op == 0x010A:                              # ReadBuffer8
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
            self._start_rx((p[0] << 16 | p[1] << 8 | p[2]) if len(p) >= 3 else 0)
        elif op == 0x020A:
            self._start_tx()
        elif op == 0x020B:
            self.freq = u32(0)
        elif op == 0x020E:
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
        self.tw.cancel(self.op_ev)
        self.tw.cancel(self.rx_timeout_ev)
        self.op_ev = self.rx_timeout_ev = None
        if self.mode == MODE_TX and self.ether:
            self.ether.abort(self)
        self.receiving = None

    def _start_tx(self):
        self._abort_radio()
        self.mode = MODE_TX
        pkt = bytes(self.tx_buf[:self.payload_len] if not self.explicit else self.tx_buf)
        dur = self.toa_us(len(pkt))
        self.tx_packets.append({'t': self.tw.now(), 'data': pkt, 'toa_us': dur, 'power': self.power, **self.params()})
        self.tw.log('lora_tx', len=len(pkt), toa_ms=dur / 1000, freq=self.freq, sf=self.sf, power=self.power)
        if self.ether:
            self.ether.transmit(self, pkt, dur)
        self.op_ev = self.tw.schedule(dur * 64 + 64 * 100, self._tx_done)    # + montée du PA

    def _tx_done(self):
        self.op_ev = None
        self.mode = MODE_STBY_RC
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
            self.mode = MODE_STBY_RC
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

    # Appelés par l'éther (dans le temps de ce jumeau)
    def air_start(self, frame):
        """Début d'un paquet sur l'air, reçu avec frame['rssi'] / frame['snr']."""
        if self.mode != MODE_RX or self.receiving is not None:
            if self.receiving is not None and frame['rssi'] > self.receiving['rssi'] - 6:
                self.receiving['collided'] = True          # pas d'effet de capture : paquet perdu
            return
        if not self._compatible(frame):
            return
        rx = {'frame': frame, 'rssi': frame['rssi'], 'collided': False}
        self.receiving = rx
        sym = self.symbol_us()
        t_pre = int((min(frame['preamble'], self.preamble) + 4.25) * sym)
        t_hdr = t_pre + int(8 * sym)

        def preamble():
            if self.receiving is rx:
                self._raise_irq(IRQ_PREAMBLE)
        def header():
            if self.receiving is rx:
                self.tw.cancel(self.rx_timeout_ev)
                self._raise_irq(IRQ_HEADER_VALID)
        self.tw.schedule(t_pre * 64, preamble)
        self.tw.schedule(t_hdr * 64, header)

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
        if not self.rx_continuous:
            self.mode = MODE_STBY_RC
        self._raise_irq(IRQ_RX_DONE | (IRQ_CRC_ERR if bad else 0))

    def _compatible(self, f):
        return (abs(f['freq'] - self.freq) < 1000 and f['sf'] == self.sf and f['bw'] == self.bw
                and f['sync'] == self.sync_word and f['invert_iq'] == self.invert_iq and self.packet_type == 2)
