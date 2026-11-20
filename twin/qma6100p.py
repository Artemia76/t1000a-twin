"""Accéléromètre QMA6100P (QST) du T1000-A, esclave I²C au niveau des broches (le firmware fait de l'I²C logiciel).

Câblage relevé dans le firmware Seeed v2.5 : alimentation GPIO7 (1 = allumé), SDA = GPIO8, SCL = GPIO9,
adresse 0x12. Registres modélisés : identifiant (0x00 = 0x90), données X/Y/Z (0x01..0x06, 14 bits cadrés à
gauche, bit 0 = nouvelle donnée), mode (0x11, bit 7 = actif), plage (0x0F), reset logiciel (0x36 = 0xB6).
"""

ADDR = 0x12
CHIP_ID = 0x90
LSB_PER_G = {0x01: 4096, 0x02: 2048, 0x04: 1024, 0x08: 512, 0x0F: 256}


class Qma6100p:
    def __init__(self, gpio, pwr=7, sda=8, scl=9):
        self.gpio, self.pwr, self.sda, self.scl = gpio, pwr, sda, scl
        self.accel_g = (0.0, 0.0, 1.0)          # posé à plat
        self.chip_id = CHIP_ID                  # autre valeur : puce différente ou absente
        self.transactions = []                  # (lecture?, registre, octets) pour les tests
        self._power_off()
        gpio.watch(scl, self._scl)
        gpio.watch(sda, self._sda)
        gpio.watch(pwr, lambda lvl: None if lvl else self._power_off())

    # ------------------------------------------------------------------ registres
    def _power_off(self):
        self.regs = bytearray(0x80)
        self.regs[0x00] = self.chip_id
        self.regs[0x0F] = 0x01                  # ±2 g par défaut
        self.state, self._drive, self.ptr = 'idle', None, 0
        self.gpio.drive(self.sda, None)

    def _powered(self):
        if self.gpio.level(self.pwr) != 1:
            return False
        self.regs[0x00] = self.chip_id
        return True

    def _sample(self):
        if not self.regs[0x11] & 0x80:          # pas en mode actif : pas de mesure
            return
        k = LSB_PER_G.get(self.regs[0x0F] & 0x0F, 4096)
        for i, g in enumerate(self.accel_g):
            v = max(-8192, min(8191, int(round(g * k))))
            raw = (v << 2) & 0xFFFF
            self.regs[1 + 2 * i] = (raw & 0xFC) | 1
            self.regs[2 + 2 * i] = raw >> 8

    def _write_reg(self, reg, val):
        if reg == 0x36 and val == 0xB6:
            self.regs = bytearray(0x80)
            self.regs[0x00], self.regs[0x0F] = self.chip_id, 0x01
            return
        if reg != 0x00:
            self.regs[reg & 0x7F] = val

    def _read_reg(self):
        if self.ptr == 0x01:
            self._sample()
        v = self.regs[self.ptr & 0x7F]
        if 1 <= self.ptr <= 6 and self.ptr % 2 == 0:
            self.regs[self.ptr - 1] &= 0xFE      # nouvelle donnée consommée
        self.ptr += 1
        return v

    # ------------------------------------------------------------------ protocole
    # États : rx (8 bits reçus, adresse ou donnée), ack_out (l'esclave tient l'ACK pendant la 9e impulsion),
    # tx (8 bits émis), ack_in (ACK / NACK du maître). L'esclave ne change SDA que sur un front descendant de SCL.
    def _put(self, bit):
        """Pose un bit sur SDA : 0 = tirée à la masse, 1 = relâchée (tirage vers le haut)."""
        self._drive = None if bit else 0
        self.gpio.drive(self.sda, self._drive)

    def _sda(self, lvl):
        if not self._powered() or self.gpio.level(self.scl) != 1:
            return
        if lvl == 0:                            # START (ou START répété)
            self.state, self.phase, self.bit, self.byte, self.nbytes = 'rx', 'addr', 0, 0, 0
        else:                                   # STOP
            self.state = 'idle'

    def _scl(self, lvl):
        if not self._powered() or self.state == 'idle':
            return
        if lvl:                                 # front montant : le récepteur échantillonne
            if self.state == 'rx':
                self.byte = (self.byte << 1 | self.gpio.level(self.sda)) & 0xFF
                self.bit += 1
            elif self.state == 'tx':
                self.bit += 1
            elif self.state == 'ack_in':
                self.master_ack = self.gpio.level(self.sda) == 0
            return
        if self.state == 'rx' and self.bit == 8:
            self._byte_received(self.byte)
        elif self.state == 'ack_out':
            if self.next == 'tx':
                self._send(self._read_reg())
            else:
                self._put(1)
                self.state, self.bit, self.byte = 'rx', 0, 0
        elif self.state == 'tx':
            if self.bit == 8:
                self._put(1)
                self.state = 'ack_in'
            else:
                self._put(self.cur >> (7 - self.bit) & 1)
        elif self.state == 'ack_in':
            if self.master_ack:
                self._send(self._read_reg())
            else:
                self._put(1)
                self.state = 'idle'

    def _send(self, b):
        self.cur, self.bit, self.state = b, 0, 'tx'
        self._put(b >> 7 & 1)

    def _byte_received(self, b):
        if self.phase == 'addr':
            if b >> 1 != ADDR:
                self.state = 'idle'
                return
            self.phase = 'data'
            self.next = 'tx' if b & 1 else 'rx'
            if b & 1:
                self.transactions.append(('read', self.ptr))
        else:
            if self.nbytes == 0:
                self.ptr = b
            else:
                self._write_reg(self.ptr, b)
                self.transactions.append(('write', self.ptr, b))
                self.ptr += 1
            self.nbytes += 1
            self.next = 'rx'
        self._put(0)                            # ACK
        self.state = 'ack_out'
