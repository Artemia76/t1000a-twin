"""Modèle LR1110 seul, piloté au niveau SPI (sans firmware) : chaque test reprend une mesure faite sur le boîtier
(sonde `lr`, 2026-11-16) ou une règle du manuel utilisateur LR1110 Rev 2.3. Voir le tableau de twin/lr1110.py.
"""
import heapq, struct
import pytest
from twin.lr1110 import (LR1110, MODE_SLEEP, MODE_STBY_RC, MODE_STBY_XOSC, MODE_FS, MODE_RX, IRQ_RX_DONE,
                         IRQ_TX_DONE, IRQ_TIMEOUT, IRQ_ERROR, CMD_OK, CMD_PERR, CMD_FAIL)

US = 64                                         # cycles par µs (horloge virtuelle du jumeau)


class FakeTwin:
    name, seed = 'test', 0

    def __init__(self):
        self.t, self.heap, self.seq, self.logs = 0, [], 0, []

    def now(self):
        return self.t

    def schedule(self, delay, fn, *args):
        self.seq += 1
        ev = [self.t + max(0, int(delay)), self.seq, fn, args, True]
        heapq.heappush(self.heap, ev)
        return ev

    @staticmethod
    def cancel(ev):
        if ev:
            ev[4] = False

    def log(self, kind, **data):
        self.logs.append((self.t, kind, data))

    def advance(self, us):
        end = self.t + int(us * US)
        while self.heap and self.heap[0][0] <= end:
            ev = heapq.heappop(self.heap)
            if ev[4]:
                self.t = ev[0]
                ev[2](*ev[3])
        self.t = end


class FakeGpio:
    def __init__(self):
        self.ext, self.watchers = {}, {}

    def drive(self, pin, level):
        self.ext[pin] = level

    def watch(self, pin, fn):
        self.watchers[pin] = fn

    def level(self, pin):
        return self.ext.get(pin, 0)


class FakeSpi:
    def attach(self, cs, dev):
        self.dev = dev


class Host:
    """Un pilote minimal : comme RadioLib, il attend BUSY bas avant chaque commande (sauf demande contraire)."""

    def __init__(self):
        self.tw, self.gpio, self.spi = FakeTwin(), FakeGpio(), FakeSpi()
        self.r = LR1110(self.tw, self.gpio, self.spi)

    def busy(self):
        return bool(self.gpio.level(13))

    def wait_busy(self, limit_us=1_000_000):
        """Attend BUSY bas ; renvoie la durée (µs)."""
        t0 = self.tw.t
        while self.busy():
            assert self.tw.t - t0 < limit_us * US, 'BUSY reste haut'
            self.tw.advance(10)
        return (self.tw.t - t0) / US

    def xfer(self, data, wait=True):
        if wait:
            self.wait_busy()
        self.r.select()
        out = self.r.xfer(bytes(data))
        self.r.deselect()
        return out

    def cmd(self, op, *params, wait=True):
        return self.xfer(bytes([op >> 8, op & 0xFF, *params]), wait)

    def read(self, op, n, *params):
        self.cmd(op, *params)
        return self.xfer(bytes(n + 1))

    def status(self):
        s = self.cmd(0x0100, 0, 0, 0, 0)
        return {'cmd': (s[0] >> 1) & 7, 'mode': (s[1] >> 1) & 7, 'reset': s[1] >> 4,
                'irq': struct.unpack('>I', s[2:6])[0]}

    def setup_lora(self, tcxo=True):
        if tcxo:
            self.cmd(0x0117, 6, 0, 0x01, 0x40)              # SetTcxoMode 3,0 V
        self.cmd(0x020E, 2)                                  # LoRa
        self.cmd(0x020F, 8, 0x03, 1, 0)                      # SF8 BW62.5 CR4/5
        self.cmd(0x0210, 0, 32, 0, 0xFF, 1, 0)               # préambule 32
        self.wait_busy()


@pytest.fixture
def h():
    h = Host()
    h.wait_busy()
    return h


def frame(h, data=b'x' * 40, preamble=32):
    r = h.r
    return {'freq': r.freq, 'sf': r.sf, 'bw': r.bw, 'sync': r.sync_word, 'invert_iq': r.invert_iq,
            'preamble': preamble, 'data': data, 'rssi': -60.0, 'snr': 9.0}


def receive(h, f):
    h.r.air_start(f)
    h.tw.advance(h.r.toa_us(len(f['data'])))
    h.r.air_end(f)


# ------------------------------------------------------------------ interface et BUSY
def test_version_mesuree(h):
    assert h.read(0x0101, 4)[1:5] == bytes([0x22, 0x01, 0x03, 0x07])


def test_busy_commande_simple_et_demarrage_tcxo(h):
    h.setup_lora()
    h.cmd(0x020E, 2)
    assert h.wait_busy() <= 60, 'commande simple : 30 à 60 µs'
    h.cmd(0x011C, 1)                                         # SetStandby XOSC depuis STBY_RC
    assert 4900 <= h.wait_busy() <= 5200, 'démarrage du TCXO ≈ 5 ms'
    h.cmd(0x011C, 0)
    h.cmd(0x0111, 0xD7, 0xDB)
    assert 6300 <= h.wait_busy() <= 6700, 'CalibImage ≈ 6,5 ms'


def test_commande_pendant_busy_perdue(h):
    h.cmd(0x0111, 0xD7, 0xDB)
    h.tw.advance(100)
    assert h.busy()
    h.cmd(0x011C, 1, wait=False)                             # SetStandby XOSC pendant BUSY : perdue
    h.wait_busy()
    assert h.status()['mode'] == MODE_STBY_RC
    assert h.r.lost and h.r.violations


def test_opcode_inconnu_cmd_perr(h):
    h.cmd(0x02FE)
    assert h.status()['cmd'] == CMD_PERR
    assert h.r.unknown_cmds


def test_reglages_hors_stby_rc_refuses(h):
    h.setup_lora()
    h.cmd(0x011D)                                            # FS
    h.cmd(0x0110, 1)                                         # SetRegMode hors STBY_RC
    assert h.status()['cmd'] == CMD_FAIL


# ------------------------------------------------------------------ sommeil
def test_sommeil_chaud_reveil_par_nop(h):
    h.setup_lora()
    h.cmd(0x011B, 1, 0, 0, 0, 0)
    h.tw.advance(20_000)
    assert h.busy(), 'BUSY haut en sommeil'
    h.cmd(0x0000, wait=False)                                # NOP : réveil
    assert 600 <= h.wait_busy() <= 700, 'réveil ≈ 640 µs'
    s = h.status()
    assert s['mode'] == MODE_STBY_RC and s['reset'] == 5, s
    assert h.status()['reset'] == 0, 'GetStatus efface ResetStatus'
    assert h.read(0x0202, 1)[1] == 2, 'configuration conservée'


def test_transaction_de_reveil_perdue(h):
    h.setup_lora()
    h.cmd(0x011B, 1, 0, 0, 0, 0)
    h.tw.advance(20_000)
    h.cmd(0x011C, 1, wait=False)                             # SetStandby XOSC comme transaction de réveil
    h.wait_busy()
    assert h.status()['mode'] == MODE_STBY_RC, 'commande perdue'


def test_sommeil_froid_contexte_perdu(h):
    h.setup_lora()
    h.cmd(0x011B, 0, 0, 0, 0, 0)
    h.tw.advance(20_000)
    h.cmd(0x0000, wait=False)
    assert 13_000 <= h.wait_busy() <= 13_600, 'réveil ≈ 13,3 ms'
    s = h.status()
    assert s['reset'] == 5 and s['irq'] & IRQ_ERROR
    assert h.read(0x0202, 1)[1] == 0, 'type de paquet perdu'


# ------------------------------------------------------------------ RX / TX et mode de repli
@pytest.mark.parametrize('fallback', [1, 2, 3])
def test_expiration_rx_toujours_stby_rc(h, fallback):
    h.setup_lora()
    h.cmd(0x0213, fallback)
    h.cmd(0x0209, 0, 0x01, 0x48)                             # 10 ms
    h.tw.advance(40_000)
    s = h.status()
    assert s['mode'] == MODE_STBY_RC and s['irq'] & IRQ_TIMEOUT


def test_tx_done_mode_de_repli(h):
    h.setup_lora()
    h.cmd(0x0213, 3)                                         # repli FS
    h.cmd(0x0109, *b'bonjour')
    h.cmd(0x020A, 0, 0, 0)
    h.tw.advance(h.r.toa_us(7) + 20_000)
    s = h.status()
    assert s['irq'] & IRQ_TX_DONE and s['mode'] == MODE_FS


def test_rx_simple_mode_de_repli(h):
    h.setup_lora()
    h.cmd(0x0213, 2)
    h.cmd(0x0209, 0, 0, 0)                                   # RX simple
    h.wait_busy()
    receive(h, frame(h))
    s = h.status()
    assert s['irq'] & IRQ_RX_DONE and s['mode'] == MODE_STBY_XOSC


def test_tampons_tx_et_rx_distincts(h):
    h.setup_lora()
    h.cmd(0x0109, *range(16))
    assert h.read(0x010A, 16, 0, 16)[1:17] != bytes(range(16))


# ------------------------------------------------------------------ écoute cyclique
def duty(h, rx_ms=36.8, sl_ms=59.5):
    h.setup_lora()
    h.cmd(0x0113, 0, 0, 0x04, 0xC8, 0, 0, 0, 0)
    r, s = round(rx_ms * 32.768), round(sl_ms * 32.768)
    h.cmd(0x0214, *r.to_bytes(3, 'big'), *s.to_bytes(3, 'big'), 0)
    h.wait_busy()


def chronogram(h, ms, settle=True):
    if settle:
        h.wait_busy(70_000)                                  # BUSY de la dernière commande
    edges, lvl, t0 = [], h.busy(), h.tw.t
    last = t0
    while h.tw.t - t0 < ms * 1000 * US:
        h.tw.advance(50)
        if h.busy() != lvl:
            edges.append(('H' if lvl else 'L', (h.tw.t - last) / US / 1000))
            last, lvl = h.tw.t, h.busy()
    return edges


def test_cycle_bas_36_8_haut_64_7(h):
    duty(h)
    e = chronogram(h, 500, settle=False)[1:-1]
    lows = [d for k, d in e if k == 'L']
    highs = [d for k, d in e if k == 'H']
    assert lows and highs
    assert all(abs(d - 36.8) < 0.5 for d in lows), lows
    assert all(abs(d - 64.7) < 0.5 for d in highs), highs


def test_cycle_reset_status_rtc(h):
    duty(h)
    h.tw.advance(250_000)
    while h.busy():
        h.tw.advance(100)
    assert h.status()['reset'] == 6


def test_cycle_rx_done_stby_rc_et_fin_du_cycle(h):
    duty(h, 56.0)                                            # fenêtre assez longue pour attendre l'en-tête
    while h.busy():
        h.tw.advance(100)
    receive(h, frame(h))
    assert not h.busy()
    s = h.status()
    assert s['mode'] == MODE_STBY_RC and s['irq'] & IRQ_RX_DONE
    assert h.r.duty is None
    assert chronogram(h, 300) == [], 'le cycle ne reprend pas'
    assert h.read(0x0203, 2)[1] == 40, 'trame lisible'


def test_cycle_nss_pendant_le_sommeil(h):
    duty(h)
    while not h.busy():
        h.tw.advance(100)
    h.cmd(0x0000, wait=False)
    assert 600 <= h.wait_busy() <= 700
    s = h.status()
    assert s['mode'] == MODE_STBY_RC and s['reset'] == 5
    assert h.r.duty is None and chronogram(h, 300) == []


def test_cycle_setstandby_pendant_une_fenetre(h):
    duty(h)
    h.cmd(0x011C, 0)                                         # attend BUSY bas : dans une fenêtre
    h.wait_busy()
    assert h.r.duty is None and chronogram(h, 300) == []


def test_cycle_commande_attend_la_fenetre(h):
    """Un pilote qui attend BUSY bas patiente jusqu'à la fenêtre suivante (≤ 64,7 ms) : piste de la panne rx-duty."""
    duty(h)
    while not h.busy():
        h.tw.advance(100)
    waited = h.wait_busy()
    assert 1_000 < waited <= 64_800


# ------------------------------------------------------------------ délai d'en-tête en écoute cyclique
def catch_rate(rx_ms, sl_ms, n=60):
    """Trames de 32 symboles (SF8, 62,5 kHz) à des instants répartis sur le cycle : part reçue."""
    h = Host()
    h.wait_busy()
    duty(h, rx_ms, sl_ms)
    got = 0
    period_us = (rx_ms + sl_ms + 5.2) * 1000
    for i in range(n):
        h.tw.advance(period_us * (i % 7 + 0.37 * i) % period_us + 3_000)
        if h.r.duty is None:                                 # trame précédente reçue : relancer le cycle
            r, s = round(rx_ms * 32.768), round(sl_ms * 32.768)
            h.cmd(0x0114, 0xFF, 0xFF, 0xFF, 0xFF)
            h.cmd(0x0214, *r.to_bytes(3, 'big'), *s.to_bytes(3, 'big'), 0)
            h.wait_busy(70_000)
        n0 = len(h.r.rx_packets)
        f = frame(h)
        h.r.air_start(f)
        h.tw.advance(h.r.toa_us(len(f['data'])))
        h.r.air_end(f)
        got += len(h.r.rx_packets) > n0
    return got / n


def test_cycle_radiolib_perd_les_trames_de_32_symboles():
    """Mesuré sur le boîtier : 6 trames sur 20 avec le cycle de RadioLib (en-tête après le délai 2 × rx + sommeil)."""
    assert 0.2 <= catch_rate(36.8, 59.5) <= 0.4


def test_cycle_fenetre_elargie_recoit_tout():
    assert catch_rate(56.0, 59.5) == 1.0
