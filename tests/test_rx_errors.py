"""Paquets reçus rejetés (CRC) : diagnostic de la console « rxdiag » (firmware debug).

Sur le boîtier, le 19/11/2026, en zone dense : 227 paquets rejetés (-7, CRC_MISMATCH) pour 205 bons. RadioLib
renvoie -7 pour un CRC de données faux comme pour une erreur d'en-tête sans en-tête valide ; le firmware debug
classe les rejets d'après les drapeaux d'IRQ et compare SNR / RSSI des paquets bons et rejetés.
"""
import re
import random
import pytest
from conftest import MAGIC, preset
from helpers import Console
from twin import T1000A, time_on_air_us
from twin.lr1110 import BW_KHZ


@pytest.fixture(scope='module')
def node(meshcore_debug_elf):
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(15000)
    n.con = Console(n).open()
    if 'rx rejected' not in n.con.send('rxdiag'):
        pytest.skip('firmware sans diagnostic des paquets rejetés')
    return n


def send_frame(n, corrupt, rssi, snr, rng):
    r = n.radio
    data = bytes([0x11, 0x00]) + bytes(rng.randrange(256) for _ in range(30))
    frame = dict(r.params(), data=data, rssi=rssi, snr=snr, preamble=32, corrupt=corrupt)
    toa_ms = time_on_air_us(len(data), r.sf, BW_KHZ[r.bw], r.cr, 32) / 1000
    r.air_start(frame)
    n.run(int(toa_ms) + 1)
    r.air_end(frame)
    n.run(rng.randint(300, 600))


def test_rejets_classes_crc_avec_snr_rssi(node):
    rng = random.Random(3)
    node.con.send('rxdiag reset')
    for _ in range(3):
        send_frame(node, False, -90.0, 7.0, rng)          # bons : SNR 5..10, RSSI -95..-85
    for _ in range(2):
        send_frame(node, True, -110.0, -7.0, rng)         # rejetés : SNR -10..-5, RSSI -115..-105
    out = node.con.send('rxdiag', 800)
    m = re.search(r'rx rejected: crc (\d+), header (\d+), other (\d+)', out)
    assert m, out
    assert (int(m.group(1)), int(m.group(2)), int(m.group(3))) == (2, 0, 0), out
    assert ' 5:3/0 ' in out and ' -10:0/2 ' in out, out           # histogramme SNR (bons / rejetés)
    assert ' -95:3/0 ' in out and ' -115:0/2 ' in out, out        # histogramme RSSI
    rej = re.findall(r'\[\d+s ([0-9a-f]+) (-?\d+) (-?\d+\.\d+) (\d+) ([0-9a-f]{2}) ([0-9a-f]{2})\]', out)
    assert len(rej) == 2, out
    for irq, rssi, snr, ln, b0, b1 in rej:
        assert int(irq, 16) & (1 << 7), irq                        # IRQ_CRC_ERR
        assert int(rssi) == -110 and float(snr) == -7.0 and int(ln) == 32 and (b0, b1) == ('11', '00')
