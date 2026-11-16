"""Réception LoRa en cycle veille/écoute (LR1110 SetRxDutyCycle) sur le compagnon MeshCore du T1000-A.

À SF8 / 62,5 kHz, avec le préambule de 32 symboles de MeshCore ≥ 1.16, la radio écoute ≈ 37 ms puis dort ≈ 60 ms.
- toute trame à préambule de 32 symboles est captée, quelle que soit sa phase ; une partie des trames à préambule
  de 16 symboles (MeshCore ≤ 1.15) est perdue : limite acceptée, documentée ici ;
- aucun accès SPI pendant le sommeil de la radio (il la réveillerait et casserait le cycle) : le bruit de fond est
  mesuré par de courtes salves en écoute continue, au démarrage puis toutes les 10 min ;
- console « rxduty [on|off] », réglage enregistré en Flash.
"""
import random
import re
import struct
import pytest
from conftest import MAGIC, preset
from test_console import Console
from twin import T1000A, World, time_on_air_us
from twin.companion import Companion, decode
from twin.lr1110 import BW_KHZ

NOW = 1794571200


@pytest.fixture(scope='module')
def node(meshcore_debug_elf):
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(15000)                       # démarrage, puis première mesure du bruit de fond en écoute continue
    n.con = Console(n).open()
    return n


def inject(n, preamble, count=40, seed=1):
    """Trames synthétiques (préambule donné) à des phases variées. Renvoie (captées, manquées)."""
    r = n.radio
    rng = random.Random(seed)
    got0, miss0 = len(r.rx_packets), r.rx_missed
    for i in range(count):
        n.run(rng.randint(150, 650))
        frame = dict(r.params(), data=bytes(rng.randrange(256) for _ in range(24)), rssi=-90.0, snr=5.0)
        frame['preamble'] = preamble
        toa_ms = time_on_air_us(len(frame['data']), r.sf, BW_KHZ[r.bw], r.cr, preamble) / 1000
        r.air_start(frame)
        n.run(int(toa_ms) + 1)
        r.air_end(frame)
    n.run(500)
    return len(r.rx_packets) - got0, r.rx_missed - miss0


def test_cycle_actif_apres_calibration(node):
    r = node.radio
    assert r.duty is not None, 'la réception doit être en cycle veille/écoute'
    assert 0.30 <= r.duty_share <= 0.45, f'part d\'écoute {r.duty_share:.1%}'
    out = node.con.send('rxduty')
    m = re.search(r'rxduty: on, cycling, listen (\d+) ms / sleep (\d+) ms \(([\d.]+) %\), noise floor (-\d+) dBm', out)
    assert m, out
    assert 30 <= int(m.group(1)) <= 45 and 50 <= int(m.group(2)) <= 70
    assert int(m.group(4)) < -60, 'bruit de fond mesuré (écoute continue au démarrage)'
    levels = set()
    for _ in range(40):
        node.run(10)
        levels.add(node.gpio.level(r.busy_pin))
    assert levels == {0, 1}, 'BUSY alterne : fenêtres d\'écoute et sommeil'


def test_preambule_32_toutes_captees(node):
    got, missed = inject(node, 32)
    assert (got, missed) == (40, 0), f'{got} captées, {missed} manquées'
    assert node.radio.duty is not None, 'retour au cycle après chaque réception'


def test_preambule_16_partiellement_perdues(node):
    got, missed = inject(node, 16, seed=2)
    assert missed > 0 and got > 0, f'{got} captées, {missed} manquées (MeshCore ≤ 1.15 : limite acceptée)'


def test_repos_10_min_sans_spi_pendant_le_sommeil(node):
    r = node.radio
    v0, runs0 = len(r.violations), r.duty_runs
    node.con.send('power reset')
    node.run(610000)
    assert r.violations[v0:] == [], r.violations[v0:v0 + 3]
    assert r.duty_runs > runs0, 'le bruit de fond a été remesuré (salve en écoute continue), puis retour au cycle'
    assert r.duty is not None
    rows = [l for l in node.con.send('power log', wait_ms=1000).splitlines() if l and not l.startswith('#')]
    assert rows and int(rows[0].split(',')[11], 16) & 0x80, 'drapeau F_RX_DUTY dans le journal'
    assert re.search(r'rx duty 3\d\.\d %', node.con.send('power'))


def test_off_on_et_reglage_conserve(node):
    r = node.radio
    assert 'rxduty: off, continuous receive' in node.con.send('rxduty off')
    node.run(500)
    assert r.duty is None
    node.con.send('reboot')
    node.run(15000)
    node.con = Console(node).open()
    assert r.duty is None, 'réglage « off » conservé au redémarrage'
    assert 'rxduty: off' in node.con.send('rxduty')
    node.con.send('rxduty on')
    node.run(10000)
    assert r.duty is not None, 'retour au cycle après la mesure du bruit de fond'
    assert 'rxduty: on, cycling' in node.con.send('rxduty')


def test_deux_noeuds_advert_message_et_accuse(meshcore_elf):
    w = World()
    a = w.add(T1000A(meshcore_elf, name='A'))
    b = w.add(T1000A(meshcore_elf, name='B'))
    w.run(15000)
    assert a.radio.duty is not None and b.radio.duty is not None
    ca, cb = Companion(a).start(), Companion(b).start()
    pk = {}
    for name, c in (('A', ca), ('B', cb)):
        info = [d for d in c.app_start() if d['code'] == 5][0]
        pk[name] = info['pubkey']
        c.set_time(NOW)
    cb.send_advert()
    w.run(3000)
    ca.send_advert()
    w.run(3000)
    assert [c['pubkey'] for c in ca.contacts()] == [pk['B']]
    assert [c['pubkey'] for c in cb.contacts()] == [pk['A']]
    for src, dst, c_src, c_dst, text in (('A', 'B', ca, cb, 'aller'), ('B', 'A', cb, ca, 'retour')):
        n = len(c_src.inbox)
        assert any(d['name'] == 'SENT' for d in c_src.send_text(pk[dst], text, ts=NOW + 10))
        w.run(8000)
        assert 'PUSH_SEND_CONFIRMED' in [decode(f)['name'] for f in c_src.inbox[n:]], f'accusé {src} → {dst}'
        assert [m.get('text') for m in c_dst.sync_messages()] == [text]
    for x in (a, b):
        assert x.radio.violations == [] and x.radio.duty is not None
