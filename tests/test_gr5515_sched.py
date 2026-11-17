"""Ordonnanceur du GR5515 : réveils de la veille légère par le contrôleur BLE, calés sur le boîtier.

pwr_mgmt_schedule() est en ROM. Mesures du 17/11/2026 sur le T1000-A (firmware de sonde, console « sched »,
.claude-work/captures/sched-2026-11-17.txt) :
- sans connexion BLE (annonce 152,5 ms) : ≈ 16 sorties de veille par seconde, veille moyenne ≈ 66 ms, jamais
  plus de 159 ms ; histogramme < 10 ms ≈ 6,6/s, 100 ms – 1 s ≈ 6,1/s (une longue veille par annonce, une courte
  à la fin de l'événement radio) ;
- CPU tenu éveillé (« sched awake ») : pwr_mgmt_schedule() en mode actif rend la main en ≈ 10 µs.
Le jumeau modélise ces réveils (twin/ble.py, _radio_event) ; ces tests utilisent le mode fidèle « all ».
"""
import re
import pytest
from conftest import MAGIC, preset
from helpers import Console, GUS_TX
from twin import T1000A


@pytest.fixture
def console(meshcore_debug_elf):
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.tw.radio_wakes = 'all'
    return Console(n).open()


def reconnect(c):
    c.ph.connect()
    c.node.run(200)
    c.ph.subscribe(GUS_TX)
    c.node.run(200)


def sched(c):
    r = c.send('sched')
    if 'idle' not in r:
        pytest.skip('firmware sans sonde « sched » (plateforme sans GR551X_SCHED_STATS)')
    out = {}
    for name in ('active', 'idle', 'loop'):
        m = re.search(name + r'\s+n=(\d+) ev=(\d+) avg=(\d+)us max=(\d+)ms \| <1ms (\d+) <10ms (\d+) <100ms (\d+) '
                      r'<1s (\d+) >=1s (\d+)', r)
        assert m, r
        out[name] = dict(zip(('n', 'ev', 'avg_us', 'max_ms', 'h1', 'h10', 'h100', 'h1s', 'hmax'),
                             map(int, m.groups())))
    return out


def test_annonce_reveille_le_cpu_comme_le_boitier(console):
    """Sans connexion : sorties de veille au rythme des annonces, comme mesuré (≈ 16/s, max ≈ 160 ms)."""
    node = console.node
    node.run(5000)
    console.send('sched reset')
    console.ph.disconnect()
    s0 = node.tw.sleeps
    node.run(30000)
    sleeps_per_s = (node.tw.sleeps - s0) / 30
    reconnect(console)
    st = sched(console)['idle']
    assert 12 <= sleeps_per_s <= 22, f'{sleeps_per_s:.1f} veilles/s (boîtier ≈ 16)'
    assert st['max_ms'] <= 170, st
    assert st['hmax'] == 0, st
    assert 45 <= st['avg_us'] / 1000 <= 90, st          # boîtier ≈ 66 ms
    assert st['h1s'] / 30 >= 4, st                      # une longue veille par annonce (boîtier ≈ 6/s)


def test_connexion_reveille_a_chaque_intervalle(console):
    """Connecté : au moins une sortie de veille par intervalle de connexion du téléphone."""
    node = console.node
    node.run(2000)
    s0 = node.tw.sleeps
    node.run(3000)
    per_s = (node.tw.sleeps - s0) / 3
    assert per_s >= 1000 / node.ble.conn['interval_ms'], f'{per_s:.0f} veilles/s'


def test_cpu_tenu_eveille_boucle_sans_attente(console):
    """« sched awake » : la boucle tourne sans dormir, pwr_mgmt_schedule() actif ne bloque pas."""
    console.send('sched awake 5')
    console.node.run(5000)
    st = sched(console)
    assert st['active']['n'] > 100, st
    assert st['active']['max_ms'] <= 1, st
    assert st['loop']['hmax'] == 0 and st['loop']['h1s'] == 0, st


def test_mode_turbo_saute_les_reveils_a_vide(meshcore_debug_elf):
    """Mode par défaut : les réveils dont le tour ne fait rien sont sautés (vitesse), sans perdre le rythme
    d'une seconde ni les vraies interruptions."""
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    assert n.tw.radio_wakes in ('turbo', 'all', 'off')
    n.tw.radio_wakes = 'turbo'
    n.run(20000)
    if n.tw.turbo:
        assert n.tw.radio_wake_skipped > n.tw.radio_wake_count, (n.tw.radio_wake_count, n.tw.radio_wake_skipped)
    assert n.tw.sleeps >= 15, 'au moins le battement d\'une seconde'
