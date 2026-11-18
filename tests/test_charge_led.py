"""LED d'état du compagnon MeshCore sur le chargeur (AON3, PWM1_A).

Câble de charge : MSIO0 ≈ 2,5 V branché, ≈ 35 mV débranché. Chargeur : AON4 bas pendant la charge.
- en charge : respiration sinusoïdale (rapport cyclique PWM variable, jamais nul hors extinction) ;
- chargé : allumée fixe (100 %) ;
- activité MeshCore en négatif : la LED s'éteint 20 ms par cycle de 4 s (200 ms si des messages ne sont pas lus) ;
- sur batterie : clignotement de vie en tout ou rien, sans PWM ;
- sur le chargeur, le CPU ne passe plus en veille.
"""
import re
import pytest
from conftest import MAGIC, preset
from helpers import Console, CABLE, CHG_STAT, LED_PWM, plug, unplug  # noqa: F401
from twin import T1000A

BUZZER_PWM = 0


def sample(node, ms, step=10):
    """[(rapport cyclique PWM de la LED ou None, niveau GPIO de la LED)] toutes les `step` ms."""
    out = []
    for _ in range(ms // step):
        node.run(step)
        out.append((node.pwm.duty.get(LED_PWM), node.led()))
    return out


def dark_runs(duties):
    """Longueurs (en échantillons) des passages à 0 consécutifs."""
    runs, n = [], 0
    for d in duties:
        if d == 0:
            n += 1
        elif n:
            runs.append(n)
            n = 0
    return runs


@pytest.fixture(scope='module')
def board(meshcore_debug_elf):
    """Un seul boîtier pour le module (le démarrage coûte ≈ 20 s), avec la console BLE ouverte."""
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    unplug(n)
    n.run(4000)
    n.con = Console(n).open()
    return n


@pytest.fixture
def node(board):
    unplug(board)               # chaque test part du boîtier débranché, clignotement de vie revenu
    board.run(1500)
    return board


def test_sur_batterie_clignotement_sans_pwm(node):
    s = sample(node, 8000)
    assert all(d is None for d, _ in s), 'pas de PWM sur batterie'
    on = sum(1 for _, led in s if led)
    # le flash dure 20 ms : le CPU est réveillé pour l'éteindre (STATUS_LED_WAKE), sinon il durerait jusqu'au
    # réveil suivant (au plus 1 s)
    assert 0 < on <= len(s) * 0.02, f'clignotement de vie attendu ({on}/{len(s)} échantillons allumés)'


def test_respiration_en_charge(node):
    plug(node, charging=True)
    node.run(1500)
    duties = [d for d, _ in sample(node, 4200)]
    assert all(d is not None for d in duties), 'PWM actif en permanence sur le chargeur'
    lit = [d for d in duties if d > 0]
    assert max(lit) == 100 and min(lit) <= 2, f'respiration du minimum au maximum ({min(lit)}..{max(lit)} %)'
    assert len(set(lit)) > 20, 'variation continue de la luminosité'
    runs = dark_runs(duties)
    assert 1 <= len(runs) <= 2 and all(r <= 4 for r in runs), f'une extinction de ≈ 20 ms par cycle de 4 s : {runs}'
    for i, d in enumerate(duties[1:], 1):
        if d == 0 and duties[i - 1] != 0:
            assert duties[i - 1] >= 90, 'extinction au sommet de la courbe'


def test_charge_terminee_allumee_fixe(node):
    plug(node, charging=False)
    node.run(1500)
    duties = [d for d, _ in sample(node, 4200)]
    assert set(duties) <= {0, 100}, set(duties)
    runs = dark_runs(duties)
    assert 1 <= len(runs) <= 2 and all(r <= 4 for r in runs), runs


def test_debranchement_retour_au_clignotement(node):
    plug(node)
    # câble relu au plus une fois par seconde, et sur batterie le CPU dort jusqu'à 1 s entre deux tours :
    # jusqu'à ≈ 2 s avant la respiration selon la phase (mesuré : 10 à 1280 ms)
    for _ in range(250):
        node.run(10)
        if node.pwm.duty.get(LED_PWM) is not None:
            break
    assert node.pwm.duty.get(LED_PWM) is not None, 'respiration de la LED sur le chargeur'
    unplug(node)
    node.run(2500)
    s = sample(node, 8000)
    assert all(d is None for d, _ in s), 'PWM de la LED arrêté'
    assert any(led for _, led in s), 'clignotement de vie revenu'


def test_pas_de_veille_sur_le_chargeur(node):
    con = node.con
    plug(node)
    node.run(1500)
    con.send('power reset')
    node.run(5000)
    idle = float(re.search(r'idle (\d+\.\d) %', con.send('power')).group(1))
    assert idle < 20, f'CPU éveillé sur le chargeur (idle {idle} %)'
    unplug(node)
    node.run(1500)
    con.send('power reset')
    node.run(10000)
    idle = float(re.search(r'idle (\d+\.\d) %', con.send('power')).group(1))
    assert idle > 80, f'veille de retour après débranchement (idle {idle} %)'


def test_buzzer_pendant_la_respiration(node):
    con = node.con
    plug(node)
    node.run(1500)
    assert 'beep 2000 Hz' in con.send('beep', wait_ms=100)    # son de 500 ms : vérifié pendant qu'il sonne
    assert node.pwm.running.get(BUZZER_PWM) == 2000, 'le buzzer (PWM0) sonne'
    assert node.pwm.duty.get(LED_PWM) is not None, 'la LED (PWM1) continue de respirer'


def test_message_non_lu_extinction_longue(meshcore_debug_elf):
    from twin import World
    from twin.companion import Companion
    import struct
    w = World()
    a = w.add(T1000A(meshcore_debug_elf, name='A', flash=preset(guard=MAGIC + b'\x00')))
    b = w.add(T1000A(meshcore_debug_elf, name='B'))
    unplug(a)
    w.run(4000)
    cb = Companion(b).start()
    # message sur le canal public : A n'a pas de téléphone connecté, il le garde « non lu »
    r = cb.query(bytes([3, 0, 0]) + struct.pack('<I', 1767225600) + b'coucou', expect=0)
    assert any(d['code'] == 0 for d in r), r
    w.run(5000)
    plug(a)
    w.run(1500)
    duties = []
    for _ in range(420):
        w.run(10)
        duties.append(a.pwm.duty.get(LED_PWM))
    runs = dark_runs(duties)
    assert 1 <= len(runs) <= 2 and all(15 <= r <= 25 for r in runs), f'extinction de ≈ 200 ms (message non lu) : {runs}'
