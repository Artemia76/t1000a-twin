"""LR1110 restée endormie en écoute cyclique (BUSY haut, aucune IRQ) : blocage observé sur le T1000-A le 18/11/2026.

Sur le boîtier, après ≈ 30 min d'émissions (Mesh Mapper), la radio ne se réveillait plus pour ses fenêtres d'écoute.
Toute commande SPI attendait alors BUSY 1 s puis échouait. Dispatcher::checkSend() estimait la durée d'émission par
RadioLib (getTimeOnAir → getPacketType, en SPI), qui renvoyait le code d'erreur -20 lu comme ≈ 71 min : la file d'envoi
était repoussée de plusieurs heures, chaque tour de boucle perdait 1 s, l'appli ramait et le CPU ne dormait plus.

Attendu du firmware corrigé :
- durée d'émission calculée sans SPI : la boucle ne bloque plus ;
- radio endormie trop longtemps détectée, réveillée et remise en réception (console : « stuck asleep ») ;
- le paquet demandé par l'appli part en quelques secondes.
"""
import re
import pytest
from conftest import MAGIC, preset
from helpers import Console, GUS_TX
from twin import T1000A
from twin.companion import Companion
from twin.lr1110 import MODE_SLEEP


@pytest.fixture(scope='module')
def node(meshcore_debug_elf):
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(15000)                        # démarrage, première mesure du bruit de fond, puis écoute cyclique
    n.comp = Companion(n).start()
    n.con = Console(n)
    n.phone.subscribe(GUS_TX)
    n.run(500)
    return n


def freeze_asleep(r):
    """La radio quitte son cycle et reste endormie : BUSY haut, plus aucune fenêtre d'écoute, aucune IRQ."""
    r._duty_stop()
    r.mode = MODE_SLEEP
    r._set_busy(1)


def test_radio_figee_reveillee_et_paquet_emis(node):
    r = node.radio
    assert r.duty is not None, 'écoute cyclique attendue avant le test'
    node.con.send('pending reset')
    freeze_asleep(r)
    node.run(3000)                      # le firmware doit s'en apercevoir sans attendre de paquet à émettre
    out = node.con.send('rxduty')
    m = re.search(r'stuck asleep (\d+)', out)
    if not m:
        pytest.skip('firmware sans chien de garde de la radio endormie')
    assert int(m.group(1)) >= 1, out
    assert r.duty is not None and not r.busy or r.mode != MODE_SLEEP, 'radio remise en réception'

    # radio de nouveau figée, puis l'appli demande une annonce : elle doit partir vite
    freeze_asleep(r)
    n0 = len(r.tx_packets)
    assert [d['name'] for d in node.comp.send_advert()] == ['OK']
    assert node.comp.wait(lambda: len(r.tx_packets) > n0, 8000), 'annonce jamais émise : file d\'envoi bloquée'
    pend = node.con.send('pending')
    m = re.search(r'checkSend (\d+) \(isReceiving', pend)
    assert m and int(m.group(1)) < 500, pend           # plus d'attente SPI de 1 s dans checkSend
    m = re.search(r'nextTxIn (-?\d+) ms', pend)
    assert m and int(m.group(1)) < 60000, pend         # aucune émission repoussée de plusieurs heures
