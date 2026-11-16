"""Firmware release du compagnon MeshCore (t1000a_companion_radio_ble) : mêmes fonctions que la debug, sans console.

La console BLE (service GUS) n'existe que dans le firmware debug : elle n'est pas protégée par le code PIN du
compagnon. La release se met à jour par le maintien du bouton 6 s (DFU).
"""
import pytest
from conftest import MAGIC, preset, has_console
from test_console import GUS_TX
from twin import T1000A
from twin.companion import Companion

CABLE, CHG_STAT, LED_PWM = 0, 0x24, 1


@pytest.fixture
def node(meshcore_release_elf):
    n = T1000A(meshcore_release_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.adc.sources[CABLE] = lambda: 35
    n.gpio.drive(CHG_STAT, 1)
    n.run(10000)
    return n


def test_sans_console(meshcore_release_elf):
    assert not has_console(meshcore_release_elf), 'la release ne doit pas embarquer la console BLE'


def test_demarre_compagnon_et_pas_de_service_gus(node):
    assert node.tw.resets == []
    comp = Companion(node).start()
    names = [d['name'] for d in comp.device_query()]
    assert 'DEVICE_INFO' in names, names
    assert node.ble.find(GUS_TX) is None, 'pas de service GUS : pas de console'


def test_clignotement_de_vie(node):
    led = []
    for _ in range(80):
        node.run(100)
        led.append(node.led())
    assert 0 < sum(led) < len(led), 'clignotement de vie'


def test_led_de_charge(node):
    node.adc.sources[CABLE] = lambda: 2490
    node.gpio.drive(CHG_STAT, 0)
    node.run(2000)
    duties = []
    for _ in range(50):
        node.run(20)
        duties.append(node.pwm.duty.get(LED_PWM))
    assert all(d is not None for d in duties) and len(set(duties)) > 5, 'respiration de charge'


def test_arret_et_rallumage(node):
    node.press(3500)
    node.run(5000)
    assert node.is_off()
    node.press(2600)
    node.run(8000)
    assert not node.is_off() and node.ble.adv_on


def test_maintien_6s_dfu(node):
    node.press(7000)
    node.run(9000)
    assert node.tw.state == 'dfu', 'seule entrée en DFU de la release'
