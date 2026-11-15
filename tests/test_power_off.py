"""Arrêt véritable du compagnon MeshCore : veille ultra-profonde du GR5515, réveil par le bouton ou le câble.

- appui de 3 s (au-delà des 8 s de démarrage) : GNSS et son alimentation de secours, pont batterie (GPIO14) et
  buzzer coupés, LR1110 en sommeil, puis veille ultra-profonde réveillable par AON0 et par le comparateur MSIO0 ;
- éteint : un appui court rendort aussitôt le boîtier (sans compter un démarrage raté), un appui de 2 s le
  rallume ; le maintien de 6 s reste le DFU ;
- éteint, câble branché : la LED montre la charge sans démarrer MeshCore, le débranchement rendort.
"""
import re
import pytest
from conftest import MAGIC, preset, GUARD
from test_console import Console
from twin import T1000A

CABLE, CHG_STAT, LED_PWM = 0, 0x24, 1
OFF_REC = GUARD + 0x800


def unplug(n):
    n.adc.sources[CABLE] = lambda: 35
    n.gpio.drive(CHG_STAT, 1)


def plug(n, charging=True):
    n.adc.sources[CABLE] = lambda: 2490
    n.gpio.drive(CHG_STAT, 0 if charging else 1)


def power_off(meshcore_elf):
    """Boîtier démarré puis éteint par un appui de 3,5 s."""
    n = T1000A(meshcore_elf, flash=preset(guard=MAGIC + b'\x00'))
    unplug(n)
    n.run(10000)                      # les 8 premières secondes, un appui long ouvre le mode secours
    n.press(3500)
    n.run(5000)
    assert n.is_off(), 'le boîtier doit être éteint après un appui de 3 s'
    return n


def resets(n):
    return [r for _, r in n.tw.resets]


def guard_count(n):
    return n.tw.flash_bytes(GUARD + 4, 16).count(0)


def test_arret_puis_appuis_courts(meshcore_elf):
    n = power_off(meshcore_elf)
    assert n.off_pins == {'gps_en': 0, 'vrtc_en': 0, 'bat_en': 0, 'buzzer_en': 0, 'led': 0, 'lora_sleep': True}
    assert n.wake_ext and n.wake_ext[0] & 1, 'réveil par le bouton (AON0)'
    assert n.wake_comp == (0, 990), 'réveil par le comparateur sur MSIO0, seuil ≈ 1 V'
    assert n.tw.flash_bytes(OFF_REC, 4) == b'OFF1'
    assert not n.ble.adv_on
    for i in range(5):
        n.press(300)
        n.run(3000)
        assert n.is_off(), f'appui court n° {i + 1} : le boîtier doit se rendormir'
    assert resets(n).count('réveil après arrêt véritable') == 5
    assert n.tw.state != 'dfu' and guard_count(n) == 0, 'les réveils ne comptent pas comme des démarrages ratés'
    assert n.off_pins['vrtc_en'] == 0 and n.off_pins['bat_en'] == 0
    assert n.tw.flash_bytes(OFF_REC, 6)[4:] == (3900).to_bytes(2, 'little'), 'tension du premier arrêt conservée'


def test_appui_long_rallume(meshcore_elf):
    n = power_off(meshcore_elf)
    n.press(2600)
    n.run(8000)
    assert not n.is_off() and n.tw.state == 'run'
    assert n.tw.flash_bytes(OFF_REC, 4) == b'\x00' * 4, 'état « éteint » effacé'
    assert n.ble.adv_on, 'MeshCore a démarré : annonce BLE'
    con = Console(n).open()
    r = con.send('power')
    assert re.search(r'last off: \d+h\d\dm .*bat 3900 -> \d+ mV', r), r


def test_maintien_6s_dfu_depuis_eteint(meshcore_elf):
    n = power_off(meshcore_elf)
    n.press(7000)
    n.run(9000)
    assert n.tw.state == 'dfu'


def test_charge_a_l_arret(meshcore_elf):
    n = power_off(meshcore_elf)
    plug(n, charging=True)            # le comparateur réveille le boîtier
    n.run(1500)
    assert not n.is_off()
    duties = []
    for _ in range(200):
        n.run(20)
        duties.append(n.pwm.duty.get(LED_PWM))
    assert all(d is not None for d in duties) and len(set(duties)) > 10, 'la LED respire'
    assert 0 not in duties, 'pas d\'extinction d\'activité : MeshCore ne tourne pas'
    assert not n.ble.adv_on, 'MeshCore n\'a pas démarré'
    n.gpio.drive(CHG_STAT, 1)          # chargé
    n.run(500)
    assert n.pwm.duty.get(LED_PWM) == 100
    unplug(n)
    n.run(2500)
    assert n.is_off(), 'câble débranché : retour à l\'arrêt'
    plug(n)
    n.run(1500)
    n.press(2600)
    n.run(8000)
    assert not n.is_off() and n.ble.adv_on, 'appui de 2 s pendant la charge : MeshCore démarre'
