"""Arrêt véritable du compagnon MeshCore : veille ultra-profonde du GR5515, réveil par le bouton ou le câble.

- appui de 3 s (au-delà des 8 s de démarrage) : GNSS et son alimentation de secours, pont batterie (GPIO14) et
  buzzer coupés, LR1110 en sommeil, puis veille ultra-profonde réveillable par AON0 (bouton) et AON4 (charge) ;
- éteint : le boîtier se rallume si le bouton est encore tenu quand l'application démarre (sur la carte, le
  réveil par le bootloader prend 2 à 3 s : appui d'environ 3 s ; le jumeau démarre aussitôt, d'où 300 ms ici) ;
  un appui plus court rendort aussitôt le boîtier, sans compter un démarrage raté ; le maintien de 6 s reste le DFU ;
- éteint, début de charge (AON4 à 0) : la LED montre la charge sans démarrer MeshCore, le débranchement rendort ;
  le comparateur MSIO ne réveillant pas de ce mode, un boîtier déjà chargé ne se réveille pas au branchement.
"""
import re
import pytest
from conftest import MAGIC, preset, GUARD
from helpers import Console, CABLE, CHG_STAT, LED_PWM, plug, unplug  # noqa: F401
from twin import T1000A

OFF_REC = GUARD + 0x800


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
    assert n.wake_ext == {0: 1, 4: 0}, 'réveil par le bouton (AON0 haut) et par le début de charge (AON4 bas)'
    assert n.tw.flash_bytes(OFF_REC, 4) == b'OFF1'
    assert not n.ble.adv_on
    for i in range(5):
        n.press(100)
        n.run(3000)
        assert n.is_off(), f'appui court n° {i + 1} : le boîtier doit se rendormir'
    assert resets(n).count('réveil après arrêt véritable') == 5
    assert n.tw.state != 'dfu' and guard_count(n) == 0, 'les réveils ne comptent pas comme des démarrages ratés'
    assert n.off_pins['vrtc_en'] == 0 and n.off_pins['bat_en'] == 0
    assert n.tw.flash_bytes(OFF_REC, 6)[4:] == (3900).to_bytes(2, 'little'), 'tension du premier arrêt conservée'


def test_appui_long_rallume(meshcore_debug_elf):
    n = power_off(meshcore_debug_elf)
    for _ in range(2):
        n.press(100)
        n.run(2000)
    n.press(2600)
    n.run(8000)
    assert not n.is_off() and n.tw.state == 'run'
    assert n.tw.flash_bytes(OFF_REC, 4) == b'\x00' * 4, 'état « éteint » effacé'
    assert n.ble.adv_on, 'MeshCore a démarré : annonce BLE'
    con = Console(n).open()
    r = con.send('power')
    assert re.search(r'last off: bat 3900 -> \d+ mV, 2 wake\(s\) back to sleep', r), r


def test_maintien_6s_dfu_depuis_eteint(meshcore_elf):
    n = power_off(meshcore_elf)
    n.press(7000)
    n.run(9000)
    assert n.tw.state == 'dfu'


def test_charge_a_l_arret(meshcore_elf):
    n = power_off(meshcore_elf)
    plug(n, charging=False)           # déjà chargé : AON4 reste à 1, pas de réveil
    n.run(1500)
    assert n.is_off()
    unplug(n)
    plug(n, charging=True)            # début de charge : AON4 à 0 réveille le boîtier
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
