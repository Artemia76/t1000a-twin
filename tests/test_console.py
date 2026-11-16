"""Console BLE de secours (BleConsole, service GUS de Goodix) : seul accès au T1000-A hors application.

Commandes vérifiées : info, dfu (retour au bootloader Seeed), reboot, commande inconnue, deux lignes
reçues d'affilée (aucune perdue).
"""
import pytest
from conftest import GUARD, DFU_FLAG, MAGIC, preset, require_console
from twin import T1000A
from helpers import Console, GUS_TX, GUS_RX  # noqa: F401  (réexportés)


@pytest.fixture
def console(any_elf):
    require_console(any_elf)
    n = T1000A(any_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.tw.trace('app_aon_wdt_deinit')
    return Console(n).open()


def test_info_repond(console):
    r = console.send('info')
    assert r.strip(), 'la console doit répondre à « info »'
    assert console.node.tw.resets == []


def test_dfu_renvoie_au_bootloader(console):
    console.send('dfu', wait_ms=3000)
    n = console.node
    assert n.tw.state == 'dfu'
    assert n.tw.flash_bytes(DFU_FLAG, 16) == b'\x01' + b'\xff' * 15
    assert 'app_aon_wdt_deinit' in n.tw.calls, 'chien de garde coupé avant le DFU'


def test_inconnue_puis_reboot(console):
    console.send('xyz')
    console.send('reboot', wait_ms=2000)
    n = console.node
    assert n.tw.resets, 'reboot doit redémarrer'
    assert n.tw.state == 'run'
    assert n.tw.flash_bytes(DFU_FLAG, 1) == b'\xff', 'pas de DFU'


def test_deux_lignes_d_affilee(console):
    n = console.node
    ph = console.ph
    ph.write(GUS_RX, b'xyz\r\n')
    ph.write(GUS_RX, b'reboot\r\n')          # 2e ligne avant que la boucle n'ait traité la 1re
    n.run(2000)
    assert n.tw.resets, 'la 2e commande ne doit pas être perdue'
