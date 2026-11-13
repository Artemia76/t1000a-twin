"""Console BLE de secours (BleConsole, service GUS de Goodix) : seul accès au T1000-A hors application.

Commandes vérifiées : info, dfu (retour au bootloader Seeed), reboot, commande inconnue, deux lignes
reçues d'affilée (aucune perdue).
"""
import pytest
from conftest import GUARD, DFU_FLAG, MAGIC, preset
from twin import T1000A

_BASE = [0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3]
GUS_TX = bytes(_BASE + [0x02, 0x02, 0xED, 0xA6])        # notifications du T1000-A
GUS_RX = bytes(_BASE + [0x03, 0x02, 0xED, 0xA6])        # écritures du téléphone


class Console:
    def __init__(self, node):
        self.node, self.ph = node, node.phone
        self.out = bytearray()
        self.ph.on_notify.append(lambda u, d: self.out.extend(d) if u == GUS_TX else None)

    def open(self):
        n = self.node
        n.run(3000)
        assert n.ble.find(GUS_RX), 'service GUS absent de la base GATT'
        self.ph.connect()
        n.run(200)
        assert self.ph.subscribe(GUS_TX) in (None, 0)
        n.run(200)
        return self

    def send(self, line, wait_ms=500):
        k = len(self.out)
        err = self.ph.write(GUS_RX, line.encode() + b'\r\n')
        assert err in (None, 0), f'écriture console refusée ({err:#x})'
        self.node.run(wait_ms)
        return self.out[k:].decode('utf-8', 'replace')


@pytest.fixture
def console(any_elf):
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
