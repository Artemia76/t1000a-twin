"""Accéléromètre QMA6100P : commande console « acc » (firmware debug) et absence de la puce."""
import re
import pytest
from conftest import MAGIC, preset
from helpers import Console
from twin import T1000A

ACC = re.compile(r'acc: (ok|FAILED) chip id 0x([0-9a-f]{2})  x (-?\d+) y (-?\d+) z (-?\d+)  '
                 r'\((-?\d+\.\d+) (-?\d+\.\d+) (-?\d+\.\d+) g, \|a\| (\d+\.\d+) g\)')


@pytest.fixture
def console(meshcore_debug_elf):
    return Console(T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))).open()


def test_acc_a_plat(console):
    console.node.accel.accel_g = (0.0, 0.0, 1.0)
    m = ACC.search(console.send('acc'))
    assert m and m.group(1) == 'ok' and m.group(2) == '90', m
    assert (int(m.group(3)), int(m.group(4)), int(m.group(5))) == (0, 0, 1024)
    assert float(m.group(9)) == 1.0


def test_acc_incline_et_coupe_ensuite(console):
    n = console.node
    n.accel.accel_g = (-0.7071, 0.0, 0.7071)
    m = ACC.search(console.send('acc'))
    assert m and m.group(1) == 'ok', m
    assert abs(float(m.group(6)) + 0.707) <= 0.002 and abs(float(m.group(8)) - 0.707) <= 0.002
    assert abs(float(m.group(9)) - 1.0) <= 0.002
    assert (n.gpio.level(7), n.gpio.level(8), n.gpio.level(9)) == (0, 0, 0)
    assert n.accel.regs[0x11] == 0, "registres perdus à la coupure de l'alimentation"


def test_acc_puce_absente(console):
    console.node.accel.chip_id = 0x00             # autre puce ou pas de réponse attendue : pas de lecture
    m = ACC.search(console.send('acc'))
    assert m and m.group(1) == 'FAILED' and m.group(2) == '00', m
    assert console.node.gpio.level(7) == 0
