"""Configuration pytest du jumeau numérique T1000-A.

Firmwares testés (chemins par défaut, ou options) :
  --meshcore  ELF du compagnon MeshCore BLE (third_party/meshcore/.pio/build/t1000a_companion_radio_ble/firmware.elf)
  --blink     ELF de l'exemple platform-gr551x (BleConsole)
Un test dont le firmware est absent est sauté (pas d'échec).
"""
import os, sys
import pytest

HERE = os.path.dirname(__file__)
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))

DEFAULTS = {
    'meshcore': f'{ROOT}/third_party/meshcore/.pio/build/t1000a_companion_radio_ble/firmware.elf',
    'blink': f'{ROOT}/platform-gr551x/examples/blink/.pio/build/t1000-a/firmware.elf',
}

# Zones de Flash que nos firmwares ont le droit d'écrire (variante t1000-a de platform-gr551x)
GUARD, DFU_FLAG, DIAG, NVDS = 0x010E3000, 0x010E4000, 0x010E2000, 0x010FC000
FS = (0x010C2000, 0x20000)
ALLOWED_WRITES = [(GUARD, 0x1000), (DFU_FLAG, 0x1000), (NVDS, 0x4000), (DIAG, 0x1000), FS]
MAGIC = b'T1GD'


def pytest_addoption(parser):
    for k, v in DEFAULTS.items():
        parser.addoption(f'--{k}', default=v, help=f'ELF {k} (défaut : {v})')


def elf_path(config, kind):
    p = config.getoption(kind)
    if not os.path.exists(p):
        pytest.skip(f'firmware {kind} absent : {p}')
    return p


@pytest.fixture(scope='session')
def meshcore_elf(pytestconfig):
    return elf_path(pytestconfig, 'meshcore')


@pytest.fixture(scope='session')
def blink_elf(pytestconfig):
    return elf_path(pytestconfig, 'blink')


@pytest.fixture(scope='session', params=['meshcore', 'blink'])
def any_elf(request, pytestconfig):
    return elf_path(pytestconfig, request.param)


def flash_writes_outside(node):
    tw = node.tw
    bad = []
    for _, kind, d in tw.log_list:
        if kind in ('flash_write', 'flash_erase'):
            a = d['addr']
            if not any(s <= a < s + n for s, n in ALLOWED_WRITES):
                bad.append((kind, hex(a)))
    return bad


def preset(**zones):
    """Image Flash vierge (0xFF) avec des zones préremplies : preset(guard=b'T1GD\\0')."""
    img = bytearray(b'\xff' * 0x100000)
    names = {'guard': GUARD, 'dfu_flag': DFU_FLAG}
    for k, data in zones.items():
        a = names[k] - 0x01000000
        img[a:a + len(data)] = data
    return bytes(img)
