"""Configuration pytest du jumeau numérique T1000-A.

Firmwares testés (chemins par défaut, ou options) :
  --meshcore  ELF du compagnon MeshCore BLE, firmware debug (…/t1000a_companion_radio_ble_debug/firmware.elf)
  --meshcore-release  ELF release, sans console (…/t1000a_companion_radio_ble/firmware.elf)
  --blink     ELF de l'exemple platform-gr551x (BleConsole)
  --firmware  un seul ELF à valider (garde-fou avant flash) : les tests génériques s'appliquent à lui, les tests
              MeshCore seulement s'il s'agit d'un firmware MeshCore.
Un test dont le firmware est absent est sauté (pas d'échec). Les tests qui passent par la console BLE sont sautés
sur un firmware sans console (release).
"""
import os, sys
import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.abspath(os.path.join(HERE, '..')))
from twin.paths import ROOT  # dépôt T1000A (voir twin/paths.py)

DEFAULTS = {
    # debug : console BLE et journal d'énergie (couverture complète) ; release : sans console (test_release.py)
    'meshcore': f'{ROOT}/firmware/meshcore/.pio/build/t1000a_companion_radio_ble_debug/firmware.elf',
    'meshcore_release': f'{ROOT}/firmware/meshcore/.pio/build/t1000a_companion_radio_ble/firmware.elf',
    'blink': f'{ROOT}/platform/gr551x/examples/blink/.pio/build/t1000-a/firmware.elf',
}

# Zones de Flash que nos firmwares ont le droit d'écrire (variante t1000-a de platform-gr551x)
GUARD, DFU_FLAG, DIAG, NVDS = 0x010E3000, 0x010E4000, 0x010E2000, 0x010FC000
FS = (0x010C2000, 0x20000)
ALLOWED_WRITES = [(GUARD, 0x1000), (DFU_FLAG, 0x1000), (NVDS, 0x4000), (DIAG, 0x1000), FS]
MAGIC = b'T1GD'


def pytest_addoption(parser):
    for k, v in DEFAULTS.items():
        parser.addoption(f'--{k.replace("_", "-")}', dest=k, default=v, help=f'ELF {k} (défaut : {v})')
    parser.addoption('--firmware', default=None, help='valider uniquement cet ELF (garde-fou avant flash)')
    parser.addoption('--endurance-h', type=float, default=1.0, help="durée virtuelle du test d'endurance (h)")


_symbols = {}


def symbols(path):
    if path not in _symbols:
        from elftools.elf.elffile import ELFFile
        st = ELFFile(open(path, 'rb')).get_section_by_name('.symtab')
        _symbols[path] = {s.name for s in st.iter_symbols()}
    return _symbols[path]


def is_meshcore(path):
    return any('MyMesh' in n for n in symbols(path))


def has_console(path):
    """Console BLE (service GUS) présente : firmware debug, ou exemple blink."""
    return any('BleConsole' in n for n in symbols(path))


def require_console(path):
    if not has_console(path):
        pytest.skip('firmware sans console BLE (release) : test réservé au firmware debug')
    return path


def elf_path(config, kind):
    fw = config.getoption('firmware')
    if fw:
        if not os.path.exists(fw):
            pytest.fail(f'firmware introuvable : {fw}')
        if kind in ('meshcore', 'meshcore_release') and not is_meshcore(fw):
            pytest.skip('tests MeshCore : le firmware validé n\'est pas MeshCore')
        if kind == 'meshcore_release' and has_console(fw):
            pytest.skip('tests de la release : le firmware validé est une debug')
        if kind == 'blink':
            pytest.skip('--firmware : seul ce firmware est validé')
        return fw
    p = config.getoption(kind)
    if not os.path.exists(p):
        pytest.skip(f'firmware {kind} absent : {p}')
    return p


@pytest.fixture(scope='session')
def meshcore_elf(pytestconfig):
    return elf_path(pytestconfig, 'meshcore')


@pytest.fixture(scope='session')
def meshcore_debug_elf(pytestconfig):
    """Compagnon MeshCore avec console BLE (firmware debug) ; sauté sur une release."""
    return require_console(elf_path(pytestconfig, 'meshcore'))


@pytest.fixture(scope='session')
def meshcore_release_elf(pytestconfig):
    return elf_path(pytestconfig, 'meshcore_release')


@pytest.fixture(scope='session')
def blink_elf(pytestconfig):
    return elf_path(pytestconfig, 'blink')


def pytest_generate_tests(metafunc):
    if 'any_elf' in metafunc.fixturenames:
        kinds = ['firmware'] if metafunc.config.getoption('firmware') else ['meshcore', 'blink']
        metafunc.parametrize('any_elf', kinds, indirect=True, scope='session')


@pytest.fixture(scope='session')
def any_elf(request, pytestconfig):
    if request.param == 'firmware':
        return elf_path(pytestconfig, 'any')
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
