#!/usr/bin/env python3
"""Scénarios de validation d'un croquis construit avec platform-gr551x (cœur Arduino) pour le T1000-A.

Vérifie la séquence de démarrage du cœur et les sécurités de la variante T1000-A :
constructeurs C++, garde de démarrage, chien de garde, retour au DFU (bouton, console BLE),
écritures Flash limitées aux zones prévues.

Usage : test_arduino.py firmware.elf [--console]   (--console : le croquis utilise BleConsole)
"""
import re, struct, sys
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from gr5515_sim import GR5515Sim, Stop, Reset

ELF = sys.argv[1]
CONSOLE = '--console' in sys.argv
GUARD, FLAG, NVDS, DIAG = 0x010E3000, 0x010E4000, 0x010FC000, 0x010E2000
FS = (0x010C2000, 0x20000)
ALLOWED_WRITES = [(GUARD, 0x1000), (FLAG, 0x1000), (NVDS, 0x1000), (DIAG, 0x1000), FS]
MAGIC = b'T1GD'
F_CPU = 64_000_000
results = []


def check(name, cond, detail=''):
    results.append((name, bool(cond), detail))
    print(f'  [{"OK " if cond else "ÉCHEC"}] {name}{" — " + detail if detail and not cond else ""}')


def _lookup(table, name):
    """Symbole `name` tel quel (C) ou décoré C++ (_Z<len>name…, _ZL<len>name… pour un symbole statique)."""
    if name in table:
        return name
    pref = re.compile(r'_ZL?%d%s' % (len(name), re.escape(name)))   # la longueur rend le nom exact
    names = [n for n in table if pref.match(n)]
    if len(names) != 1:
        raise KeyError(f'{name} : {names or "introuvable"}')
    return names[0]


def sym(sim, name):
    return _lookup(sim.sym, name)


def var(sim, name):
    return sim.var[_lookup(sim.var, name)]


def millis(sim):
    a = var(sim, 's_cyc_total')
    return (sim.r32(a) | sim.r32(a + 4) << 32) // (F_CPU // 1000)


def advance(ms):
    """Fait passer `ms` millisecondes à chaque tour de loop() (compteur DWT simulé)."""
    def f(sim):
        sim.cyc = (sim.cyc + ms * (F_CPU // 1000)) & 0xFFFFFFFF
    return f


def writes_ok(sim):
    bad = [e for e in sim.events if e[0] in ('flash_write', 'flash_erase')
           and not any(a <= e[1] < a + n for a, n in ALLOWED_WRITES)]
    return not bad, bad


def new(preset=None):
    s = GR5515Sim(ELF, flash_preset=preset)
    s.trace('app_aon_wdt_init', 'app_aon_wdt_deinit', 'app_aon_wdt_refresh', 'safety_enter_dfu',
            'system_platform_init', 'setup', 'loop', 'ble_stack_init')
    if CONSOLE:
        s.trace('gus_service_init', 'ble_gap_adv_start')
    return s


def dfu_flag_set(s):
    return s.flash_bytes(FLAG, 1) == b'\x01' and s.flash_bytes(FLAG + 1, 15) == b'\xff' * 15


def ble_event(sim, evt_id, status=0, index=0):
    evt = 0x3003F200
    sim.uc.mem_write(evt, struct.pack('<HHB', evt_id, status, index) + bytes(0x28 - 5))
    sim.call(sym(sim, 'ble_evt_handler'), evt)


BLE_COMMON_EVT_STACK_INIT = 0x0100
BLE_GAPC_EVT_CONNECTED = 0x0301
buf, evt = 0x3003F000, 0x3003F100


def console_cmd(sim, text):
    sim.uc.mem_write(buf, text)
    sim.uc.mem_write(evt, struct.pack('<BBxxIHxx', 1, 0, buf, len(text)))   # GUS_EVT_RX_DATA_RECEIVED
    sim.call(sym(sim, 'gus_evt'), evt)


print('A1 — démarrage à froid : constructeurs, setup(), loop(), sécurités')
s = new()
r = s.boot()
check('atteint la boucle principale', r == 'atteint pwr_mgmt_schedule', r)
check('system_platform_init exécutée une seule fois', s.calls.count('system_platform_init') == 1,
      str(s.calls.count('system_platform_init')))
check('setup() puis loop() appelés', 'setup' in s.calls and 'loop' in s.calls
      and s.calls.index('setup') < s.calls.index('loop'))
check('pile BLE initialisée avant setup()', 'ble_stack_init' in s.calls
      and s.calls.index('ble_stack_init') < s.calls.index('setup'))
check('constructeur de Serial exécuté (vtable)', s.r32(var(s, 'Serial')) != 0)
check('compteur de démarrages : en-tête + 1', s.flash_bytes(GUARD, 6) == MAGIC + b'\x00\xff')
check('chien de garde initialisé', 'app_aon_wdt_init' in s.calls)
check('aucun drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')
ok, bad = writes_ok(s); check('écritures Flash limitées aux zones prévues', ok, str(bad))

print('A2 — deux démarrages courts déjà comptés → le 3e bascule en DFU')
s = new({GUARD: MAGIC + b'\x00\x00'})
try:
    s.boot(); reset = False
except Reset:
    reset = True
check('redémarrage demandé', reset)
check('drapeau DFU = 0x01', dfu_flag_set(s), s.flash_bytes(FLAG, 4).hex())
check('setup() jamais atteint', 'setup' not in s.calls)

print('A3 — 3 min de fonctionnement : chien de garde rafraîchi, garde effacée après 2 min')
s = new({GUARD: MAGIC + b'\x00'})
s.on_idle = advance(100)
r = s.run_main_loop(idles=1000)                       # ≈ 100 s
check('boucle principale tourne', 'passages en veille' in r, r)
check('compteur conservé avant 2 min', s.flash_bytes(GUARD, 7) == MAGIC + b'\x00\x00\xff',
      s.flash_bytes(GUARD, 7).hex())
r = s.resume(idles=800)                               # ≈ 180 s au total
check('garde effacée après 2 min', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff', s.flash_bytes(GUARD, 7).hex())
check('chien de garde rafraîchi régulièrement', s.calls.count('app_aon_wdt_refresh') > 1000,
      str(s.calls.count('app_aon_wdt_refresh')))
check('temps écoulé cohérent (≈ 180 s)', 170_000 <= millis(s) <= 200_000, str(millis(s)))
leds = [e[3] for e in s.events if e[0] == 'io' and e[2] == 0x8]
check('le croquis pilote la LED', 0 in leds and 1 in leds, str(leds[:10]))
ok, bad = writes_ok(s); check('écritures Flash limitées', ok, str(bad))

print('A4 — bouton maintenu → DFU après 6 s, pas avant')
s = new()
s.on_idle = advance(10)
s.button = 1
t_dfu = None
try:
    s.run_main_loop(idles=2000)
except Reset:
    t_dfu = millis(s)
check('bascule en DFU', t_dfu is not None)
check('après 6 s environ', t_dfu is not None and 5_900 <= t_dfu <= 7_200, str(t_dfu))
check('drapeau DFU écrit', dfu_flag_set(s))
check('chien de garde coupé avant le DFU', 'app_aon_wdt_deinit' in s.calls)

print('A5 — bouton relâché avant 6 s → pas de DFU')
s = new()
s.on_idle = lambda sim: (advance(10)(sim), setattr(sim, 'button', 1 if millis(sim) < 4000 else 0))
try:
    s.run_main_loop(idles=1500); reset = False
except Reset:
    reset = True
check('pas de redémarrage', not reset)
check('pas de drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')

print('A6 — pile BLE prête puis connexion : garde effacée')
s = new({GUARD: MAGIC + b'\x00'})
s.on_idle = advance(10)
s.run_main_loop(idles=5)
ble_event(s, BLE_COMMON_EVT_STACK_INIT)
if CONSOLE:
    check('service console ajouté à la pile prête', 'gus_service_init' in s.calls)
    check('annonces BLE démarrées', 'ble_gap_adv_start' in s.calls)
ble_event(s, BLE_GAPC_EVT_CONNECTED)
check('garde effacée à la connexion', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff')
s.resume(idles=5)

if CONSOLE:
    print('A7 — console BLE : « dfu »')
    s = new()
    s.on_idle = advance(10)
    s.run_main_loop(idles=5)
    ble_event(s, BLE_COMMON_EVT_STACK_INIT)
    console_cmd(s, b'dfu\r\n')
    try:
        s.resume(idles=200); reset = False
    except Reset:
        reset = True
    check('bascule en DFU', reset)
    check('drapeau DFU écrit', dfu_flag_set(s))

    print('A8 — console BLE : commande inconnue puis « reboot »')
    s = new({GUARD: MAGIC + b'\x00'})
    s.on_idle = advance(10)
    s.run_main_loop(idles=5)
    ble_event(s, BLE_COMMON_EVT_STACK_INIT)
    for cmd in (b'xyz\r\n', b'reboot\r\n'):            # deux lignes reçues d'affilée
        console_cmd(s, cmd)
    try:
        s.resume(idles=200); reset = False
    except Reset:
        reset = True
    check('redémarrage', reset)
    check('pas de drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')
    check('compteur effacé', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff')

failed = [r for r in results if not r[1]]
print(f'\n{len(results) - len(failed)}/{len(results)} vérifications réussies')
sys.exit(1 if failed else 0)
