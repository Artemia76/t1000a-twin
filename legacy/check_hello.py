#!/usr/bin/env python3
"""Scénarios de validation du firmware de test T1000-A dans l'émulateur GR5515.

Chaque scénario part d'une Flash simulée (bootloader non modélisé, zone appli = notre image),
démarre le firmware et vérifie un comportement de sécurité. Code de sortie 0 si tout passe.
"""
import os, struct, sys
_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [_here, os.path.dirname(_here)]   # legacy/ (gr5515_sim) et la racine du jumeau (twin)
from gr5515_sim import GR5515Sim, Stop, Reset
from twin.paths import in_root

ELF = sys.argv[1] if len(sys.argv) > 1 else in_root('firmware', 'hello', 'build', 't1000a_hello.elf')
GUARD, FLAG, NVDS, DIAG = 0x010E3000, 0x010E4000, 0x010FC000, 0x010E2000
ALLOWED_WRITES = [(GUARD, 0x1000), (FLAG, 0x1000), (NVDS, 0x1000), (DIAG, 0x1000)]
MAGIC = b'T1GD'
TICK_MS = 50
results = []


def check(name, cond, detail=''):
    results.append((name, bool(cond), detail))
    print(f'  [{"OK " if cond else "ÉCHEC"}] {name}{" — " + detail if detail and not cond else ""}')


def timer_irq(sim):
    """Interruption simulée du temporisateur de 50 ms (tick_cb du firmware)."""
    up, pend = sim.var['s_uptime_ms'], sim.var['s_tick_pending']
    sim.w32(up, sim.r32(up) + TICK_MS)
    sim.uc.mem_write(pend, b'\x01')


def writes_ok(sim):
    bad = [e for e in sim.events if e[0] in ('flash_write', 'flash_erase')
           and not any(a <= e[1] < a + n for a, n in ALLOWED_WRITES)]
    return not bad, bad


def new(preset=None):
    s = GR5515Sim(ELF, flash_preset=preset)
    s.trace('app_aon_wdt_init', 'app_aon_wdt_deinit', 'app_aon_wdt_refresh', 'safety_enter_dfu', 'console_poll')
    return s


def dfu_flag_set(s):
    return s.flash_bytes(FLAG, 1) == b'\x01' and s.flash_bytes(FLAG + 1, 15) == b'\xff' * 15


print('S1 — démarrage à froid, Flash vierge')
s = new()
r = s.boot()
check('atteint la boucle principale', r == 'atteint pwr_mgmt_schedule', r)
check('compteur de démarrages : en-tête + 1', s.flash_bytes(GUARD, 6) == MAGIC + b'\x00\xff')
check('chien de garde initialisé', 'app_aon_wdt_init' in s.calls)
check('aucun drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')
ok, bad = writes_ok(s); check('écritures Flash limitées aux zones prévues', ok, str(bad))

print('S2 — secteur de garde contenant des données étrangères (ancien firmware)')
s = new({GUARD: bytes(range(256)) * 4})
r = s.boot()
check('démarre normalement', r == 'atteint pwr_mgmt_schedule', r)
check('garde réinitialisée : en-tête + 1', s.flash_bytes(GUARD, 6) == MAGIC + b'\x00\xff')
check('pas de DFU', s.flash_bytes(FLAG, 1) == b'\xff')

print('S3 — deux démarrages rapides déjà comptés → le 3e bascule en DFU')
s = new({GUARD: MAGIC + b'\x00\x00'})
try:
    s.boot(); reset = False
except Reset:
    reset = True
check('redémarrage demandé', reset)
check('drapeau DFU = 0x01 à 0x010E4000', dfu_flag_set(s), s.flash_bytes(FLAG, 4).hex())
check('garde remise à zéro', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff')
ok, bad = writes_ok(s); check('écritures Flash limitées', ok, str(bad))

print('S4 — un démarrage rapide déjà compté → démarrage normal, compteur à 2')
s = new({GUARD: MAGIC + b'\x00'})
r = s.boot()
check('démarre normalement', r == 'atteint pwr_mgmt_schedule', r)
check('compteur à 2', s.flash_bytes(GUARD, 7) == MAGIC + b'\x00\x00\xff')

print('S5 — 60 s de fonctionnement SANS connexion BLE → compteur conservé')
s = new()
s.on_idle = timer_irq
r = s.run_main_loop(idles=61000 // TICK_MS + 5)
check('boucle principale tourne', 'passages en veille' in r, r)
check('compteur conservé (pas de connexion)', s.flash_bytes(GUARD, 6) == MAGIC + b'\x00\xff')
check('chien de garde rafraîchi régulièrement', s.calls.count('app_aon_wdt_refresh') > 1000,
      str(s.calls.count('app_aon_wdt_refresh')))
leds = [e[3] for e in s.events if e[0] == 'io' and e[1] == 3 and e[2] == 0x8]
check('LED clignote (battement)', 0 in leds and 1 in leds, str(leds[:10]))

print('S6 — bouton maintenu → DFU après 6 s, pas avant')
s = new()
s.on_idle = timer_irq
s.button = 1
reset_at = None
try:
    s.run_main_loop(idles=200)
except Reset:
    reset_at = s.r32(s.var['s_uptime_ms'])
check('bascule en DFU', reset_at is not None)
check('après 6 s environ', reset_at is not None and 5900 <= reset_at <= 6200, str(reset_at))
check('drapeau DFU écrit', dfu_flag_set(s))
i_deinit = s.calls.index('app_aon_wdt_deinit') if 'app_aon_wdt_deinit' in s.calls else None
check('chien de garde coupé avant le passage en DFU', i_deinit is not None)
check('LED allumée pendant l\'appui', any(e[0] == 'io' and e[2] == 0x8 and e[3] == 1 for e in s.events))

print('S7 — bouton relâché avant 6 s → pas de DFU')
s = new()
s.on_idle = lambda sim: (timer_irq(sim), setattr(sim, 'button', 1 if sim.idle_count < 80 else 0))
try:
    r = s.run_main_loop(idles=200); reset = False
except Reset:
    reset = True
check('pas de redémarrage', not reset)
check('pas de drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')

print('S8 — commande « dfu » reçue sur la console BLE')
s = new()
s.on_idle = timer_irq
s.run_main_loop(idles=3)
buf, evt = 0x3003F000, 0x3003F100            # zone libre en haut de RAM (sous la pile)
s.uc.mem_write(buf, b'dfu\r\n')
s.uc.mem_write(evt, struct.pack('<BBxxIHxx', 1, 0, buf, 5))   # GUS_EVT_RX_DATA_RECEIVED
s.call('gus_evt', evt)
try:
    s.resume(idles=5); reset = False
except Reset:
    reset = True
check('bascule en DFU', reset)
check('drapeau DFU écrit', dfu_flag_set(s))

print('S9 — commande inconnue puis « reboot » : redémarrage sans DFU, compteur effacé')
s = new({GUARD: MAGIC + b'\x00'})
s.on_idle = timer_irq
s.run_main_loop(idles=3)
for cmd in (b'xyz\r\n', b'reboot\r\n'):
    s.uc.mem_write(buf, cmd)
    s.uc.mem_write(evt, struct.pack('<BBxxIHxx', 1, 0, buf, len(cmd)))
    s.call('gus_evt', evt)
    try:
        s.resume(idles=3); reset = False
    except Reset:
        reset = True
check('redémarrage', reset)
check('pas de drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')
check('compteur effacé', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff')

print('S13 — deux commandes reçues d\'affilée avant traitement : aucune perdue')
s = new()
s.on_idle = timer_irq
s.run_main_loop(idles=3)
for cmd in (b'led on\r\n', b'reboot\r\n'):          # deux événements sans passage par la boucle
    s.uc.mem_write(buf, cmd)
    s.uc.mem_write(evt, struct.pack('<BBxxIHxx', 1, 0, buf, len(cmd)))
    s.call('gus_evt', evt)
try:
    s.resume(idles=3); reset = False
except Reset:
    reset = True
check('1re commande exécutée (LED allumée)', any(e[0] == 'io' and e[2] == 0x8 and e[3] == 1 for e in s.events))
check('2e commande exécutée (redémarrage)', reset)

print('S14 — « lr init » sans radio : ni faute ni blocage, échec propre de begin()')
s = new()
s.on_idle = timer_irq
s.run_main_loop(idles=3)
cmd = b'lr init\r\n'
s.uc.mem_write(buf, cmd)
s.uc.mem_write(evt, struct.pack('<BBxxIHxx', 1, 0, buf, len(cmd)))
s.call('gus_evt', evt)
try:
    r = s.resume(idles=40, max_insn=400_000_000); stop = None
except (Stop, Reset) as e:
    r, stop = None, e
steps = [b for b in s.flash_bytes(0x010E2004, 32) if b != 0xFF]
check('pas de faute ni de blocage', stop is None and r and 'passages en veille' in r, str(stop or r))
check('fil d\'Ariane jusqu\'à begin() (10…14, 20)', steps[:7] == [10, 11, 12, 13, 14, 20, 91], str(steps))
check('aucune faute enregistrée', s.flash_bytes(0x010E2100, 4) == b'\xff' * 4)

BLE_GAPC_EVT_CONNECTED = 0x0301


def ble_connect(sim):
    evt = 0x3003F200
    sim.uc.mem_write(evt, struct.pack('<HHB', BLE_GAPC_EVT_CONNECTED, 0, 0) + bytes(0x28 - 5))
    sim.call('ble_evt_handler', evt)


print('S10 — connexion BLE → compteur remis à zéro, plus de redémarrage automatique')
s = new({GUARD: MAGIC + b'\x00'})
s.on_idle = timer_irq
s.run_main_loop(idles=20)
check('compteur à 2 avant connexion', s.flash_bytes(GUARD, 7) == MAGIC + b'\x00\x00\xff')
ble_connect(s)
check('compteur effacé à la connexion', s.flash_bytes(GUARD, 5) == MAGIC + b'\xff')
try:
    s.resume(idles=6 * 60 * 1000 // TICK_MS); reset = False
except Reset:
    reset = True
check('pas de redémarrage après 6 min une fois connecté', not reset)

print('S11 — aucune connexion BLE pendant 5 min → redémarrage qui conserve le compteur')
s = new()
s.on_idle = timer_irq
reset_at = None
try:
    s.run_main_loop(idles=6 * 60 * 1000 // TICK_MS)
except Reset:
    reset_at = s.r32(s.var['s_uptime_ms'])
check('redémarrage', reset_at is not None)
check('vers 5 min', reset_at is not None and 299_000 <= reset_at <= 301_000, str(reset_at))
check('compteur conservé (en-tête + 1)', s.flash_bytes(GUARD, 6) == MAGIC + b'\x00\xff')
check('pas de drapeau DFU', s.flash_bytes(FLAG, 1) == b'\xff')
check('chien de garde coupé avant le redémarrage', 'app_aon_wdt_deinit' in s.calls)

print('S12 — BLE muet de bout en bout : démarrages successifs jusqu\'au DFU')
flash = None
outcome = []
for boot in range(1, 5):
    # On ne transmet d'un démarrage à l'autre que la garde et le drapeau DFU : le NVDS écrit par
    # l'émulateur (en-tête issu de la ROM factice) n'a pas de sens au démarrage suivant.
    s = new(None if flash is None else {GUARD: flash[GUARD - 0x01000000:][:0x1000],
                                        FLAG: flash[FLAG - 0x01000000:][:0x1000]})
    s.on_idle = timer_irq
    try:
        s.run_main_loop(idles=6 * 60 * 1000 // TICK_MS)
        outcome.append('tourne')
    except Reset:
        outcome.append('DFU' if dfu_flag_set(s) else 'redémarrage')
    flash = bytes(s.flash)
    if outcome[-1] == 'DFU':
        break
check('séquence redémarrage, redémarrage, DFU', outcome == ['redémarrage', 'redémarrage', 'DFU'], str(outcome))
check('DFU atteint en ≈ 10 min (2 × 5 min)', len(outcome) == 3)

failed = [r for r in results if not r[1]]
print(f'\n{len(results) - len(failed)}/{len(results)} vérifications réussies')
sys.exit(1 if failed else 0)
