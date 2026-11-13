"""Sécurités du cœur Arduino / variante T1000-A, vérifiées sur chaque firmware destiné au boîtier.

Sans USB ni SWD, ces mécanismes sont le seul moyen de récupérer le T1000-A : un firmware qui les casse
ne doit jamais être flashé.
"""
from conftest import GUARD, DFU_FLAG, MAGIC, flash_writes_outside, preset
from twin import T1000A


def guard(node, n=8):
    return node.tw.flash_bytes(GUARD, n)


def dfu_flag(node):
    return node.tw.flash_bytes(DFU_FLAG, 16)


def test_demarrage_a_froid(any_elf):
    n = T1000A(any_elf)
    n.tw.trace('app_aon_wdt_init')
    n.run(3000)
    assert n.tw.resets == [], n.tw.resets
    assert n.tw.state == 'run'
    assert guard(n, 6) == MAGIC + b'\x00\xff', 'compteur de démarrages : en-tête + 1'
    assert n.wdt.period is not None, 'chien de garde AON armé'
    assert abs(n.wdt.period / 64e6 - 8.0) < 0.01, 'chien de garde de 8 s'
    assert dfu_flag(n)[0] == 0xFF, 'pas de drapeau DFU'
    assert n.ble.adv_on, 'annonce BLE active'
    assert flash_writes_outside(n) == []


def test_troisieme_demarrage_rapide_passe_en_dfu(any_elf):
    n = T1000A(any_elf, flash=preset(guard=MAGIC + b'\x00\x00'))
    n.run(3000)
    assert n.tw.state == 'dfu', 'le 3e démarrage court consécutif doit renvoyer au bootloader'
    assert dfu_flag(n) == b'\x01' + b'\xff' * 15
    assert guard(n, 5) == MAGIC + b'\xff', 'garde remise à zéro'
    assert flash_writes_outside(n) == []


def test_garde_effacee_apres_2_minutes(any_elf):
    n = T1000A(any_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(100_000)
    assert guard(n, 7) == MAGIC + b'\x00\x00\xff', 'compteur conservé avant 2 min'
    n.run(30_000)
    assert guard(n, 5) == MAGIC + b'\xff', 'garde effacée après 2 min de fonctionnement'
    assert n.tw.resets == []
    assert n.wdt.refreshes > 100, 'chien de garde rafraîchi'


def test_garde_effacee_a_la_connexion_ble(any_elf):
    n = T1000A(any_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(3000)
    assert guard(n, 7) == MAGIC + b'\x00\x00\xff'
    n.phone.connect()
    n.run(500)
    assert guard(n, 5) == MAGIC + b'\xff'


def test_bouton_6_s_passe_en_dfu(any_elf):
    n = T1000A(any_elf)
    n.run(3000)
    t0 = n.tw.now_ms()
    n.press()
    n.run(10_000)
    assert n.tw.state == 'dfu', 'bouton maintenu → DFU'
    t = n.tw.resets[0][0] * 1000 // 64_000_000 - t0
    assert 5_900 <= t <= 7_500, f'DFU après {t} ms (attendu ≈ 6 s + 1 s de LED)'
    assert dfu_flag(n)[0] == 0x01
    assert n.wdt.period is None, 'chien de garde coupé avant le DFU'


def test_bouton_relache_avant_6_s(any_elf):
    n = T1000A(any_elf)
    n.run(3000)
    n.press(4000)
    n.run(10_000)
    assert n.tw.state == 'run' and n.tw.resets == []
    assert dfu_flag(n)[0] == 0xFF


def test_chien_de_garde_redemarre_un_firmware_bloque(any_elf):
    n = T1000A(any_elf)
    n.run(3000)
    # le firmware « oublie » de rafraîchir le chien de garde (comme une boucle bloquée)
    n.tw.on('app_aon_wdt_refresh', lambda: 0)
    n.run(9_000)
    assert [r for _, r in n.tw.resets] == ['chien de garde AON'], n.tw.resets
    t = n.tw.resets[0][0] / 64e6 - 3.0
    assert 7.0 <= t <= 9.0, f'redémarrage après {t:.1f} s'


# --- Mutations : un firmware ou un boîtier défaillant doit finir en DFU, jamais en brique ----------------
def test_firmware_bloque_dans_loop_finit_en_dfu(any_elf):
    """loop() remplacée par « b . » : chien de garde → redémarrages courts → 3e démarrage → DFU."""
    n = T1000A(any_elf)
    loop = n.tw.addr('loop')
    n.tw.patch(loop, b'\xfe\xe7')                       # Thumb : b . (boucle infinie)
    n.run(40_000)
    assert n.tw.state == 'dfu', f'firmware bloqué non récupéré : {n.tw.resets}'
    assert [r for _, r in n.tw.resets][:2] == ['chien de garde AON'] * 2


def test_radio_muette_finit_en_dfu(meshcore_elf):
    """LR1110 absent ou mort : MeshCore s'arrête dans setup() ; le boîtier doit rester récupérable."""
    n = T1000A(meshcore_elf, radio=False)
    n.run(60_000)
    assert n.tw.state == 'dfu', f'boîtier sans radio non récupéré : {n.tw.resets}'
