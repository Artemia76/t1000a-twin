"""GNSS périodique en intérieur : arrêt rapide sans satellite capté, puis essais espacés (10 → 30 min).

Avec `gps period 10m`, une fenêtre d'acquisition dure jusqu'au fix, au plus 5 min (1 h avant le premier fix).
Si aucun satellite n'a de SNR dans les trames GSV au bout de 60 s, la fenêtre s'arrête et l'essai suivant est
repoussé : période, 2 × période… au plus 30 min. Un satellite capté ou un fix remet le rythme normal.
Le firmware force la sortie GSV ($PAIR062,3,1) au début de chaque fenêtre.

Les scénarios partagent un nœud : l'ordre des tests compte.
"""
import pytest
from twin import T1000A
from twin.companion import Companion

from helpers import NOW
MIN = 60_000
CLI, CLI_REPLY = 66, 29


def cli(c, line):
    r = [d for d in c.query(bytes([CLI]) + line.encode(), expect=CLI_REPLY) if d['code'] == CLI_REPLY]
    assert r, f'pas de réponse à « {line} »'
    return r[0]['raw'][1:].decode()


def windows(node, since_ms=0):
    """Fenêtres d'allumage du GNSS [(début_ms, fin_ms ou None)], en fusionnant les coupures < 1 s (impulsion de
    reset au démarrage d'une fenêtre)."""
    tw = node.tw
    ev = sorted([(t * 1000 // 64_000_000, 1) for t, _ in tw.events('gnss_on')] +
                [(t * 1000 // 64_000_000, 0) for t, _ in tw.events('gnss_off')])
    out = []
    for t, on in ev:
        if on:
            if out and out[-1][1] is not None and t - out[-1][1] < 1000:
                out[-1] = (out[-1][0], None)
            elif not out or out[-1][1] is not None:
                out.append((t, None))
        elif out and out[-1][1] is None:
            out[-1] = (out[-1][0], t)
    return [w for w in out if (w[1] or 1 << 62) >= since_ms]


@pytest.fixture(scope='module')
def node(meshcore_elf):
    n = T1000A(meshcore_elf, name='gnss')
    n.gnss.no_sky()
    n.gnss.gsv_rate = 0                 # réglage enregistré qui coupe GSV : le firmware doit la rétablir
    n.run(4000)
    n.comp = Companion(n).start()
    n.comp.app_start()
    n.comp.set_time(NOW)
    return n


def test_sans_ciel_arret_apres_60_s(node):
    t0 = node.tw.now_ms()
    assert cli(node.comp, 'gps period 10m') == 'OK'
    node.run(2 * MIN)
    assert any(c.startswith('$PAIR062,3,1') for c in node.gnss.commands), node.gnss.commands
    assert node.gnss.gsv_rate == 1, 'GSV rétablie par le firmware'
    w = windows(node, t0)
    assert len(w) == 1 and w[0][1] is not None, f'GNSS toujours allumé (limite d\'1 h avant le premier fix) : {w}'
    on_s = (w[0][1] - w[0][0]) / 1000
    assert 55 <= on_s <= 66, f'fenêtre sans ciel de {on_s:.0f} s'


def test_sans_ciel_essais_espaces_jusqu_a_30_min(node):
    t0 = windows(node)[-1][0]
    node.run(100 * MIN - (node.tw.now_ms() - t0))
    w = windows(node, t0)
    starts = [s for s, _ in w]
    gaps = [round((b - a) / MIN) for a, b in zip(starts, starts[1:])]
    # écart entre deux débuts = fenêtre de 1 min + attente de 10, 20, puis 30 min au plus
    assert gaps[:4] == [11, 21, 31, 31][:len(gaps)] and len(gaps) >= 3, f'écarts {gaps} min'
    for s, e in w:
        assert e is not None and (e - s) / 1000 <= 66, f'fenêtre de {(e - s) / 1000:.0f} s'
    # régime établi : une fenêtre d'1 min toutes les 31 min (au lieu de 5 min sur 10 avant)
    share = (w[-2][1] - w[-2][0]) / (w[-1][0] - w[-2][0])
    assert share < 0.035, f'GNSS allumé {share:.1%} du temps en régime établi'


def test_retour_au_ciel_fix_puis_rythme_normal(node):
    node.gnss.fix(48.8584, 2.2945, 35.0)
    t0 = node.tw.now_ms()
    assert cli(node.comp, 'gps period 10m') == 'OK'     # relance immédiate (ex. l'utilisateur sort)
    node.run(25 * MIN)
    w = windows(node, t0)
    assert len(w) >= 3, w
    assert (w[0][1] - w[0][0]) / 1000 < 30, 'fix rapide puis arrêt'
    gaps = [round((b[0] - a[0]) / MIN) for a, b in zip(w, w[1:])]
    assert all(g == 10 for g in gaps), f'rythme normal de 10 min attendu : {gaps}'


def test_ciel_faible_fenetre_complete(node):
    node.gnss.weak_sky(4)
    w0 = windows(node)[-1]
    node.run(max(0, w0[1] + 10 * MIN + 2000 - node.tw.now_ms()))    # début de la fenêtre suivante
    t0 = node.tw.now_ms() - 3000
    node.run(17 * MIN)
    w = windows(node, t0)
    assert len(w) >= 2, w
    on_s = (w[0][1] - w[0][0]) / 1000
    assert 295 <= on_s <= 305, f'des satellites sont captés : la fenêtre doit durer 5 min, pas {on_s:.0f} s'
    assert round((w[1][0] - w[0][1]) / MIN) == 10, 'pas de veille lente quand des satellites sont captés'


def test_mode_continu_jamais_coupe(node):
    node.gnss.no_sky()
    t0 = node.tw.now_ms()
    assert cli(node.comp, 'gps period 0') == 'OK'
    node.run(4 * MIN)
    w = windows(node, t0)
    assert w and w[-1][1] is None and node.gnss.running, f'GNSS coupé en mode continu : {w}'
