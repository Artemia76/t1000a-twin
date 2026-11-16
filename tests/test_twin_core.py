"""Cœur du jumeau : accélération des attentes actives (Twin._clock_poll).

delay(), delayMicroseconds() et les delay_ms du SDK relisent l'horloge en boucle. Le jumeau saute le temps
pendant ces attentes : elles doivent durer ce qui est demandé (au plus ≈ 6 % de plus) et les événements
programmés pendant l'attente doivent être traités à l'heure exacte.
"""
import pytest
from conftest import MAGIC, preset
from twin import T1000A
from twin.core import F_CPU, ms


@pytest.fixture(scope='module')
def node(meshcore_elf):
    n = T1000A(meshcore_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.adc.sources[0] = lambda: 35
    n.gpio.drive(0x24, 1)
    n.run(3000)
    return n


@pytest.mark.parametrize('fn, n, unit', [('delay', 1, 1e-3), ('delay', 10, 1e-3), ('delay', 1000, 1e-3),
                                         ('delayMicroseconds', 50, 1e-6), ('delayMicroseconds', 5000, 1e-6)])
def test_duree_des_attentes(node, fn, n, unit):
    tw = node.tw
    t0, skipped0 = tw.cycles, tw.poll_skipped
    tw.call(fn, n, max_insn=2_000_000)
    dt = (tw.cycles - t0) / F_CPU
    low = (n - 1) * unit if fn == 'delay' else n * unit     # delay() compte en millisecondes entières (millis())
    assert low <= dt <= n * unit * 1.07 + 50e-6, f'{fn}({n}) : {dt * 1e3:.3f} ms'
    if n * unit >= 1e-3 and tw.turbo:
        assert tw.poll_skipped > skipped0, 'attente accélérée'


def test_evenement_a_l_heure_pendant_une_attente(node):
    tw = node.tw
    seen = []
    due = tw.cycles + ms(300)
    tw.schedule(due - tw.cycles, lambda: seen.append(tw.cycles))
    tw.call('delay', 1000, max_insn=2_000_000)
    assert seen == [due], 'événement traité à son échéance, pas après le saut'
