"""Journal d'énergie du compagnon MeshCore (console BLE) : « power », « power log », « power reset », « chg ».

Le T1000-A n'a pas d'appareil de mesure : le firmware compte le temps passé dans chaque état (veille CPU,
BLE connecté ou en annonce, émission LoRa, GNSS) et en garde un échantillon toutes les 10 min.
"""
import re
import pytest
from conftest import MAGIC, preset
from helpers import Console
from twin import T1000A

LOG_HEADER = '# t_s,dt_s,bat_mv,idle_pm,wakes,conn_s,adv_s,gps_s,tx_ms,rx,tx,flags,msio0_mv'


@pytest.fixture
def console(meshcore_debug_elf):
    return Console(T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))).open()


def summary(console):
    r = console.send('power')
    for head in ('power:', 'cpu:', 'ble:', 'lora:', 'gps:', 'bat:', 'log:'):
        assert head in r, f'« {head} » absent de la réponse à power :\n{r}'
    return r


def pct(r, label):
    return float(re.search(label + r' (\d+\.\d) %', r).group(1))


def log_rows(console):
    r = console.send('power log', wait_ms=1000)
    lines = [l for l in r.splitlines() if l.strip()]
    assert lines[0] == LOG_HEADER
    return [l.split(',') for l in lines if not l.startswith('#')]


def test_power_repond(console):
    r = summary(console)
    assert '(open now: 1)' in r, 'le téléphone de la console est connecté'
    assert re.search(r'bat: \d+ mV', r)
    assert 'log: 0/144 samples' in r


def test_veille_cpu_au_repos(console):
    console.send('power reset')
    console.node.run(20000)
    r = summary(console)
    assert pct(r, 'idle') > 80, f'au repos, le CPU doit surtout dormir :\n{r}'
    assert pct(r, 'connected') > 80, 'connexion BLE ouverte pendant toute la mesure'


def test_reset_remet_a_zero(console):
    console.node.run(5000)
    assert 'power: reset' in console.send('power reset')
    r = summary(console)
    assert re.search(r'power: [01] s since reset', r), r
    assert 'log: 0/144 samples' in r


def test_journal_toutes_les_10_min(console):
    console.send('power reset')
    console.node.run(601000)
    rows = log_rows(console)
    assert len(rows) == 1
    t_s, dt_s, bat_mv, idle_pm, wakes, conn_s, adv_s, gps_s, tx_ms, rx, tx, flags, msio0 = rows[0]
    assert t_s == dt_s == '600'
    assert 3000 < int(bat_mv) < 4500
    assert int(idle_pm) > 800
    assert int(conn_s) > 590, 'connexion de la console ouverte tout l\'intervalle'
    assert int(flags, 16) & 0x02, 'flag « BLE connecté » au moment de la lecture'
    assert int(flags, 16) & 0x20, 'GPIO14 (diviseur de mesure) à l\'état haut'


def test_chg(console):
    r = console.send('chg')
    assert re.search(r'chg: aon4=[01] aon5=[01] msio0=-?\d+ mV bat=\d+ mV', r), r


def test_temps_d_emission_lora(meshcore_debug_elf):
    from twin.companion import Companion
    from helpers import GUS_TX
    n = T1000A(meshcore_debug_elf, flash=preset(guard=MAGIC + b'\x00'))
    n.run(4000)
    comp = Companion(n).start()                     # connexion du téléphone (appairage) …
    con = Console(n)
    assert n.phone.subscribe(GUS_TX) in (None, 0)   # … sur laquelle on ouvre aussi la console
    n.run(200)
    con.send('power reset')
    n0 = len(n.radio.tx_packets)
    assert [d['name'] for d in comp.send_advert()] == ['OK']
    n.run(2000)
    sent = n.radio.tx_packets[n0:]
    assert len(sent) == 1
    toa_ms = sent[0]['toa_us'] / 1000
    r = summary(con)
    tx_ms = int(re.search(r'lora: tx (\d+) ms', r).group(1))
    assert toa_ms <= tx_ms < toa_ms + 100, f'temps d\'émission compté {tx_ms} ms, temps d\'antenne {toa_ms:.0f} ms'
    assert 'pkts rx 0 tx 1' in r
