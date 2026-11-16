"""Firmware MeshCore compagnon BLE sur le jumeau T1000-A : radio, BLE, GNSS, persistance, réseau à deux nœuds.

Les scénarios d'un même module partagent un monde démarré une fois (fixtures de module) : l'ordre des
tests compte à l'intérieur d'une classe.
"""
import struct
import pytest
from conftest import flash_writes_outside
from twin import T1000A, World, time_on_air_us
from twin.companion import Companion, decode, NUS_RX
from twin.lr1110 import BW_KHZ, MODE_RX, IRQ_RX_DONE

NOW = 1794571200        # 13/11/2026 12:00 UTC


def self_info(c):
    r = [d for d in c.app_start() if d['code'] == 5]
    assert r, 'pas de SELF_INFO en réponse à APP_START'
    return r[0]


# ----------------------------------------------------------------------------------------------------------
@pytest.fixture(scope='module')
def node(meshcore_elf):
    n = T1000A(meshcore_elf, name='seul')
    n.run(4000)
    return n


@pytest.fixture(scope='module')
def comp(node):
    return Companion(node).start()


@pytest.fixture(scope='module')
def world(meshcore_elf):
    w = World()
    a = w.add(T1000A(meshcore_elf, name='A'))
    b = w.add(T1000A(meshcore_elf, name='B'))
    w.run(4000)
    ca, cb = Companion(a).start(), Companion(b).start()
    w.info = {'A': self_info(ca), 'B': self_info(cb)}
    ca.set_time(NOW)
    cb.set_time(NOW)
    w.a, w.b, w.ca, w.cb = a, b, ca, cb
    return w


class TestNoeudSeul:
    def test_radio_initialisee_et_en_reception(self, node):
        r = node.radio
        assert node.tw.resets == []
        assert r.unknown_cmds == [], 'commandes LR1110 non modélisées : ' + str(r.unknown_cmds)
        assert r.violations == [], 'protocole SPI/BUSY non respecté : ' + str(r.violations)
        assert r.packet_type == 2, 'LoRa'
        # écoute cyclique : la puce dort entre deux fenêtres (mode SLEEP), c'est attendu
        assert (r.mode == MODE_RX and r.rx_continuous) or r.duty is not None, 'en réception (continue ou cyclique)'
        assert r.dio_mask & IRQ_RX_DONE, 'RX_DONE routé sur la ligne IRQ'
        assert r.tcxo is not None, 'TCXO configuré (LR11X0_DIO3_TCXO_VOLTAGE)'

    def test_annonce_ble_nom_mac(self, node):
        name = node.ble.adv_name()
        mac = node.ble.mac[::-1].hex().upper()
        assert name == f'MeshCore-{mac}'

    def test_ecriture_refusee_sans_appairage(self, node):
        ph = node.phone
        ph.connect()
        node.run(200)
        err = ph.write(NUS_RX, bytes([22, 3]))
        assert err == 0x05, 'NUS RX doit exiger un lien chiffré et authentifié'
        ph.disconnect()
        node.run(500)
        assert node.ble.adv_on, "l'annonce doit reprendre après la déconnexion"

    def test_mauvais_code_refuse(self, node):
        ph = node.phone
        good = ph.passkey
        ph.passkey = 111111
        ph.connect()
        node.run(100)
        ph.pair()
        node.run(500)
        assert ph.pair_status not in (None, 0), 'un code faux doit faire échouer l\'appairage'
        ph.disconnect()
        ph.passkey = good
        ph.pair_status = None
        node.run(500)

    def test_device_info_et_self_info(self, node, comp):
        di = [d for d in comp.device_query() if d['code'] == 13]
        assert di and di[0]['max_contacts'] == 350 and di[0]['max_channels'] == 40
        si = self_info(comp)
        r = node.radio
        assert si['node_name'] == node.ble.mac[::-1].hex().upper()
        assert si['freq_khz'] == round(r.freq / 1000)
        assert si['sf'] == r.sf and si['bw_hz'] == int(BW_KHZ[r.bw] * 1000)
        assert si['tx_power'] == r.power == 22

    def test_batterie_mesuree_via_gpio14(self, node, comp):
        node.battery_mv = 3712
        r = [d for d in comp.battery() if d['code'] == 12]
        assert r and abs(r[0]['battery_mv'] - 3712) <= 20

    @pytest.mark.xfail(reason='point ouvert : GPIO14 (pont batterie, peut-être alimentation RF) reste à 1 en '
                              'permanence ; à ne couper que pendant la mesure une fois son rôle confirmé', strict=True)
    def test_pont_batterie_coupe_hors_mesure(self, node, comp):
        comp.battery()
        node.run(100)
        assert node.gpio.level(14) == 0

    def test_advert_emis_valide(self, node, comp):
        n0 = len(node.radio.tx_packets)
        assert [d['name'] for d in comp.send_advert()] == ['OK']
        node.run(2000)
        tx = node.radio.tx_packets[n0:]
        assert len(tx) == 1
        p = tx[0]
        hdr = p['data'][0]
        assert hdr & 0x03 == 1 and (hdr >> 2) & 0x0F == 4, f'en-tête {hdr:#x} : advert en inondation attendu'
        assert p['power'] == 22
        expect = time_on_air_us(len(p['data']), p['sf'], BW_KHZ[p['bw']], p['cr'], p['preamble'])
        assert p['toa_us'] == expect
        r = node.radio                                   # écoute cyclique : la puce peut dormir entre deux fenêtres
        assert r.mode == MODE_RX or r.duty is not None, 'retour en réception après l\'émission'

    def test_gnss_position_dans_self_info(self, node, comp):
        node.gnss.fix(48.8584, 2.2945, 35.0)
        r = comp.query(b'\x29gps:1', expect=0)
        assert r and r[-1]['code'] == 0
        node.run(500)
        assert node.gnss.running, 'GNSS alimenté (GPS_EN haut, reset relâché)'
        node.run(5000)
        assert node.uart.tx_log is not None
        si = self_info(comp)
        assert abs(si['lat'] - 48.8584) < 1e-4 and abs(si['lon'] - 2.2945) < 1e-4

    def test_gnss_coupe(self, node, comp):
        r = comp.query(b'\x29gps:0', expect=0)
        assert r and r[-1]['code'] == 0
        node.run(500)
        assert not node.gnss.running, 'GNSS hors tension après gps:0'

    def test_pas_d_ecriture_flash_hors_zones(self, node):
        assert flash_writes_outside(node) == []

    def test_endurance(self, node, comp, pytestconfig):
        """N h virtuelles (--endurance-h, 1 par défaut) connecté au téléphone : ni redémarrage, ni dérive de
        l'horloge, ni fuite de tas."""
        hours = pytestconfig.getoption('endurance_h')
        tw = node.tw
        start = NOW + 86400            # l'horloge a pu être réglée par le GNSS : on avance (MeshCore refuse de reculer)
        assert [d['code'] for d in comp.set_time(start)][-1:] == [0]
        t0 = tw.now_ms()
        heap = lambda: (tw.r32(tw.var['heap_end.0']), tw.r32(tw.var['__malloc_max_sbrked_mem']))
        h0, n_resets, refresh0 = heap(), len(tw.resets), node.wdt.refreshes
        node.run(int(hours * 3_600_000))
        r = [d for d in comp.query(bytes([5]), expect=9) if d['code'] == 9]
        assert r, 'pas de réponse à GET_DEVICE_TIME'
        epoch = struct.unpack_from('<I', r[0]['raw'], 1)[0]
        elapsed = (tw.now_ms() - t0) / 1000
        assert abs(epoch - (start + elapsed)) <= 2, f'horloge : {epoch - start} s pour {elapsed:.0f} s écoulées'
        assert len(tw.resets) == n_resets, tw.resets
        assert heap() == h0, f'le tas a grossi : {h0} → {heap()}'
        assert node.wdt.refreshes - refresh0 > hours * 3600 / 8, 'chien de garde rafraîchi pendant la veille'
        assert node.phone.connected

    def test_veille_entre_deux_tics(self, node):
        tw = node.tw
        s0, c0 = tw.slept, tw.cycles
        node.run(10_000)
        ratio = (tw.slept - s0) / (tw.cycles - c0)
        assert ratio > 0.95, f'le MCU ne dort que {ratio:.0%} du temps au repos'


# ----------------------------------------------------------------------------------------------------------
class TestDeuxNoeuds:
    def test_identites_distinctes(self, world):
        assert world.info['A']['pubkey'] != world.info['B']['pubkey']
        assert world.a.ble.mac != world.b.ble.mac

    def test_advert_recu_et_contact_ajoute(self, world):
        world.cb.send_advert()
        world.run(3000)
        names = [decode(f)['name'] for f in world.ca.inbox]
        assert 'PUSH_ADVERT' in names or 'PUSH_NEW_ADVERT' in names
        ct = world.ca.contacts()
        assert [c['pubkey'] for c in ct] == [world.info['B']['pubkey']]
        world.ca.send_advert()
        world.run(3000)
        assert [c['pubkey'] for c in world.cb.contacts()] == [world.info['A']['pubkey']]

    def test_message_direct_et_accuse(self, world):
        n = len(world.ca.inbox)
        r = world.ca.send_text(world.info['B']['pubkey'], 'Bonjour du jumeau A', ts=NOW + 10)
        assert any(d['name'] == 'SENT' for d in r)
        world.run(8000)
        assert 'PUSH_SEND_CONFIRMED' in [decode(f)['name'] for f in world.ca.inbox[n:]], 'ACK non reçu'
        msgs = world.cb.sync_messages()
        assert [m.get('text') for m in msgs] == ['Bonjour du jumeau A']

    def test_redemarrage_conserve_identite_et_contacts(self, world):
        a, ca = world.a, world.ca
        world.run(6000)                                 # écriture différée des contacts (5 s)
        r = ca.query(bytes([19]) + b'reboot', max_ms=500)
        world.run(5000)
        assert a.tw.resets, 'CMD_REBOOT doit redémarrer le MCU'
        assert not a.phone.connected
        ca2 = Companion(a).start()
        si = self_info(ca2)
        assert si['pubkey'] == world.info['A']['pubkey'], 'identité perdue au redémarrage'
        assert [c['pubkey'] for c in ca2.contacts()] == [world.info['B']['pubkey']], 'contacts perdus'

    def test_collision_deux_emetteurs(self, world):
        """Deux adverts simultanés : aucun n'est reçu intact par un 3e récepteur (pas d'effet de capture)."""
        rx = world.ether.radios
        before = {r.tw.name: len(r.rx_packets) for r in rx}
        world.ether.inject(b'\x11' + bytes(60), rssi=-80)
        world.ether.inject(b'\x11' + bytes(60), rssi=-82)
        world.run(2000)
        for r in rx:
            got = r.rx_packets[before[r.tw.name]:]
            assert all(not p['ok'] for p in got), f'{r.tw.name} a reçu intact un paquet en collision'
