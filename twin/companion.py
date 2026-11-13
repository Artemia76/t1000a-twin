"""Client du protocole compagnon MeshCore côté téléphone virtuel (service Nordic UART).

Une écriture sur NUS RX = une trame ; une notification sur NUS TX = une trame (comme tools/meshcore_ble.py).

    c = Companion(node)
    c.start()                               # connexion, appairage (code), abonnement
    info = c.query(bytes([22, 3]))          # DEVICE_QUERY -> trames reçues
"""
import struct


def _uuid(s):
    return bytes.fromhex(s.replace('-', ''))[::-1]


NUS_RX = _uuid('6e400002-b5a3-f393-e0a9-e50e24dcca9e')
NUS_TX = _uuid('6e400003-b5a3-f393-e0a9-e50e24dcca9e')

CMD = dict(APP_START=1, SEND_TXT_MSG=2, SEND_CHANNEL_TXT_MSG=3, GET_CONTACTS=4, GET_DEVICE_TIME=5,
           SET_DEVICE_TIME=6, SEND_SELF_ADVERT=7, SET_ADVERT_NAME=8, SYNC_NEXT_MESSAGE=10, SET_ADVERT_LATLON=14,
           REBOOT=19, GET_BATT_AND_STORAGE=20, DEVICE_QUERY=22, SET_OTHER_PARAMS=38)
RESP = {0: 'OK', 1: 'ERR', 2: 'CONTACTS_START', 3: 'CONTACT', 4: 'END_OF_CONTACTS', 5: 'SELF_INFO', 6: 'SENT',
        9: 'CURR_TIME', 10: 'NO_MORE_MESSAGES', 12: 'BATT_AND_STORAGE', 13: 'DEVICE_INFO', 16: 'CONTACT_MSG_RECV_V3',
        17: 'CHANNEL_MSG_RECV_V3', 0x80: 'PUSH_ADVERT', 0x81: 'PUSH_PATH_UPDATED', 0x82: 'PUSH_SEND_CONFIRMED',
        0x83: 'PUSH_MSG_WAITING', 0x8A: 'PUSH_NEW_ADVERT'}


def decode(f):
    """Décode les trames utiles aux tests (dict avec 'code' et 'name')."""
    code = f[0]
    d = {'code': code, 'name': RESP.get(code, f'0x{code:02x}'), 'raw': f}
    if code == 5 and len(f) >= 58:
        d.update(adv_type=f[1], tx_power=f[2], max_tx_power=f[3], pubkey=f[4:36])
        d['lat'], d['lon'] = (v / 1e6 for v in struct.unpack_from('<ii', f, 36))
        d['freq_khz'], d['bw_hz'] = struct.unpack_from('<II', f, 48)
        d['sf'], d['cr'] = f[56], f[57]
        d['node_name'] = f[58:].decode('utf-8', 'replace')
    elif code == 13 and len(f) >= 4:
        d.update(fw_ver=f[1], max_contacts=f[2] * 2, max_channels=f[3])
        txt = lambda b: b.split(b'\0')[0].decode('utf-8', 'replace')
        d.update(build=txt(f[8:20]), model=txt(f[20:60]), version=txt(f[60:80]))
    elif code == 12 and len(f) >= 3:
        d['battery_mv'] = struct.unpack_from('<H', f, 1)[0]
    elif code == 3 and len(f) >= 100:
        d['pubkey'] = f[1:33]
        d['contact_name'] = f[36 + 64:36 + 64 + 32].split(b'\0')[0].decode('utf-8', 'replace')
    elif code in (16, 7) and len(f) > 16:
        d['text'] = f[16:].decode('utf-8', 'replace') if code == 16 else f[13:].decode('utf-8', 'replace')
    elif code == 6 and len(f) >= 10:
        d['is_flood'] = f[1]
        d['ack'] = f[2:6]
        d['timeout_ms'] = struct.unpack_from('<I', f, 6)[0]
    return d


class Companion:
    def __init__(self, node, app_name='jumeau'):
        self.node = node
        self.phone = node.phone
        self.app_name = app_name
        self.inbox = []
        self.phone.on_notify.append(self._rx)

    def _rx(self, uuid, data):
        if uuid == NUS_TX:
            self.inbox.append(bytes(data))

    def run(self, ms_):
        self.node.run(ms_)

    def wait(self, cond, max_ms=5000, step_ms=20):
        t = 0
        while not cond():
            if t >= max_ms:
                return False
            self.run(step_ms)
            t += step_ms
        return True

    def start(self, timeout_ms=10000):
        ph = self.phone
        ok = self.wait(lambda: self.node.ble.adv_on, timeout_ms)
        if not ok:
            raise AssertionError("le T1000-A n'annonce pas en BLE")
        ph.connect()
        self.run(100)
        ph.pair()
        if not self.wait(lambda: ph.pair_status is not None, 5000):
            raise AssertionError("l'appairage n'aboutit pas")
        if ph.pair_status != 0:
            raise AssertionError(f'appairage refusé (statut {ph.pair_status:#x})')
        n = len(ph.write_results)
        err = ph.subscribe(NUS_TX)
        if err:
            raise AssertionError(f'abonnement NUS TX refusé ({err:#x})')
        self.wait(lambda: len(ph.write_results) > n, 2000)
        return self

    def send(self, frame):
        n = len(self.phone.write_results)
        err = self.phone.write(NUS_RX, frame)
        if err:
            raise AssertionError(f'écriture NUS RX refusée ({err:#x})')
        self.wait(lambda: len(self.phone.write_results) > n, 2000)

    def query(self, frame, max_ms=3000, quiet_ms=300, expect=None):
        """Envoie une trame et renvoie les trames décodées reçues (jusqu'à `expect` codes ou un silence)."""
        start = len(self.inbox)
        self.send(frame)
        t, last_n, last_t = 0, start, 0
        while t < max_ms:
            self.run(20)
            t += 20
            if len(self.inbox) != last_n:
                last_n, last_t = len(self.inbox), t
            got = [f[0] for f in self.inbox[start:]]
            if expect is not None and expect in got:
                break
            if expect is None and len(self.inbox) > start and t - last_t >= quiet_ms:
                break
        return [decode(f) for f in self.inbox[start:]]

    # commandes courantes
    def device_query(self):
        return self.query(bytes([22, 3]), expect=13)

    def app_start(self):
        return self.query(bytes([1, 3]) + bytes(6) + self.app_name.encode(), expect=5)

    def send_advert(self, flood=True):
        return self.query(bytes([7, 1 if flood else 0]), expect=0)

    def contacts(self):
        return [d for d in self.query(bytes([4]), expect=4) if d['code'] == 3]

    def battery(self):
        return self.query(bytes([20]), expect=12)

    def set_time(self, epoch):
        return self.query(bytes([6]) + struct.pack('<I', epoch), expect=0)

    def send_text(self, pubkey_prefix, text, attempt=0, ts=0):
        return self.query(bytes([2, 0, attempt]) + struct.pack('<I', ts) + pubkey_prefix[:6] + text.encode(), expect=6)

    def sync_messages(self):
        out = []
        while True:
            r = self.query(bytes([10]), expect=None, max_ms=1000)
            if not r or r[-1]['code'] == 10:
                return out
            out += [d for d in r if d['code'] in (7, 8, 16, 17)]
