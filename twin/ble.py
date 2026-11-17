"""Modèle de la pile BLE du GR5515 (en ROM sur la puce) et téléphone virtuel (central).

La pile est remplacée au niveau de l'API du SDK (ble_stack_init, ble_gap_*, ble_gatts_*, ble_sec_*) :
  * événements livrés en interruption au gestionnaire passé à ble_stack_init (et, pour les attributs d'un
    profil, au rappel passé à ble_gatts_prf_add) ;
  * base GATT construite à partir des tables d'attributs du firmware (poignées attribuées comme la ROM) ;
  * permissions vérifiées : une écriture sur un attribut qui exige un lien chiffré et authentifié est
    refusée (ATT 0x05) tant que l'appairage n'a pas réussi ;
  * appairage par code (TK) : le code saisi sur le téléphone est comparé à celui que le firmware fournit ;
  * liaisons (bonding) conservées à travers les redémarrages du MCU, comme dans la NVDS ;
  * notifications : file limitée (8) vidée à chaque intervalle de connexion → contre-pression réaliste.
"""
import struct

SUCCESS = 0
SDK_ERR_NO_RESOURCES, SDK_ERR_DISCONNECTED = 0x0010, 0x001C
ATT_ERR_INSUFF_AUTHEN = 0x05
SEC_ERR_PASSKEY_ENTRY_FAIL = 0x61
UUID_CCCD = bytes.fromhex('0229') + bytes(14)          # 0x2902 en 128 bits (LSB d'abord, base BT SIG omise)
PERM_READ, PERM_WRITE_CMD, PERM_WRITE_REQ, PERM_NOTIFY, PERM_INDICATE = 0x02, 0x04, 0x08, 0x10, 0x20


def uuid16(u):
    return struct.pack('<H', u)


ADV_INTERVAL_DEFAULT_MS = 152.5     # intervalle d'annonce si ble_gap_adv_param_set() n'est pas lisible
RADIO_EVENT_MS = 3                  # durée d'un événement radio BLE (2e réveil du cœur)


class BleStack:
    def __init__(self, tw, mac=None):
        self.tw = tw
        L = tw.L
        self.E = L.enum
        r = __import__('random').Random(f'{tw.seed}-{tw.name}-mac')
        self.mac = mac or bytes([r.randrange(256) for _ in range(2)]) + bytes([0x38, 0x17, 0x28, 0xFD])
        self.bonds = {}                     # adresse du téléphone -> {'cccd': {poignée: valeur}}
        self.next_handle = 0x0010
        self.phones = []
        self._rng = r
        self._reset()
        tw.reset_listeners.append(self._on_mcu_reset)
        api = {
            'ble_stack_init': self._stack_init,
            'ble_gatts_prf_add': self._prf_add,
            'ble_gap_adv_param_set': self._adv_param_set,
            'ble_gap_adv_data_set': self._adv_data_set,
            'ble_gap_adv_start': self._adv_start,
            'ble_gap_adv_stop': self._adv_stop,
            'ble_gap_device_name_set': self._name_set,
            'ble_gap_addr_get': self._addr_get,
            'ble_gap_disconnect': self._disconnect,
            'ble_gap_disconnect_with_reason': lambda idx, r: self._disconnect(idx),
            'ble_gatts_noti_ind': self._noti_ind,
            'ble_gatts_read_cfm': self._read_cfm,
            'ble_gatts_write_cfm': self._write_cfm,
            'ble_sec_params_set': self._sec_params,
            'ble_sec_enc_cfm': self._enc_cfm,
            'ble_gap_pair_enable': lambda en: SUCCESS,
            'ble_gap_privacy_params_set': lambda t, en: SUCCESS,
            'ble_gap_conn_param_update_reply': lambda idx, ok: SUCCESS,
            'ble_gap_l2cap_params_set': lambda a, b, c: SUCCESS,
            'ble_gap_data_length_set': lambda a, b: SUCCESS,
            'ble_gap_pref_phy_set': lambda a, b: SUCCESS,
            'ble_gap_conn_param_update': lambda idx, p: SUCCESS,
        }
        for name, fn in api.items():
            tw.on(name, fn)

    def _reset(self):
        self.handler = None
        self.ready = False
        self.profiles = []
        self.attrs = {}                     # poignée -> dict(uuid, perm, ext, max, prf, idx)
        self.next_handle = 0x0010
        self.adv_on = False
        self.adv_data, self.scan_rsp = b'', b''
        self.adv_interval_ms = ADV_INTERVAL_DEFAULT_MS
        self._radio_ev = None
        self.name = b''
        self.conn = None                    # dict(phone, encrypted, mtu, cccd, pending_ntf…)
        self.sec = None

    def _on_mcu_reset(self):
        if self.conn:
            ph = self.conn['phone']
            self.conn = None
            ph._link_lost('reset du T1000-A')
        self._reset()

    # ------------------------------------------------------------------ événements
    def _evt(self, evt_id, status=0, fields=None, extra=b'', to_profile=None, to_app=True):
        L = self.tw.L
        base = L.pack('ble_evt_t', evt_id=evt_id, evt_status=status)
        buf = bytearray(base)
        for path, v in (fields or {}).items():
            if path.endswith('.value') and isinstance(v, str) and v == '@extra':
                continue
            L.put(buf, 'ble_evt_t', path, v)
        size = len(buf) + len(extra)
        ptr_path = [p for p, v in (fields or {}).items() if v == '@extra']

        def build(addr, buf=bytes(buf)):
            b = bytearray(buf)
            for p in ptr_path:
                L.put(b, 'ble_evt_t', p, addr + len(base))
            return bytes(b) + extra
        if to_profile is not None:
            self.tw.irq(f'ble prf {evt_id:#x}', to_profile, (None,), ((size, build),))
        if to_app and self.handler:
            self.tw.irq(f'ble {evt_id:#x}', self.handler, (None,), ((size, build),))

    def _later(self, delay_ms, fn, *args):
        return self.tw.schedule_ms(delay_ms, fn, *args)

    # ------------------------------------------------------------------ API du SDK
    def _stack_init(self, handler, heaps):
        self.handler = handler
        self.tw.log('ble_stack_init')
        self._later(5, self._stack_ready)
        return SUCCESS

    def _stack_ready(self):
        self.ready = True
        self._evt(self.E('BLE_COMMON_EVT_STACK_INIT'))

    def _prf_add(self, p_db, cb):
        tw, L = self.tw, self.tw.L
        d = tw.read(p_db, L.size('ble_gatts_create_db_t'))
        g = lambda f: L.get(d, 'ble_gatts_create_db_t', f)
        shdl, puuid, pcfg, nb, ptab, typ = g('shdl'), g('uuid'), g('attr_tab_cfg'), g('max_nb_attr'), \
            g('attr_tab'), g('attr_tab_type')
        is128 = typ == self.E('BLE_GATTS_SERVICE_TABLE_TYPE_128') if 'BLE_GATTS_SERVICE_TABLE_TYPE_128' in L.enums else typ != 0
        dname = 'ble_gatts_attm_desc_128_t' if is128 else 'ble_gatts_attm_desc_t'
        dsz = L.size(dname)
        cfg = tw.read(pcfg, (nb + 7) // 8) if pcfg else b'\xff' * ((nb + 7) // 8)
        start = tw.r32(shdl) & 0xFFFF if shdl else 0
        start = start or self.next_handle
        prf = {'cb': cb, 'start': start, 'uuid': tw.read(puuid, 16 if is128 else 2), 'handles': {}}
        h = start
        for i in range(nb):
            if not cfg[i // 8] >> (i % 8) & 1:
                continue
            a = tw.read(ptab + i * dsz, dsz)
            uuid = a[:16] if is128 else a[:2]
            attr = {'uuid': bytes(uuid), 'perm': L.get(a, dname, 'perm'), 'ext': L.get(a, dname, 'ext_perm'),
                    'max': L.get(a, dname, 'max_size'), 'prf': prf, 'idx': i, 'value': b''}
            self.attrs[h] = attr
            prf['handles'][i] = h
            h += 1
        self.next_handle = h
        if shdl:
            tw.write(shdl, struct.pack('<H', start))
        self.profiles.append(prf)
        tw.log('ble_prf_add', uuid=prf['uuid'][::-1].hex(), start=start, n=h - start)
        return SUCCESS

    def _adv_data_set(self, idx, typ, data, n):
        b = self.tw.read(data, n)
        if typ == self.E('BLE_GAP_ADV_DATA_TYPE_SCAN_RSP'):
            self.scan_rsp = b
        else:
            self.adv_data = b
        return SUCCESS

    def _adv_param_set(self, idx, own, p):
        # Intervalle effectif mesuré sur le boîtier = adv_intv_max (+ advDelay 0–10 ms de la norme)
        try:
            L = self.tw.L
            d = self.tw.read(p, L.size('ble_gap_adv_param_t'))
            self.adv_interval_ms = L.get(d, 'ble_gap_adv_param_t', 'adv_intv_max') * 0.625
        except Exception:
            self.adv_interval_ms = ADV_INTERVAL_DEFAULT_MS
        return SUCCESS

    # ------------------------------------------------------------------ réveils du contrôleur BLE
    # Chaque événement radio (annonce sur 3 canaux, ou événement de connexion) sort le cœur de veille deux
    # fois : au début, puis à la fin (≈ RADIO_EVENT_MS plus tard). Mesuré le 17/11/2026 (sonde « sched ») :
    # sans connexion ≈ 6 longues veilles/s (≈ 150 ms) + autant de courtes (< 10 ms) ; connecté, une veille par
    # intervalle de connexion + une courte.
    def _radio_schedule(self):
        self.tw.cancel(self._radio_ev)
        self._radio_ev = None
        if self.conn is not None:
            period = self.conn['interval_ms']
        elif self.adv_on:
            period = self.adv_interval_ms + self._rng.uniform(0, 10)      # advDelay
        else:
            return
        self._radio_ev = self._later(period, self._radio_event)

    def _radio_event(self):
        self._radio_ev = None
        self.tw.radio_wake()
        self._later(RADIO_EVENT_MS, self.tw.radio_wake)
        self._radio_schedule()

    def _adv_start(self, idx, ptime):
        if not self.ready:
            return 0x0009                                  # SDK_ERR_INVALID_STATE (pile pas prête)
        if self.conn:
            return 0x0009
        self.adv_on = True
        self._radio_schedule()
        self.tw.log('ble_adv_start', name=self.adv_name())
        self._later(1, self._evt, self.E('BLE_GAPM_EVT_ADV_START'))
        return SUCCESS

    def _adv_stop(self, idx):
        if self.adv_on:
            self.adv_on = False
            self._radio_schedule()
            self._later(1, self._evt, self.E('BLE_GAPM_EVT_ADV_STOP'), 0,
                        {'evt.gapm_evt.params.adv_stop.reason': self.E('BLE_GAP_STOPPED_REASON_ON_USER')})
        return SUCCESS

    def _name_set(self, perm, p, n):
        self.name = self.tw.read(p, n)
        return SUCCESS

    def _addr_get(self, p):
        self.tw.write(p, self.mac + b'\x01')
        return SUCCESS

    def _disconnect(self, idx):
        if self.conn is None:
            return SDK_ERR_DISCONNECTED
        ph = self.conn['phone']
        self.conn = None
        self._radio_schedule()
        self._later(5, self._evt, self.E('BLE_GAPC_EVT_DISCONNECTED'), 0,
                    {'evt.gapc_evt.index': 0, 'evt.gapc_evt.params.disconnected': 0x16})
        ph._link_lost('déconnecté par le T1000-A')
        return SUCCESS

    def _noti_ind(self, idx, p):
        L, tw = self.tw.L, self.tw
        if self.conn is None:
            return SDK_ERR_DISCONNECTED
        d = tw.read(p, L.size('ble_gatts_noti_ind_t'))
        g = lambda f: L.get(d, 'ble_gatts_noti_ind_t', f)
        handle, n = g('handle'), g('length')
        data = tw.read(g('value'), n) if n else b''
        c = self.conn
        if len(c['ntf_q']) >= 8:
            return SDK_ERR_NO_RESOURCES
        if n > c['mtu'] - 3:
            tw.log('ble_ntf_too_long', len=n, mtu=c['mtu'])
            return 0x0003                                   # SDK_ERR_INVALID_PARAM
        c['ntf_q'].append((handle, data))
        if c['ntf_ev'] is None:
            c['ntf_ev'] = self._later(c['interval_ms'], self._flush_ntf)
        return SUCCESS

    def _flush_ntf(self):
        c = self.conn
        if c is None:
            return
        c['ntf_ev'] = None
        q, c['ntf_q'] = c['ntf_q'], []
        for handle, data in q:
            attr = self.attrs.get(handle, {})
            c['phone']._notified(handle, attr.get('uuid'), data)
            self._evt(self.E('BLE_GATTS_EVT_NTF_IND'), 0, {'evt.gatts_evt.index': 0},
                      to_profile=attr.get('prf', {}).get('cb'))

    def _read_cfm(self, idx, p):
        L, tw = self.tw.L, self.tw
        d = tw.read(p, L.size('ble_gatts_read_cfm_t'))
        g = lambda f: L.get(d, 'ble_gatts_read_cfm_t', f)
        n = g('length')
        val = tw.read(g('value'), n) if n else b''
        if self.conn:
            self.conn['phone']._read_done(g('handle'), g('status'), val)
        return SUCCESS

    def _write_cfm(self, idx, p):
        L, tw = self.tw.L, self.tw
        d = tw.read(p, L.size('ble_gatts_write_cfm_t'))
        if self.conn:
            self.conn['phone']._write_done(L.get(d, 'ble_gatts_write_cfm_t', 'handle'),
                                           L.get(d, 'ble_gatts_write_cfm_t', 'status'))
        return SUCCESS

    def _sec_params(self, p):
        self.sec = self.tw.read(p, 16)
        return SUCCESS

    def _enc_cfm(self, idx, p):
        L, tw = self.tw.L, self.tw
        d = tw.read(p, L.size('ble_sec_cfm_enc_t'))
        rt, accept = L.get(d, 'ble_sec_cfm_enc_t', 'req_type'), L.get(d, 'ble_sec_cfm_enc_t', 'accept')
        c = self.conn
        if c is None:
            return SDK_ERR_DISCONNECTED
        ph = c['phone']
        if not accept:
            self._later(10, self._encrypted, 0x68)
        elif rt == self.E('BLE_SEC_PAIR_REQ'):
            self._later(20, self._evt, self.E('BLE_SEC_EVT_LINK_ENC_REQUEST'), 0,
                        {'evt.sec_evt.index': 0, 'evt.sec_evt.params.enc_req.req_type': self.E('BLE_SEC_TK_REQ'),
                         'evt.sec_evt.params.enc_req.data.tk_type': self.E('BLE_SEC_TK_DISPLAY')})
        elif rt == self.E('BLE_SEC_TK_REQ'):
            tk = struct.unpack_from('<I', d, L.off('ble_sec_cfm_enc_t', 'data.tk.key'))[0]
            tw.log('ble_passkey', firmware=tk, phone=ph.passkey)
            self._later(30, self._encrypted, SUCCESS if tk == ph.passkey else SEC_ERR_PASSKEY_ENTRY_FAIL)
        return SUCCESS

    def _encrypted(self, status):
        c = self.conn
        if c is None:
            return
        ph = c['phone']
        if status == SUCCESS:
            c['encrypted'] = True
            self.bonds.setdefault(ph.addr, {'cccd': {}})
        self._evt(self.E('BLE_SEC_EVT_LINK_ENCRYPTED'), status, {'evt.sec_evt.index': 0})
        ph._paired(status)
        if status == SUCCESS:
            for h, v in self.bonds[ph.addr]['cccd'].items():
                attr = self.attrs.get(h)
                if attr:
                    c['cccd'][h] = v
                    self._evt(self.E('BLE_GATTS_EVT_CCCD_RECOVERY'), 0,
                              {'evt.gatts_evt.index': 0, 'evt.gatts_evt.params.cccd_recovery.handle': h,
                               'evt.gatts_evt.params.cccd_recovery.cccd_val': v},
                              to_profile=attr['prf']['cb'], to_app=False)

    def adv_name(self):
        for d in (self.scan_rsp, self.adv_data):
            i = 0
            while i + 1 < len(d):
                n, t = d[i], d[i + 1]
                if t in (0x08, 0x09):
                    return d[i + 2:i + 1 + n].decode('utf-8', 'replace')
                i += n + 1
        return self.name.decode('utf-8', 'replace')

    # ------------------------------------------------------------------ côté téléphone
    def _connect(self, phone):
        if not self.adv_on:
            raise RuntimeError("le T1000-A n'annonce pas : connexion impossible")
        self.adv_on = False
        self.conn = {'phone': phone, 'encrypted': False, 'mtu': 23, 'cccd': {}, 'ntf_q': [], 'ntf_ev': None,
                     'interval_ms': phone.interval_ms, 'pending': {}}
        self._radio_schedule()
        E = self.E
        self._evt(E('BLE_GAPM_EVT_ADV_STOP'), 0, {'evt.gapm_evt.params.adv_stop.reason': E('BLE_GAP_STOPPED_REASON_CONN_EST')})
        self._evt(E('BLE_GAPC_EVT_CONNECTED'), 0, {
            'evt.gapc_evt.index': 0, 'evt.gapc_evt.params.connected.conn_handle': 0,
            'evt.gapc_evt.params.connected.conn_interval': int(phone.interval_ms / 1.25),
            'evt.gapc_evt.params.connected.sup_timeout': 400, 'evt.gapc_evt.params.connected.peer_addr_type': 1,
            'evt.gapc_evt.params.connected.peer_addr.addr': phone.addr})
        self._later(30, self._mtu, phone.mtu)
        if phone.addr in self.bonds and phone.auto_encrypt:
            self._later(60, self._encrypted, SUCCESS)

    def _mtu(self, mtu):
        if self.conn:
            self.conn['mtu'] = mtu
            self._evt(self.E('BLE_GATT_COMMON_EVT_MTU_EXCHANGE'), 0,
                      {'evt.gatt_common_evt.index': 0, 'evt.gatt_common_evt.params.mtu_exchange.mtu': mtu})

    def _phone_disconnect(self, reason=0x13):
        if self.conn is None:
            return
        c, self.conn = self.conn, None
        self.tw.cancel(c['ntf_ev'])
        self._radio_schedule()
        self._evt(self.E('BLE_GAPC_EVT_DISCONNECTED'), 0,
                  {'evt.gapc_evt.index': 0, 'evt.gapc_evt.params.disconnected': reason})

    def _pair_request(self):
        self._evt(self.E('BLE_SEC_EVT_LINK_ENC_REQUEST'), 0,
                  {'evt.sec_evt.index': 0, 'evt.sec_evt.params.enc_req.req_type': self.E('BLE_SEC_PAIR_REQ')})

    def _allowed(self, attr, kind):
        perm = attr['perm']
        props = perm >> 8
        if kind == 'write':
            if not props & (PERM_WRITE_REQ | PERM_WRITE_CMD):
                return 0x03                                   # écriture non permise
            lvl = (perm >> 2) & 3
        elif kind == 'read':
            if not props & PERM_READ:
                return 0x02
            lvl = perm & 3
        else:
            lvl = (perm >> 6) & 3
        if lvl >= 1 and not self.conn['encrypted']:
            return ATT_ERR_INSUFF_AUTHEN
        return 0

    def _phone_write(self, handle, data, response=True):
        c = self.conn
        attr = self.attrs.get(handle)
        if c is None or attr is None:
            return 0x01
        if attr['uuid'][:2] == b'\x02\x29' and len(attr['uuid']) in (2, 16):      # CCCD
            if not self.conn['encrypted'] and (self.attrs.get(handle - 1, {}).get('perm', 0) >> 6) & 3:
                return ATT_ERR_INSUFF_AUTHEN
        err = self._allowed(attr, 'write')
        if err:
            return err
        if attr['uuid'][:2] == b'\x02\x29':
            v = int.from_bytes(data[:2], 'little')
            c['cccd'][handle] = v
            if c['encrypted']:
                self.bonds.setdefault(c['phone'].addr, {'cccd': {}})['cccd'][handle] = v
        self._evt(self.E('BLE_GATTS_EVT_WRITE_REQUEST'), 0, {
            'evt.gatts_evt.index': 0, 'evt.gatts_evt.params.write_req.handle': handle,
            'evt.gatts_evt.params.write_req.offset': 0, 'evt.gatts_evt.params.write_req.length': len(data),
            'evt.gatts_evt.params.write_req.value': '@extra'}, extra=bytes(data), to_profile=attr['prf']['cb'],
            to_app=False)
        return None                                           # réponse = write_cfm du firmware

    def _phone_read(self, handle):
        c = self.conn
        attr = self.attrs.get(handle)
        if c is None or attr is None:
            return 0x01
        err = self._allowed(attr, 'read')
        if err:
            return err
        self._evt(self.E('BLE_GATTS_EVT_READ_REQUEST'), 0, {
            'evt.gatts_evt.index': 0, 'evt.gatts_evt.params.read_req.handle': handle},
            to_profile=attr['prf']['cb'], to_app=False)
        return None

    def find(self, uuid_bytes):
        """Poignée de la valeur d'une caractéristique d'UUID donné (octets LSB d'abord)."""
        for h, a in sorted(self.attrs.items()):
            if a['uuid'] == uuid_bytes:
                return h
        return None


class Phone:
    """Téléphone virtuel : central BLE qui se connecte, s'appaire et échange avec les services GATT."""

    def __init__(self, ble, passkey=123456, addr=None, mtu=247, interval_ms=15, auto_encrypt=True, name='téléphone'):
        self.ble, self.tw = ble, ble.tw
        self.passkey = passkey
        self.addr = addr or bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0xC6])
        self.mtu, self.interval_ms, self.auto_encrypt = mtu, interval_ms, auto_encrypt
        self.name = name
        self.connected = False
        self.encrypted = False
        self.pair_status = None
        self.notifications = []             # (cycles, poignée, uuid, octets)
        self.on_notify = []                 # rappels fn(uuid, octets)
        self.write_results = []             # (poignée, statut)
        self.read_results = []
        self.events = []
        ble.phones.append(self)

    def scan(self):
        return {'name': self.ble.adv_name(), 'mac': self.ble.mac[::-1].hex(':').upper(),
                'adv': self.ble.adv_data, 'scan_rsp': self.ble.scan_rsp} if self.ble.adv_on else None

    def connect(self):
        self.ble._connect(self)
        self.connected = True
        self.events.append((self.tw.now(), 'connected'))

    def disconnect(self):
        self.ble._phone_disconnect()
        self.connected = self.encrypted = False

    def pair(self):
        self.ble._pair_request()

    def write(self, uuid_or_handle, data, response=True):
        h = uuid_or_handle if isinstance(uuid_or_handle, int) else self.ble.find(uuid_or_handle)
        if h is None:
            raise KeyError(f'caractéristique {uuid_or_handle!r} absente de la base GATT')
        err = self.ble._phone_write(h, bytes(data), response)
        if err is not None:
            self.write_results.append((h, err))
        return err

    def subscribe(self, uuid_bytes):
        h = self.ble.find(uuid_bytes)
        return self.write(h + 1, b'\x01\x00')

    def read(self, uuid_or_handle):
        h = uuid_or_handle if isinstance(uuid_or_handle, int) else self.ble.find(uuid_or_handle)
        err = self.ble._phone_read(h)
        if err is not None:
            self.read_results.append((h, err, b''))
        return err

    # rappels de la pile
    def _notified(self, handle, uuid, data):
        self.notifications.append((self.tw.now(), handle, uuid, data))
        for f in self.on_notify:
            f(uuid, data)

    def _write_done(self, handle, status):
        self.write_results.append((handle, status))

    def _read_done(self, handle, status, val):
        self.read_results.append((handle, status, val))

    def _paired(self, status):
        self.pair_status = status
        self.encrypted = status == SUCCESS

    def _link_lost(self, why):
        self.connected = self.encrypted = False
        self.events.append((self.tw.now(), 'link_lost', why))
