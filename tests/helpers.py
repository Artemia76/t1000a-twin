"""Aides partagées par les tests du jumeau : console BLE (service GUS), câble de charge, heure de référence."""

NOW = 1794571200            # 13/11/2026 12:00 UTC, heure posée par le téléphone virtuel

# Chargeur : câble sur MSIO0 (≈ 2,5 V branché, ≈ 35 mV débranché), état de charge sur AON4 (bas = en charge)
CABLE, CHG_STAT = 0, 0x24
LED_PWM = 1                 # PWM1 (AON3 = PWM1_A)


def plug(node, charging=True):
    node.adc.sources[CABLE] = lambda: 2490
    node.gpio.drive(CHG_STAT, 0 if charging else 1)


def unplug(node):
    node.adc.sources[CABLE] = lambda: 35
    node.gpio.drive(CHG_STAT, 1)


_BASE = [0x1B, 0xD7, 0x90, 0xEC, 0xE8, 0xB9, 0x75, 0x80, 0x0A, 0x46, 0x44, 0xD3]
GUS_TX = bytes(_BASE + [0x02, 0x02, 0xED, 0xA6])        # notifications du T1000-A
GUS_RX = bytes(_BASE + [0x03, 0x02, 0xED, 0xA6])        # écritures du téléphone


class Console:
    def __init__(self, node):
        self.node, self.ph = node, node.phone
        self.out = bytearray()
        self.ph.on_notify.append(lambda u, d: self.out.extend(d) if u == GUS_TX else None)

    def open(self):
        n = self.node
        n.run(3000)
        assert n.ble.find(GUS_RX), 'service GUS absent de la base GATT'
        self.ph.connect()
        n.run(200)
        assert self.ph.subscribe(GUS_TX) in (None, 0)
        n.run(200)
        return self

    def send(self, line, wait_ms=500):
        k = len(self.out)
        err = self.ph.write(GUS_RX, line.encode() + b'\r\n')
        assert err in (None, 0), f'écriture console refusée ({err:#x})'
        self.node.run(wait_ms)
        return self.out[k:].decode('utf-8', 'replace')
