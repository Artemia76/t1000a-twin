"""Télémétrie du compagnon (CayenneLPP, requête « self », commande 39) : ce que publie le T1000-A.

- tension de la batterie (canal 1) et son pourcentage (courbe Li-ion, canal 1) ;
- température de la puce GR5515 (capteur interne de l'ADC, canal 1) ;
- chargeur (canal 2) : câble branché (entrée numérique), charge en cours (interrupteur) ;
- position GNSS (canal 1) quand elle est connue.
"""
import struct
import pytest
from helpers import plug, unplug
from twin import T1000A
from twin.companion import Companion

CMD_SEND_TELEMETRY_REQ, PUSH_CODE_TELEMETRY_RESPONSE = 39, 0x8B
# type LPP -> (taille, décodeur)
LPP = {
    0: (1, lambda b: b[0]),                                        # entrée numérique
    103: (2, lambda b: struct.unpack('>h', b)[0] / 10),            # température, 0,1 °C
    116: (2, lambda b: struct.unpack('>H', b)[0] / 100),           # tension, 0,01 V
    120: (1, lambda b: b[0]),                                      # pourcentage
    136: (9, lambda b: b),                                         # GPS
    142: (1, lambda b: b[0]),                                      # interrupteur
}


def parse_lpp(data):
    out, i = {}, 0
    while i + 2 <= len(data):
        ch, typ = data[i], data[i + 1]
        size, dec = LPP[typ]
        out[(ch, typ)] = dec(bytes(data[i + 2:i + 2 + size]))
        i += 2 + size
    assert i == len(data), data.hex()
    return out


@pytest.fixture(scope='module')
def node(meshcore_elf):
    n = T1000A(meshcore_elf, name='telemetrie')
    n.run(4000)
    n.comp = Companion(n).start()
    return n


def telemetry(node):
    frames = node.comp.query(bytes([CMD_SEND_TELEMETRY_REQ, 0, 0, 0]), expect=PUSH_CODE_TELEMETRY_RESPONSE)
    f = [d['raw'] for d in frames if d['code'] == PUSH_CODE_TELEMETRY_RESPONSE]
    assert f, frames
    return parse_lpp(f[0][8:])                  # code, réservé, préfixe de clé publique (6)


def test_batterie_tension_et_pourcentage(node):
    unplug(node)
    node.battery_mv = 3800                      # 60 % sur la courbe à vide (50 % = 3720 mV)
    node.run(1500)
    t = telemetry(node)
    assert abs(t[(1, 116)] - 3.80) <= 0.02, t
    assert 58 <= t[(1, 120)] <= 62, t
    node.battery_mv = 4195
    assert telemetry(node)[(1, 120)] == 100


def test_temperature_puce(node):
    node.adc.die_temp_c = 31.0
    t = telemetry(node)
    assert abs(t[(1, 103)] - 31.0) <= 0.3, t
    node.adc.die_temp_c = -5.0
    assert abs(telemetry(node)[(1, 103)] + 5.0) <= 0.3


def test_chargeur(node):
    unplug(node)
    node.run(1500)
    t = telemetry(node)
    assert (t[(2, 0)], t[(2, 142)]) == (0, 0), t
    plug(node, charging=True)
    node.run(1500)
    t = telemetry(node)
    assert (t[(2, 0)], t[(2, 142)]) == (1, 1), t
    plug(node, charging=False)
    node.run(1500)
    t = telemetry(node)
    assert (t[(2, 0)], t[(2, 142)]) == (1, 0), t
    unplug(node)
