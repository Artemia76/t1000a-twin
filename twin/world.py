"""Monde simulé : plusieurs jumeaux en pas synchronisés et le milieu radio LoRa qui les relie.

Chaque jumeau a sa propre horloge ; `World.run` les fait avancer par quanta (1 ms par défaut) pour que leurs
temps ne s'écartent jamais de plus d'un quantum. Un paquet émis par une radio est livré aux autres au même
instant absolu (début : air_start, fin : air_end), avec un RSSI/SNR propre à chaque lien.
"""
import random
from .core import F_CPU, ms


class Ether:
    def __init__(self, world):
        self.world = world
        self.radios = []
        self.links = {}                     # (émetteur, récepteur) -> (rssi, snr) ; absent = défaut
        self.default_link = (-75.0, 9.0)
        self.loss = 0.0                     # probabilité de perte d'un paquet sur un lien
        self.frames = []                    # journal de tout ce qui a été émis
        self.rng = random.Random(1)

    def add(self, radio):
        radio.ether = self
        self.radios.append(radio)

    def link(self, a, b, rssi, snr=9.0, both=True):
        self.links[(a, b)] = (rssi, snr)
        if both:
            self.links[(b, a)] = (rssi, snr)

    def _now(self):
        return self.world.cycles

    def transmit(self, src, data, dur_us, params=None):
        p = params or src.params()
        t0 = src.tw.now() if src is not None and hasattr(src, 'tw') else self._now()
        frame = {'data': bytes(data), 'start': t0, 'end': t0 + dur_us * 64, 'src': src, **p}
        self.frames.append(frame)
        for r in self.radios:
            if r is src:
                continue
            rssi, snr = self.links.get((src, r), self.default_link) if src is not None else \
                (p.get('rssi', -75.0), p.get('snr', 9.0))
            if rssi is None or self.rng.random() < self.loss:
                continue
            f = dict(frame, rssi=rssi, snr=snr)
            tw = r.tw
            d0 = max(0, frame['start'] - tw.now())
            tw.schedule(d0, r.air_start, f)
            tw.schedule(max(d0, frame['end'] - tw.now()), r.air_end, f)
        return frame

    def inject(self, data, rssi=-75.0, snr=9.0, **params):
        """Paquet émis par un nœud fictif (hors jumeaux), avec les paramètres radio donnés (défaut : ceux
        de la première radio)."""
        from .lr1110 import time_on_air_us, BW_KHZ
        ref = self.radios[0].params() if self.radios else {}
        p = dict(ref, **params)
        p.update(rssi=rssi, snr=snr)
        dur = time_on_air_us(len(data), p['sf'], BW_KHZ.get(p['bw'], 125.0), p['cr'], p['preamble'],
                             p['explicit'], p['crc'], bool(p.get('ldro')))
        return self.transmit(None, data, dur, p)

    def abort(self, src):
        for f in self.frames:
            if f['src'] is src and f['end'] > src.tw.now():
                f['corrupt'] = True

    def on_air(self, radio):
        t = radio.tw.now()
        return [f for f in self.frames[-50:] if f['start'] <= t < f['end'] and f['src'] is not radio]

    def rssi_at(self, radio):
        fr = self.on_air(radio)
        if fr:
            return max(self.links.get((f['src'], radio), self.default_link)[0] for f in fr)
        return -110.0 + self.rng.uniform(-2, 2)

    def channel_busy(self, radio):
        return any(abs(f['freq'] - radio.freq) < 1000 and f['sf'] == radio.sf for f in self.on_air(radio))


class World:
    def __init__(self, quantum_ms=1):
        self.nodes = []
        self.ether = Ether(self)
        self.cycles = 0
        self.quantum = ms(quantum_ms)

    def add(self, node):
        node.tw.cycles = max(node.tw.cycles, self.cycles)
        self.nodes.append(node)
        if getattr(node, 'radio', None) is not None:
            self.ether.add(node.radio)
        node.world = self
        return node

    def now_ms(self):
        return self.cycles * 1000 // F_CPU

    def run(self, ms_=None, until=None, max_ms=None):
        """Fait avancer tous les jumeaux de ms_ millisecondes (ou jusqu'à until(world), au plus max_ms)."""
        end = self.cycles + ms(ms_ if ms_ is not None else max_ms) if (ms_ is not None or max_ms is not None) else None
        while True:
            if until is not None and until(self):
                return 'condition'
            if end is not None and self.cycles >= end:
                return 'temps'
            target = self.cycles + self.quantum
            if end is not None:
                target = min(target, end)
            for n in self.nodes:
                if n.tw.cycles < target:
                    n.tw.run(until_cycles=target)
            self.cycles = target
