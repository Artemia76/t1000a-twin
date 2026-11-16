"""Modèle du récepteur GNSS Airoha AG3335 du T1000-A (UART1, 115 200 bauds, NMEA).

Le récepteur n'émet que s'il est alimenté (GPS_EN = GPIO28 haut) et hors reset (GPS_RESET = GPIO16 haut,
actif bas sur le T1000-A). Après la mise sous tension il démarre en ~300 ms, puis émet chaque seconde
GGA, GSA (si fix), GSV puis RMC selon le scénario :
    gnss.no_fix()                      : pas de position ; satellites captés selon `sky` (inchangé)
    gnss.no_sky()                      : en intérieur, aucun satellite capté (GSV sans SNR)
    gnss.weak_sky(n)                   : n satellites captés (SNR), pas de position
    gnss.fix(lat, lon, alt=50, sats=9) : position fixe, `sats` satellites captés
    gnss.track(fn)                     : fn(secondes) -> (lat, lon, alt[, sats]) ou None
GSV (satellites en vue, 4 par trame, SNR vide s'ils ne sont pas captés) suit l'ordre de sortie de l'AG3335
(GGA, GLL, GSA, GSV, RMC, VTG : la fin d'une époque est RMC). Son débit se règle par $PAIR062,3,<n>
(manuel LOCOSYS AG3335M « Product Command List » : défaut 1 ; `gsv_rate` = 0 simule un réglage enregistré qui
la coupe). Les trames $PAIR reçues sont journalisées et acquittées ($PAIR001,<cmd>,0).
"""
import datetime

EN, RESET = 28, 16


def nmea(body):
    cs = 0
    for c in body.encode():
        cs ^= c
    return f'${body}*{cs:02X}\r\n'.encode()


def _deg(v, lat):
    a = abs(v)
    d = int(a)
    m = (a - d) * 60
    if lat:
        return f'{d:02d}{m:07.4f}', 'N' if v >= 0 else 'S'
    return f'{d:03d}{m:07.4f}', 'E' if v >= 0 else 'W'


class AG3335:
    def __init__(self, tw, gpio, uart, uart_id=1, en=EN, reset=RESET,
                 utc0=datetime.datetime(2026, 11, 13, 12, 0, 0, tzinfo=datetime.timezone.utc)):
        self.tw, self.gpio, self.uart, self.uid = tw, gpio, uart, uart_id
        self.en, self.rst = en, reset
        self.utc0 = utc0
        self.position = None
        self.sky = 0                        # satellites captés (SNR) sans position
        self.in_view = 12                   # satellites en vue d'après l'almanach (GSV, SNR vide si non captés)
        self.gsv_rate = 1                   # PAIR062,3 : 0 = GSV coupée
        self.ttff_s = 0                     # délai avant le premier fix après la mise sous tension
        self.running = False
        self.t_on = None
        self.ev = None
        self.sentences = 0
        self.received = bytearray()
        self.commands = []
        gpio.watch(en, lambda lv: self._update())
        gpio.watch(reset, lambda lv: self._update())
        uart.attach(uart_id, self._rx)

    # scénarios
    def no_fix(self):
        self.position = None

    def no_sky(self):
        self.position, self.sky = None, 0

    def weak_sky(self, n=4):
        self.position, self.sky = None, n

    def fix(self, lat, lon, alt=50.0, sats=9):
        self.position = lambda t: (lat, lon, alt, sats)

    def track(self, fn):
        self.position = fn

    # alimentation
    def powered(self):
        return self.gpio.level(self.en) == 1 and self.gpio.level(self.rst) == 1

    def _update(self):
        on = self.powered()
        if on and not self.running:
            self.running = True
            self.t_on = self.tw.now()
            self.tw.log('gnss_on')
            self.ev = self.tw.schedule_ms(300, self._tick)
        elif not on and self.running:
            self.running = False
            self.tw.cancel(self.ev)
            self.ev = None
            self.tw.log('gnss_off')

    def utc(self):
        return self.utc0 + datetime.timedelta(microseconds=self.tw.now_us())

    def _tick(self):
        if not self.running:
            return
        self.ev = self.tw.schedule_ms(1000, self._tick)
        t = self.utc()
        hms = t.strftime('%H%M%S') + '.000'
        dmy = t.strftime('%d%m%y')
        up = (self.tw.now() - self.t_on) / 64e6
        pos = self.position(up) if callable(self.position) and up >= self.ttff_s else None
        if pos:
            lat, lon, alt = pos[:3]
            sats = pos[3] if len(pos) > 3 else 9
            la, ns = _deg(lat, True)
            lo, ew = _deg(lon, False)
            out = nmea(f'GNGGA,{hms},{la},{ns},{lo},{ew},1,{sats:02d},0.9,{alt:.1f},M,47.0,M,,')
            out += nmea('GNGSA,A,3,01,02,03,04,05,06,07,08,09,,,,1.5,0.9,1.2,1')
            out += self._gsv(sats)
            out += nmea(f'GNRMC,{hms},A,{la},{ns},{lo},{ew},0.00,0.00,{dmy},,,A,V')
        else:
            out = nmea(f'GNGGA,{hms},,,,,0,00,99.9,,,,,,')
            out += self._gsv(self.sky)
            out += nmea(f'GNRMC,{hms},V,,,,,,,{dmy},,,N,V')
        self.sentences += out.count(b'$')
        self.uart.feed(self.uid, out)

    def _gsv(self, tracked):
        """GPGSV : `in_view` satellites, les `tracked` premiers avec un SNR, 4 par trame, identifiant de signal."""
        if not self.gsv_rate:
            return b''
        n = max(self.in_view, tracked)
        msgs = (n + 3) // 4
        out = b''
        for m in range(msgs):
            body = f'GPGSV,{msgs},{m + 1},{n:02d}'
            for i in range(m * 4, min(n, m * 4 + 4)):
                snr = f'{38 - i:02d}' if i < tracked else ''
                body += f',{i + 1:02d},{20 + 3 * i:02d},{(40 * i) % 360:03d},{snr}'
            out += nmea(body + ',1')
        return out

    def _rx(self, data):
        if not self.running:
            return
        self.received += data
        while b'\n' in self.received:
            line, _, rest = bytes(self.received).partition(b'\n')
            self.received = bytearray(rest)
            s = line.strip().decode('ascii', 'replace')
            self.commands.append(s)
            self.tw.log('gnss_cmd', line=s)
            if s.startswith('$PAIR'):
                cmd = s[5:].split(',')[0].split('*')[0]
                args = s.split('*')[0].split(',')[1:]
                if cmd == '062' and len(args) == 2 and args[0] == '3':
                    self.gsv_rate = int(args[1])
                self.tw.schedule_ms(5, self.uart.feed, self.uid, nmea(f'PAIR001,{cmd},0'))
