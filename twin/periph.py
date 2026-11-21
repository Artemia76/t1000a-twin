"""Modèles des périphériques du GR5515 au niveau des pilotes app_* du SDK.

Broches : numérotation du cœur Arduino — 0..31 GPIO, 0x20.. AON, 0x30.. MSIO.
Chaque modèle s'attache au jumeau (Twin.on) et réagit aux appels du firmware ; les composants externes
(LR1110, GNSS…) se branchent sur les broches (Gpio.watch / Gpio.drive), le SPI (Spi.attach) et l'UART
(Uart.attach / Uart.feed).
"""
import struct
from collections import defaultdict
from .core import F_CPU, ms, Reset

OK = 0


class Gpio:
    def __init__(self, tw):
        self.tw = tw
        L = tw.L
        E = L.enum
        self.T_GPIOA, self.T_GPIOB = E('APP_IO_TYPE_GPIOA'), E('APP_IO_TYPE_GPIOB')
        self.T_AON, self.T_MSIO, self.T_NORMAL = E('APP_IO_TYPE_AON'), E('APP_IO_TYPE_MSIO'), E('APP_IO_TYPE_NORMAL')
        self.M_IN, self.M_OUT, self.M_MUX = E('APP_IO_MODE_INPUT'), E('APP_IO_MODE_OUTPUT'), E('APP_IO_MODE_MUX')
        self.M_RISE, self.M_FALL = E('APP_IO_MODE_IT_RISING'), E('APP_IO_MODE_IT_FALLING')
        self.M_HIGH, self.M_LOW = E('APP_IO_MODE_IT_HIGH'), E('APP_IO_MODE_IT_LOW')
        self.P_UP, self.P_DOWN = E('APP_IO_PULLUP'), E('APP_IO_PULLDOWN')
        self.watchers = defaultdict(list)           # broche -> [fn(niveau)] sur changement de niveau
        self.ext = {}                               # broche -> niveau imposé de l'extérieur
        self.ext_default = {}                       # niveaux externes à rétablir après un reset (câblage)
        self._reset()
        tw.reset_listeners.append(self._on_mcu_reset)
        for name, fn in (('app_io_init', self._init), ('app_io_deinit', self._deinit),
                         ('app_io_write_pin', self._write), ('app_io_read_pin', self._read),
                         ('app_io_toggle_pin', self._toggle),
                         ('app_io_event_register_cb', self._register), ('app_io_event_unregister', self._unregister)):
            tw.on(name, fn, pure=(name == 'app_io_read_pin'))

    def _reset(self):
        self.cfg = {}                               # broche -> dict(mode, pull)
        self.out = {}                               # broche -> niveau de sortie
        self.irq_cb = {}                            # broche -> (cb, arg, mode, type)

    def _on_mcu_reset(self):
        before = {p: self.level(p) for p in set(self.out) | set(self.watchers)}
        self._reset()
        for p, old in before.items():
            new = self.level(p)
            if new != old:
                for f in self.watchers[p]:
                    f(new)

    # ------------------------------------------------------------------ broches
    def pins(self, typ, mask):
        base = {self.T_AON: 0x20, self.T_MSIO: 0x30, self.T_GPIOB: 16}.get(typ, 0)
        return [base + b for b in range(32) if mask >> b & 1]

    def level(self, pin):
        if pin in self.out:
            return self.out[pin]
        if pin in self.ext:
            return self.ext[pin]
        pull = self.cfg.get(pin, {}).get('pull')
        return 1 if pull == self.P_UP else 0

    def is_output(self, pin):
        return pin in self.out

    def watch(self, pin, fn):
        self.watchers[pin].append(fn)

    def drive(self, pin, level):
        """Niveau imposé par un composant externe (None = relâché)."""
        old = self.level(pin)
        if level is None:
            self.ext.pop(pin, None)
        else:
            self.ext[pin] = int(bool(level))
        self._changed(pin, old)

    def _changed(self, pin, old):
        new = self.level(pin)
        if new == old:
            return
        for f in self.watchers[pin]:
            f(new)
        h = self.irq_cb.get(pin)
        if h:
            cb, arg, mode, typ = h
            if (mode == self.M_RISE and new) or (mode == self.M_FALL and not new) or \
               (mode == self.M_HIGH and new) or (mode == self.M_LOW and not new):
                self._fire(pin, typ)

    def _fire(self, pin, typ):
        cb, arg, mode, _ = self.irq_cb[pin]
        base = {self.T_AON: 0x20, self.T_MSIO: 0x30, self.T_GPIOB: 16}.get(typ, 0)
        evt = self.tw.L.pack('app_io_evt_t', type=typ, pin=1 << (pin - base), arg=arg)
        self.tw.log('io_irq', pin=pin)
        self.tw.irq(f'io {pin:#x}', cb, (None,), (evt,))

    # ------------------------------------------------------------------ pilotes
    def _init(self, typ, p_init):
        L = self.tw.L
        d = self.tw.read(p_init, L.size('app_io_init_t'))
        mask, mode, pull = (L.get(d, 'app_io_init_t', f) for f in ('pin', 'mode', 'pull'))
        for p in self.pins(typ, mask):
            old = self.level(p)
            self.cfg[p] = {'mode': mode, 'pull': pull}
            if mode == self.M_OUT:
                self.out.setdefault(p, 0)
            else:
                self.out.pop(p, None)
            self._changed(p, old)
        return OK

    def _deinit(self, typ, mask):
        for p in self.pins(typ, mask):
            old = self.level(p)
            self.cfg.pop(p, None)
            self.out.pop(p, None)
            self.irq_cb.pop(p, None)
            self._changed(p, old)
        return OK

    def _write(self, typ, mask, state):
        for p in self.pins(typ, mask):
            old = self.level(p)
            self.out[p] = 1 if state else 0
            self.tw.log('pin', pin=p, level=self.out[p])
            self._changed(p, old)
        return OK

    def _toggle(self, typ, mask):
        for p in self.pins(typ, mask):
            old = self.level(p)
            self.out[p] = 0 if old else 1
            self._changed(p, old)
        return OK

    def _read(self, typ, mask):
        ps = self.pins(typ, mask)
        return self.level(ps[0]) if ps else 0

    def _register(self, typ, p_init, cb, arg):
        L = self.tw.L
        d = self.tw.read(p_init, L.size('app_io_init_t'))
        mask, mode, pull = (L.get(d, 'app_io_init_t', f) for f in ('pin', 'mode', 'pull'))
        for p in self.pins(typ, mask):
            old = self.level(p)
            self.cfg[p] = {'mode': mode, 'pull': pull}
            self.out.pop(p, None)
            self.irq_cb[p] = (cb, arg, mode, typ)
            self._changed(p, old)
            lv = self.level(p)
            if (mode == self.M_HIGH and lv) or (mode == self.M_LOW and not lv):
                self._fire(p, typ)
        return OK

    def _unregister(self, typ, mask):
        for p in self.pins(typ, mask):
            self.irq_cb.pop(p, None)
        return OK


class Timers:
    """app_timer : minuteurs logiciels du SDK (rappel appelé en interruption)."""

    def __init__(self, tw):
        self.tw = tw
        self.timers = {}                    # p_id -> dict(mode, cb, ev)
        self.repeat = tw.L.enum('ATIMER_REPEAT')
        tw.reset_listeners.append(self.timers.clear)
        tw.on('app_timer_create', self._create)
        tw.on('app_timer_start_api', self._start)
        tw.on('app_timer_stop_api', self._stop)
        tw.on('app_timer_delete', self._delete)

    def _create(self, p_id, mode, cb):
        self.timers[p_id] = {'mode': mode, 'cb': cb, 'ev': None}
        return OK

    def _start(self, p_id, delay, ctx):
        t = self.timers.get(p_id)
        if t is None:
            return 1
        self.tw.cancel(t['ev'])
        t['delay'], t['ctx'] = max(1, delay), ctx
        t['ev'] = self.tw.schedule_ms(t['delay'], self._fire, p_id)
        return OK

    def _fire(self, p_id):
        t = self.timers.get(p_id)
        if t is None:
            return
        self.tw.irq('app_timer', t['cb'], (t['ctx'],))
        t['ev'] = self.tw.schedule_ms(t['delay'], self._fire, p_id) if t['mode'] == self.repeat else None

    def _stop(self, p_id):
        t = self.timers.get(p_id)
        if t:
            self.tw.cancel(t['ev'])
            t['ev'] = None
        return None

    def _delete(self, p_id):
        self._stop(p_id)
        self.timers.pop(p_id, None)
        return OK


class Uart:
    """app_uart : réception asynchrone par morceaux (événement RX_DATA), émission synchrone."""

    def __init__(self, tw):
        self.tw = tw
        L = tw.L
        self.EVT_RX, self.EVT_TX = L.enum('APP_UART_EVT_RX_DATA'), L.enum('APP_UART_EVT_TX_CPLT')
        self.ports = {}                     # id -> dict(cb, baud, buf, size)
        self.devices = {}                   # id -> fn(octets) appelée sur émission du firmware
        self.fifo = defaultdict(bytearray)  # octets arrivés sans réception armée
        self.overflow = defaultdict(int)
        self.tx_log = defaultdict(bytearray)
        tw.reset_listeners.append(self._on_reset)
        tw.on('app_uart_init', self._init)
        tw.on('app_uart_deinit', self._deinit)
        tw.on('app_uart_receive_async', self._rx_async)
        tw.on('app_uart_transmit_sync', self._tx_sync)
        tw.on('app_uart_transmit_async', self._tx_async)

    def _on_reset(self):
        self.ports.clear()
        self.fifo.clear()

    def attach(self, uid, fn):
        self.devices[uid] = fn

    def is_open(self, uid):
        return uid in self.ports

    def _init(self, p_params, cb, p_txbuf):
        L = self.tw.L
        d = self.tw.read(p_params, 96)
        uid = L.get(d, 'app_uart_params_t', 'id')
        baud = L.get(d, 'app_uart_params_t', 'init.baud_rate')
        self.ports[uid] = {'cb': cb, 'baud': baud or 115200, 'buf': 0, 'size': 0, 'ev': None}
        self.fifo[uid].clear()
        self.tw.log('uart_init', id=uid, baud=baud)
        return OK

    def _deinit(self, uid):
        p = self.ports.pop(uid, None)
        if p:
            self.tw.cancel(p['ev'])
        self.tw.log('uart_deinit', id=uid)
        return OK

    def _rx_async(self, uid, buf, size):
        p = self.ports.get(uid)
        if not p:
            return 1
        p['buf'], p['size'] = buf, size
        if self.fifo[uid]:
            self._deliver(uid)
        return OK

    def _tx_sync(self, uid, data, size, timeout):
        p = self.ports.get(uid)
        if not p:
            return 1
        b = self.tw.read(data, size)
        self.tx_log[uid] += b
        self.tw.cycles_add(size * 10 * F_CPU // p['baud'])
        f = self.devices.get(uid)
        if f:
            f(b)
        return OK

    def _tx_async(self, uid, data, size):
        r = self._tx_sync(uid, data, size, 0)
        if r == OK:
            evt = self.tw.L.pack('app_uart_evt_t', type=self.EVT_TX, data__size=size)
            self.tw.irq('uart tx', self.ports[uid]['cb'], (None,), (evt,))
        return r

    def feed(self, uid, data):
        """Octets émis par le composant vers le GR5515 (arrivent au rythme du débit de l'UART)."""
        p = self.ports.get(uid)
        if not p:
            self.overflow[uid] += len(data)          # UART fermé : octets perdus
            return
        self.fifo[uid] += data
        if len(self.fifo[uid]) > 4096:
            self.overflow[uid] += len(self.fifo[uid]) - 4096
            del self.fifo[uid][:-4096]
        if p['size'] and p['ev'] is None:
            # morceau livré après la durée de transmission (+ délai d'inactivité du récepteur)
            n = min(len(self.fifo[uid]), p['size'])
            p['ev'] = self.tw.schedule((n + 4) * 10 * F_CPU // p['baud'], self._deliver, uid)

    def _deliver(self, uid):
        p = self.ports.get(uid)
        if not p:
            return
        p['ev'] = None
        if not p['size'] or not self.fifo[uid]:
            return
        n = min(len(self.fifo[uid]), p['size'])
        chunk = bytes(self.fifo[uid][:n])
        del self.fifo[uid][:n]
        self.tw.write(p['buf'], chunk)
        p['size'] = 0                                   # le firmware réarme dans le rappel
        evt = self.tw.L.pack('app_uart_evt_t', type=self.EVT_RX, data__size=n)
        self.tw.irq('uart rx', p['cb'], (None,), (evt,))
        if self.fifo[uid]:
            p['ev'] = self.tw.schedule(min(len(self.fifo[uid]), 64) * 10 * F_CPU // p['baud'], self._deliver, uid)


class Spi:
    """app_spi maître : chaque composant est sélectionné par sa broche CS (GPIO pilotée par le firmware)."""

    def __init__(self, tw, gpio):
        self.tw, self.gpio = tw, gpio
        self.devices = {}                   # broche CS -> composant (select/xfer/deselect)
        self.ready = False
        tw.reset_listeners.append(lambda: setattr(self, 'ready', False))
        tw.on('app_spi_init', self._init)
        tw.on('app_spi_deinit', self._deinit)
        tw.on('app_spi_transmit_receive_sync', self._xfer)
        tw.on('app_spi_transmit_sync', self._tx)
        tw.on('app_spi_receive_sync', self._rx)

    def attach(self, cs_pin, dev):
        self.devices[cs_pin] = dev

        def on_cs(level):
            (dev.deselect if level else dev.select)()
        self.gpio.watch(cs_pin, on_cs)

    def _selected(self):
        sel = [d for p, d in self.devices.items() if self.gpio.is_output(p) and not self.gpio.level(p)]
        return sel[0] if len(sel) == 1 else None

    def _init(self, p_params, cb):
        self.ready = True
        return OK

    def _deinit(self, sid):
        self.ready = False
        return OK

    def _transfer(self, out):
        if not self.ready:
            return None
        self.tw.cycles_add(len(out) * 8 * 8)          # 8 MHz
        d = self._selected()
        if d is None:
            return b'\xff' * len(out)
        return d.xfer(bytes(out))

    def _xfer(self, sid, tx, rx, size, timeout):
        r = self._transfer(self.tw.read(tx, size))
        if r is None:
            return 1
        self.tw.write(rx, r)
        return OK

    def _tx(self, sid, data, size, timeout):
        return OK if self._transfer(self.tw.read(data, size)) is not None else 1

    def _rx(self, sid, data, size, timeout):
        r = self._transfer(b'\x00' * size)
        if r is None:
            return 1
        self.tw.write(data, r)
        return OK


# Capteur de température interne (entrée ADC_INPUT_SRC_TMP) : code = T0 + PENTE × 0,00175 × (T − 25 °C), avec les
# valeurs par défaut du SDK Goodix (exemple adc_temp_vbat), utilisées faute d'étalonnage d'usine (sys_adc_trim_get).
ADC_SRC_TMP = 5
TMP_T0, TMP_SLOPE = 4979, 4932


class Adc:
    """app_adc : tensions des broches MSIO fournies par des sources Python (mV), et température interne du GR5515."""

    def __init__(self, tw):
        self.tw = tw
        self.sources = {}                   # n° MSIO -> fn() -> mV
        self.channel = None
        self.die_temp_c = 22.0              # température de la puce (°C)
        tw.on('app_adc_init', self._init)
        tw.on('app_adc_deinit', lambda: OK)
        tw.on('app_adc_conversion_sync', self._conv)
        tw.on('app_adc_voltage_intern', self._volts)
        tw.on('sys_adc_trim_get', lambda p_trim: 1)      # pas d'étalonnage d'usine dans le jumeau

    def _init(self, p_params, cb):
        L = self.tw.L
        size = L.field('app_adc_params_t', 'init.channel_n')[0] + 4
        data = self.tw.read(p_params, max(16, size))
        if L.get(data, 'app_adc_params_t', 'init.channel_n') == ADC_SRC_TMP:
            self.channel = 'tmp'
            return OK
        mask = L.get(data, 'app_adc_params_t', 'pin_cfg.channel_p.pin')
        self.channel = (mask & -mask).bit_length() - 1
        return OK

    def _conv(self, p_data, length, timeout):
        if self.channel == 'tmp':
            code = int(round(TMP_T0 + TMP_SLOPE * 0.00175 * (self.die_temp_c - 25.0)))
            self.tw.write(p_data, struct.pack(f'<{length}H', *([code] * length)))
            self.tw.cycles_add(length * 64)
            self.tw.log('adc', source='tmp', temp_c=self.die_temp_c)
            return OK
        f = self.sources.get(self.channel)
        mv = f() if f else 0
        code = max(0, min(0xFFFF, int(round(mv * 4))))       # code fictif = 0,25 mV
        self.tw.write(p_data, struct.pack(f'<{length}H', *([code] * length)))
        self.tw.cycles_add(length * 64)
        self.tw.log('adc', msio=self.channel, mv=mv)
        return OK

    def _volts(self, inbuf, outbuf, n):
        codes = struct.unpack(f'<{n}H', self.tw.read(inbuf, 2 * n))
        self.tw.write(outbuf, struct.pack(f'<{n}d', *[c / 4000.0 for c in codes]))
        return OK


class Pwm:
    """app_pwm : on journalise la fréquence produite (buzzer) et le rapport cyclique (LED, analogWrite).

    Horloge commune : PWM0 et PWM1 n'ont qu'une horloge (MCU_SUB_CG, PWM_HCLK, bit 3 de CG_CTRL0 « coupée en WFI »
    et de CG_CTRL1 « coupée de force »). Comme le HAL réel, app_pwm_deinit() la coupe et app_pwm_init() la rouvre ;
    tant qu'elle est coupée de force, aucun module ne produit rien (duty vide), même si ses registres restent réglés
    (vu sur T1000-A : la fin d'un son du buzzer figeait la LED de charge)."""

    CG_WFI, CG_FORCE, PWM_HCLK = 0xA000E2A0, 0xA000E2A4, 1 << 3

    def __init__(self, tw):
        self.tw = tw
        self.params = {}                    # id -> fréquence configurée
        self.running = {}
        self.cfg_duty = {}                  # id -> rapport cyclique configuré (%) de la voie active
        self._duty = {}                     # id -> rapport cyclique produit (%), module démarré seulement
        tw.on('app_pwm_init', self._init)
        tw.on('app_pwm_deinit', self._deinit)
        tw.on('app_pwm_start', self._start)
        tw.on('app_pwm_stop', self._stop)
        tw.on('app_pwm_update_freq', self._freq)
        tw.on('app_pwm_config_channel', self._config)
        tw.on('gr551x_pwm_set_level', self._level)      # analogWrite() : comparateur écrit directement

    def _clock(self, on):
        for reg in (self.CG_WFI, self.CG_FORCE):
            v = self.tw.r32(reg)
            self.tw.w32(reg, (v & ~self.PWM_HCLK) if on else (v | self.PWM_HCLK))

    @property
    def duty(self):
        """Rapport cyclique réellement produit, par module : rien si l'horloge PWM est coupée de force."""
        return {} if self.tw.r32(self.CG_FORCE) & self.PWM_HCLK else self._duty

    def _init(self, p):
        self._clock(True)
        L = self.tw.L
        d = self.tw.read(p, L.size('app_pwm_params_t'))
        pid = L.get(d, 'app_pwm_params_t', 'id')
        self.params[pid] = L.get(d, 'app_pwm_params_t', 'init.freq')
        active = L.get(d, 'app_pwm_params_t', 'active_channel')
        for bit, ch in ((1, 'a'), (2, 'b'), (4, 'c')):
            if active & bit:
                self.cfg_duty[pid] = L.get(d, 'app_pwm_params_t', f'init.channel_{ch}.duty')
                break
        return OK

    def _deinit(self, pid):
        self.running.pop(pid, None)
        self._duty.pop(pid, None)
        self._clock(False)                  # HAL réel : horloge commune coupée, même si l'autre module tourne
        return OK

    def _start(self, pid):
        self.running[pid] = self.params.get(pid, 0)
        self._duty[pid] = self.cfg_duty.get(pid, 0)
        self.tw.log('pwm', id=pid, freq=self.running[pid], duty=self._duty[pid])
        return OK

    def _stop(self, pid):
        self._duty.pop(pid, None)
        if self.running.pop(pid, None) is not None:
            self.tw.log('pwm', id=pid, freq=0)
        return OK

    def _freq(self, pid, freq):
        self.params[pid] = freq
        if pid in self.running:
            self.running[pid] = freq
            self.tw.log('pwm', id=pid, freq=freq)
        return OK

    def _level(self, pid, channel, level):
        self.cfg_duty[pid] = round((level & 0xFF) * 100 / 255)
        if pid in self.running:
            self._duty[pid] = self.cfg_duty[pid]
        return None

    def _config(self, pid, channel, p_cfg):
        duty = self.tw.read(p_cfg, 1)[0]            # app_pwm_channel_init_t : duty (uint8_t) en tête
        self.cfg_duty[pid] = duty
        if pid in self.running:
            self._duty[pid] = duty
        return OK


class Watchdog:
    """Chien de garde AON : redémarre le MCU s'il n'est pas rafraîchi à temps."""

    def __init__(self, tw):
        self.tw = tw
        self.ev = None
        self.period = None
        self.refreshes = 0
        tw.reset_listeners.append(self._off)
        tw.on('app_aon_wdt_init', self._init)
        tw.on('app_aon_wdt_deinit', self._deinit)
        tw.on('app_aon_wdt_refresh', self._refresh)

    def _off(self):
        self.ev, self.period = None, None

    def _init(self, p, cb):
        counter = self.tw.L.get(self.tw.read(p, 8), 'app_aon_wdt_params_t', 'init.counter')
        self.period = counter * F_CPU // 32768
        self.tw.log('wdt_init', seconds=counter / 32768)
        self._arm()
        return OK

    def _arm(self):
        self.tw.cancel(self.ev)
        self.ev = self.tw.schedule(self.period, self._bite) if self.period else None

    def _refresh(self):
        self.refreshes += 1
        if self.period:
            self._arm()
        return OK

    def _deinit(self):
        self.tw.cancel(self.ev)
        self._off()
        return OK

    def _bite(self):
        self.tw.log('wdt_bite')
        raise Reset('chien de garde AON')
