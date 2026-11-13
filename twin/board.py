"""Assemblage du jumeau numérique d'un SenseCAP T1000-A : GR5515 + LR1110 + AG3335 + câblage de la carte.

    from twin import T1000A, World
    w = World()
    a = w.add(T1000A('firmware.elf', name='A'))
    w.run(5000)
    a.phone.connect()

Câblage modélisé (variants/t1000-a/pins_arduino.h) :
  bouton AON0 (actif haut) · LED AON3 · LR1110 : NSS 6, NRESET 12, BUSY 13, IRQ → AON1 inversé
  GNSS UART1 : EN 28, RESET 16 (actif bas) · batterie MSIO4 via pont ÷2 actif quand GPIO14 = 1
  buzzer PWM0 (EN GPIO15)
"""
from .core import Twin
from .periph import Gpio, Timers, Uart, Spi, Adc, Pwm, Watchdog
from .lr1110 import LR1110
from .ag3335 import AG3335
from .ble import BleStack, Phone

BUTTON, LED, BAT_EN, BUZZER_EN = 0x20, 0x23, 14, 15


class T1000A:
    def __init__(self, elf, name='t1000a', flash=None, seed=0, trace=False, passkey=123456, battery_mv=3900,
                 radio=True, turbo=True):
        self.tw = tw = Twin(elf, flash=flash, seed=seed, name=name, trace=trace, turbo=turbo)
        self.name = name
        self.world = None
        self.gpio = Gpio(tw)
        self.timers = Timers(tw)
        self.uart = Uart(tw)
        self.spi = Spi(tw, self.gpio)
        self.adc = Adc(tw)
        self.pwm = Pwm(tw)
        self.wdt = Watchdog(tw)
        self.radio = LR1110(tw, self.gpio, self.spi) if radio else None     # radio=False : LR1110 muet (panne)
        self.gnss = AG3335(tw, self.gpio, self.uart)
        self.ble = BleStack(tw)
        self.phone = Phone(self.ble, passkey=passkey)
        self.battery_mv = battery_mv
        self.adc.sources[4] = lambda: self.battery_mv / 2 if self.gpio.level(BAT_EN) else 0.0
        self.gpio.drive(BUTTON, 0)

    # ------------------------------------------------------------------ commandes
    def press(self, ms_=None):
        """Appuie sur le bouton (et le relâche après ms_ millisecondes si donné)."""
        self.gpio.drive(BUTTON, 1)
        self.tw.log('button', pressed=True)
        if ms_ is not None:
            self.tw.schedule_ms(ms_, self.release)

    def release(self):
        self.gpio.drive(BUTTON, 0)
        self.tw.log('button', pressed=False)

    def led(self):
        return self.gpio.level(LED)

    def run(self, ms_=None, **kw):
        if self.world is not None:
            return self.world.run(ms_, **kw)
        return self.tw.run(ms_, **kw)

    def flash_image(self):
        return bytes(self.tw.flash)
