"""Jumeau numérique du SenseCAP T1000-A (GR5515 + LR1110 + AG3335), voir README.md."""
from .core import Twin, Stop, Reset, F_CPU, ms
from .board import T1000A
from .world import World, Ether
from .ble import Phone, BleStack
from .lr1110 import LR1110, time_on_air_us
from .ag3335 import AG3335
