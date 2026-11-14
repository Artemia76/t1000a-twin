#!/usr/bin/env python3
"""Démarrage émulé d'un firmware T1000-A jusqu'à la boucle principale, avec trace en cas d'échec."""
import sys
from collections import deque
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from gr5515_sim import *
from twin.paths import in_root

elf = sys.argv[1] if len(sys.argv) > 1 else in_root('firmware', 'hello', 'build', 't1000a_hello.elf')
s = GR5515Sim(elf)
tr = deque(maxlen=int(sys.argv[2]) if len(sys.argv) > 2 else 12)
s.uc.hook_add(UC_HOOK_CODE, lambda uc, a, sz, _: tr.append((a, uc.reg_read(UC_ARM_REG_R0))))
try:
    print('RÉSULTAT :', s.boot())
except Stop as e:
    print('ARRÊT :', type(e).__name__, e)
    for a, r0 in tr:
        print(f'   {a:#010x} {s.where(a):40s} r0={r0:#x}')
print('appels ROM :', s.rom_calls.most_common(40))
print('événements :', s.events[:30])
