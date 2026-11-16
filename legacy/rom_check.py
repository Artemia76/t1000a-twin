#!/usr/bin/env python3
"""Vérifie que le firmware Seeed (qui fonctionne sur le boîtier) et notre SDK visent la même ROM GR5515.

Collecte toutes les cibles d'appel (bl / b.w) hors de la Flash applicative dans le firmware Seeed,
puis vérifie qu'elles tombent sur des points d'entrée de la table de symboles ROM du SDK
(platform/soc/linker/gcc/rom_symbol_gcc.txt). Si les adresses correspondent, la ROM de la puce
est celle pour laquelle le SDK est compilé.

Usage : rom_check.py <firmware_seeed.bin>
"""
import os, re, struct, sys, capstone
from collections import Counter
from capstone import arm

_here = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [_here, os.path.dirname(_here)]   # la racine du jumeau (twin)
from twin.paths import rom_syms  # noqa: E402
if len(sys.argv) < 2:
    sys.exit('usage : rom_check.py <firmware Seeed .bin>')
FW = sys.argv[1]
SYM = rom_syms()
BASE, CODE_END = 0x0101B000, 0x52000

syms = {}
for line in open(SYM):
    m = re.match(r'\s*(\w+)\s*=\s*(0x[0-9a-fA-F]+)', line)
    if m:
        syms[int(m.group(2), 16) & ~1] = m.group(1)

d = open(FW, 'rb').read()
md = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
md.detail = True
targets = Counter()
pos = 0
while pos < CODE_END - 2:
    got = False
    for i in md.disasm(d[pos:CODE_END], BASE + pos):
        got = True
        pos = i.address + i.size - BASE
        if i.mnemonic in ('bl', 'b.w') and i.operands and i.operands[0].type == arm.ARM_OP_IMM:
            t = i.operands[0].imm & ~1
            if t < 0x00080000:          # ROM uniquement (0x0080xxxx = code RAM propre à l'appli)
                targets[t] += 1
    if not got:
        pos += 2

real = dict(targets)
hit = {t: n for t, n in real.items() if t in syms}
miss = {t: n for t, n in real.items() if t not in syms}
calls_hit, calls_all = sum(hit.values()), sum(real.values())
print(f'cibles hors Flash appli  : {len(real)} ; dans la table ROM du SDK : {len(hit)} '
      f'({100 * len(hit) // max(1, len(real))} %) ; appels couverts : {calls_hit}/{calls_all} '
      f'({100 * calls_hit // max(1, calls_all)} %)')
print('les plus appelées présentes :')
for t, n in sorted(hit.items(), key=lambda x: -x[1])[:15]:
    print(f'   {t:#010x}  ×{n:3d}  {syms[t]}')
print('absentes de la table (les plus appelées) :')
for t, n in sorted(miss.items(), key=lambda x: -x[1])[:15]:
    near = max((a for a in syms if a <= t), default=None)
    print(f'   {t:#010x}  ×{n:3d}  (symbole précédent : {syms.get(near, "?")} @ {near:#x} +{t - near:#x})'
          if near is not None else f'   {t:#010x}  ×{n:3d}')
