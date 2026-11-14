"""Chemins vers le dépôt racine T1000A (SDK Goodix, firmwares construits), sans dépendance.

Le jumeau vit dans son propre dépôt, monté en sous-module dans T1000A (emulator/twin). Racine T1000A :
  1. $T1000_ROOT ;
  2. sinon le premier dossier parent du jumeau qui contient third_party/gr551x-sdk.
Symboles de la ROM : $GR551X_SDK/platform/soc/linker/gcc/rom_symbol_gcc.txt si $GR551X_SDK est défini,
sinon <racine>/third_party/gr551x-sdk/….
"""
import os

TWIN_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _find_root():
    env = os.environ.get('T1000_ROOT')
    if env:
        return os.path.abspath(env)
    d = TWIN_DIR
    while True:
        if os.path.isdir(os.path.join(d, 'third_party', 'gr551x-sdk')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            return None
        d = parent


ROOT = _find_root()
SDK = os.environ.get('GR551X_SDK') or (ROOT and os.path.join(ROOT, 'third_party', 'gr551x-sdk'))
ROM_SYMS = SDK and os.path.join(SDK, 'platform', 'soc', 'linker', 'gcc', 'rom_symbol_gcc.txt')


def in_root(*parts):
    """Chemin dans le dépôt T1000A, ou None si la racine est introuvable."""
    return ROOT and os.path.join(ROOT, *parts)


def rom_syms():
    """Fichier des symboles ROM ; erreur explicite s'il manque."""
    if not ROM_SYMS or not os.path.exists(ROM_SYMS):
        raise FileNotFoundError(
            f'symboles ROM introuvables ({ROM_SYMS}) : jumeau hors de T1000A ? '
            'définir T1000_ROOT=<dépôt T1000A> ou GR551X_SDK=<SDK Goodix>')
    return ROM_SYMS
