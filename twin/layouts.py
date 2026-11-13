"""Struct layouts and enum values read from the firmware's DWARF info (no hard-coded offsets).

The SDK structures shared between the firmware and the twin's models (app_io_evt_t, ble_evt_t…) are
read from the ELF itself, so the models follow the SDK actually compiled into the image.
The first parse takes a few seconds; the result is cached in ~/.cache/t1000-twin/<sha1>.json.

    L = Layouts('firmware.elf')
    L.off('ble_evt_t', 'evt.gatts_evt.params.write_req.handle')   # offset in bytes
    L.size('app_io_evt_t')
    L.enum('APP_IO_TYPE_AON')
    L.pack('app_io_evt_t', type=L.enum('APP_IO_TYPE_AON'), pin=2)  # bytes
"""
import hashlib, json, os, struct

CACHE_DIR = os.path.expanduser('~/.cache/t1000-twin')
CACHE_VERSION = 2
# Named types that are flattened (C structures of the SDK and of our code); C++ classes are ignored.
PREFIXES = ('app_', 'ble_', 'gap', 'gatt', 'sec_', 'hal_', 'sdk_', 'pwr_', 'nvds', 'aon_', 'calendar',
            'uart_', 'spi_', 'adc_', 'pwm_', 'wdt_', 'gr551x', 'exflash', 'xqspi')
MAX_DEPTH = 6
# Valeurs du SDK GR551x utilisées quand un pilote n'est pas lié dans le firmware (énumération absente du DWARF)
SDK_DEFAULTS = {
    'APP_IO_TYPE_GPIOA': 0, 'APP_IO_TYPE_GPIOB': 1, 'APP_IO_TYPE_AON': 3, 'APP_IO_TYPE_MSIO': 4,
    'APP_IO_TYPE_NORMAL': 5, 'APP_IO_MODE_INPUT': 1, 'APP_IO_MODE_OUTPUT': 2, 'APP_IO_MODE_MUX': 3,
    'APP_IO_MODE_IT_RISING': 4, 'APP_IO_MODE_IT_FALLING': 5, 'APP_IO_MODE_IT_HIGH': 6, 'APP_IO_MODE_IT_LOW': 7,
    'APP_IO_PULLUP': 1, 'APP_IO_PULLDOWN': 2, 'APP_UART_EVT_TX_CPLT': 1, 'APP_UART_EVT_RX_DATA': 2,
    'ATIMER_ONE_SHOT': 0, 'ATIMER_REPEAT': 1, 'PMR_MGMT_ACTIVE_MODE': 0, 'PMR_MGMT_IDLE_MODE': 1,
}


class Layouts:
    def __init__(self, elf_path):
        self.elf_path = elf_path
        h = hashlib.sha1(open(elf_path, 'rb').read()).hexdigest()
        self.cache = os.path.join(CACHE_DIR, f'{h}.json')
        data = None
        if os.path.exists(self.cache):
            try:
                data = json.load(open(self.cache))
                if data.get('version') != CACHE_VERSION:
                    data = None
            except (OSError, ValueError):
                data = None
        if data is None:
            data = self._parse()
            os.makedirs(CACHE_DIR, exist_ok=True)
            tmp = self.cache + '.tmp'
            with open(tmp, 'w') as f:
                json.dump(data, f)
            os.replace(tmp, self.cache)
        self.types = data['types']          # name -> {'size': n, 'fields': {path: [off, size, elem]}}
        self.enums = data['enums']          # NAME -> value

    # ------------------------------------------------------------------ queries
    def size(self, typ):
        return self.types[typ]['size']

    def field(self, typ, path):
        try:
            return self.types[typ]['fields'][path]
        except KeyError:
            raise KeyError(f'{typ}.{path} absent du DWARF du firmware') from None

    def off(self, typ, path):
        return self.field(typ, path)[0]

    def enum(self, name):
        if name in self.enums:
            return self.enums[name]
        if name in SDK_DEFAULTS:            # pilote non lié dans ce firmware : valeur du SDK
            return SDK_DEFAULTS[name]
        raise KeyError(f'énumérateur {name} absent du DWARF du firmware')

    def has(self, typ, path=None):
        return typ in self.types and (path is None or path in self.types[typ]['fields'])

    def pack(self, typ, **fields):
        """Builds the bytes of `typ` (zero-filled) with the given fields; '__' in a name stands for '.'."""
        buf = bytearray(self.size(typ))
        for k, v in fields.items():
            self.put(buf, typ, k.replace('__', '.'), v)
        return bytes(buf)

    def put(self, buf, typ, path, value, base=0):
        off, size, _ = self.field(typ, path)
        if isinstance(value, (bytes, bytearray)):
            buf[base + off:base + off + len(value)] = value
        else:
            buf[base + off:base + off + size] = (value & ((1 << (8 * size)) - 1)).to_bytes(size, 'little')

    def get(self, data, typ, path, base=0):
        off, size, _ = self.field(typ, path)
        return int.from_bytes(bytes(data[base + off:base + off + size]), 'little')

    # ------------------------------------------------------------------ DWARF parsing
    def _parse(self):
        from elftools.elf.elffile import ELFFile
        elf = ELFFile(open(self.elf_path, 'rb'))
        dwarf = elf.get_dwarf_info()
        types, enums = {}, {}
        for cu in dwarf.iter_CUs():
            for die in cu.iter_DIEs():
                tag = die.tag
                if tag == 'DW_TAG_enumerator':
                    enums.setdefault(_name(die), _const(die))
                elif tag in ('DW_TAG_typedef', 'DW_TAG_structure_type', 'DW_TAG_union_type'):
                    name = _name(die)
                    if not name or name in types or not name.startswith(PREFIXES):
                        continue
                    t = _resolve(die)
                    if t is None or t.tag not in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
                        continue
                    if 'DW_AT_declaration' in t.attributes:
                        continue
                    fields = {}
                    _flatten(t, '', 0, fields, 0)
                    types[name] = {'size': _size(t), 'fields': fields}
        return {'version': CACHE_VERSION, 'types': types, 'enums': enums}


def _name(die):
    a = die.attributes.get('DW_AT_name')
    return a.value.decode() if a else None


def _const(die):
    a = die.attributes['DW_AT_const_value']
    return a.value


def _ref(die):
    a = die.attributes.get('DW_AT_type')
    return die.get_DIE_from_attribute('DW_AT_type') if a else None


def _resolve(die):
    while die is not None and die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
        die = _ref(die)
    return die


def _size(die):
    die = _resolve(die)
    if die is None:
        return 0
    if 'DW_AT_byte_size' in die.attributes:
        return die.attributes['DW_AT_byte_size'].value
    if die.tag == 'DW_TAG_array_type':
        n = 1
        for sub in die.iter_children():
            if sub.tag == 'DW_TAG_subrange_type':
                if 'DW_AT_count' in sub.attributes:
                    n *= sub.attributes['DW_AT_count'].value
                elif 'DW_AT_upper_bound' in sub.attributes:
                    n *= sub.attributes['DW_AT_upper_bound'].value + 1
                else:
                    n = 0
        return n * _size(_ref(die))
    if die.tag == 'DW_TAG_pointer_type':
        return 4
    return 0


def _flatten(t, prefix, base, out, depth):
    if depth > MAX_DEPTH:
        return
    for m in t.iter_children():
        if m.tag != 'DW_TAG_member':
            continue
        if 'DW_AT_bit_size' in m.attributes:
            continue
        name = _name(m)
        loc = m.attributes.get('DW_AT_data_member_location')
        off = base + (loc.value if loc is not None and isinstance(loc.value, int) else 0)
        mt = _resolve(_ref(m))
        if mt is None:
            continue
        if name is None:                                  # anonymous union/structure: fields lifted
            if mt.tag in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
                _flatten(mt, prefix, off, out, depth + 1)
            continue
        path = prefix + name
        elem = _size(_ref(mt)) if mt.tag == 'DW_TAG_array_type' else 0
        out[path] = [off, _size(mt), elem]
        if mt.tag in ('DW_TAG_structure_type', 'DW_TAG_union_type'):
            _flatten(mt, path + '.', off, out, depth + 1)
