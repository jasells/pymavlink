#!/usr/bin/env python3
'''
Golden test vectors for MAVLink code generators: for every message a dialect XML file defines, a few
deterministic payloads and the exact MAVLink 2 frame pymavlink's Python implementation produces for
each. Any generated language (C#, C, C++) can check itself against the same JSON:
  1. decode `frame` and compare the fields by name;
  2. encode `fields` and compare the bytes exactly.
Two implementations that both match the vectors agree with each other, so no pairwise interop harness
is needed.

The frames come from pymavlink's Python generator run on the SAME XML, in a temporary directory, so the
installed pymavlink.dialects (which may not match the XML) are never used.

Cases per message (deterministic: re-running on the same XML and pymavlink yields an identical file):
  zeros     every field zero: MAVLink 2 trims the payload to a single byte
  max       integers all bits set (unsigned max, signed -1); floats the largest finite float32;
            char arrays 0xFF bytes
  random    seeded random, every field non-zero
  trailing  seeded random with the trailing half of the base fields (wire order) and every extension
            field zero: partial payload truncation, and an old sender that omits extensions

JSON value rules, so every language can compare exactly:
  integers are numbers, except 64-bit ones, which are decimal strings (JSON numbers lose precision);
  floats are exactly representable in float32 and never NaN/Inf; char[N] fields are hex strings of
  exactly N bytes; other arrays are lists.

Usage:
  python -m pymavlink.tools.mavgen_vectors --output DIR common.xml [more.xml ...]
writes DIR/<basename>.v2.json per XML file, with only the messages that file defines.

Released under GNU GPL version 3 or later, as the rest of the generator. The vectors are generator
output and not covered by it.
'''

import argparse
import hashlib
import importlib.util
import json
import os
import random
import struct
import subprocess
import sys
import tempfile

from pymavlink.generator import mavgen, mavparse

INT_RANGES = {
    'int8_t': (-2**7, 2**7 - 1), 'uint8_t': (0, 2**8 - 1),
    'int16_t': (-2**15, 2**15 - 1), 'uint16_t': (0, 2**16 - 1),
    'int32_t': (-2**31, 2**31 - 1), 'uint32_t': (0, 2**32 - 1),
    'int64_t': (-2**63, 2**63 - 1), 'uint64_t': (0, 2**64 - 1),
}

FLOAT32_MAX = struct.unpack('<f', struct.pack('<I', 0x7F7FFFFF))[0]

CASES = ('zeros', 'max', 'random', 'trailing')


def xml_hash(path):
    '''SHA-256 of the file's text with CRLF normalised to LF (a UTF-8 BOM ignored), lower-case hex.
    The same definition MavLink.Net.CodeGen records in MavlinkBuildInfo.XmlHashes.'''
    with open(path, 'rb') as f:
        text = f.read().decode('utf-8-sig').replace('\r\n', '\n')
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def git_commit(path):
    try:
        out = subprocess.run(['git', '-C', path, 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True)
        dirty = subprocess.run(['git', '-C', path, 'status', '--porcelain'], capture_output=True, text=True, check=True)
        return out.stdout.strip() + (' (with uncommitted changes)' if dirty.stdout.strip() else '')
    except Exception:
        return 'unknown'


def generate_python(xml_path, directory):
    '''Generate pymavlink's Python 3 implementation of the XML and import it from the file.'''
    out = os.path.join(directory, 'vectors_dialect.py')
    opts = mavgen.Opts(out, wire_protocol=mavparse.PROTOCOL_2_0, language='Python3', validate=False)
    if not mavgen.mavgen(opts, [xml_path]):
        raise SystemExit('mavgen failed for %s' % xml_path)
    spec = importlib.util.spec_from_file_location('vectors_dialect_%d' % abs(hash(xml_path)), out)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def scalar_value(field, case, rng):
    t = field.type
    if t == 'char':
        raise AssertionError('char handled as an array')
    if t in ('float', 'double'):
        if case == 'zeros':
            return 0.0
        if case == 'max':
            return FLOAT32_MAX
        # Exactly representable in float32: a small integer plus a half.
        sign = rng.choice((-1, 1))
        return sign * (rng.randint(1, 1000) + 0.5)
    lo, hi = INT_RANGES[t]
    if case == 'zeros':
        return 0
    if case == 'max':
        return hi if lo == 0 else -1
    v = 0
    while v == 0:
        v = rng.randint(lo, hi)
    return v


def field_value(field, case, rng):
    n = field.array_length
    if field.type == 'char':
        size = max(n, 1)
        if case == 'zeros':
            return bytes(size)
        if case == 'max':
            return b'\xff' * size
        return bytes(rng.randint(1, 255) for _ in range(size))
    if n:
        return [scalar_value(field, case, rng) for _ in range(n)]
    return scalar_value(field, case, rng)


def message_values(m, case):
    rng = random.Random(m.id * len(CASES) + CASES.index(case))
    base_case = 'random' if case == 'trailing' else case
    values = {f.name: field_value(f, base_case, rng) for f in m.ordered_fields}

    if case == 'trailing':
        base = m.ordered_fields[:m.base_fields()]
        extensions = m.ordered_fields[m.base_fields():]
        for f in base[(len(base) + 1) // 2:] + extensions:
            values[f.name] = field_value(f, 'zeros', rng)
    return values


def to_json(field, value):
    if field.type == 'char':
        return value.hex()
    if field.type in ('int64_t', 'uint64_t'):
        return [str(v) for v in value] if isinstance(value, list) else str(value)
    return value


def to_python(field, value):
    # pymavlink's Python messages take char fields as bytes and pack them with struct's 's'.
    return value


def vectors_for(xml_path, module, mav_class):
    x = mavparse.MAVXML(xml_path, mavparse.PROTOCOL_2_0)
    vectors = []
    for m in sorted(x.message, key=lambda mm: mm.id):
        cls = getattr(module, 'MAVLink_%s_message' % m.name.lower())
        fields_by_name = {f.name: f for f in m.fields}
        for seq, case in enumerate(CASES):
            values = message_values(m, case)
            mav = mav_class(None, srcSystem=1, srcComponent=1)
            mav.seq = seq
            msg = cls(**{name: to_python(fields_by_name[name], v) for name, v in values.items()})
            frame = msg.pack(mav)
            vectors.append({
                'name': m.name,
                'id': m.id,
                'case': case,
                'sys': 1,
                'comp': 1,
                'seq': seq,
                'fields': {f.name: to_json(f, values[f.name]) for f in m.fields},
                'frame': frame.hex(),
            })
    return x, vectors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0].strip())
    parser.add_argument('--output', required=True, help='directory for <basename>.v2.json files')
    parser.add_argument('definitions', nargs='+', metavar='XML', help='dialect XML files')
    args = parser.parse_args(argv)

    os.makedirs(args.output, exist_ok=True)
    commit = git_commit(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    for xml_path in args.definitions:
        with tempfile.TemporaryDirectory() as tmp:
            module = generate_python(xml_path, tmp)
            x, vectors = vectors_for(xml_path, module, module.MAVLink)

        document = {
            'dialect': x.basename,
            'wire': 2,
            'generator': 'pymavlink mavgen_vectors.py',
            'pymavlink': commit,
            'xmlFile': os.path.basename(xml_path),
            'xmlHash': xml_hash(xml_path),
            'cases': list(CASES),
            'vectors': vectors,
        }
        out = os.path.join(args.output, '%s.v2.json' % x.basename)
        with open(out, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(document, f, indent=1)
            f.write('\n')
        print('Wrote %s: %d vectors for %d messages' % (out, len(vectors), len(vectors) // len(CASES)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
