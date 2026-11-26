#!/usr/bin/env python3
'''
Generate a split-layout C# implementation of MAVLink: one file and one namespace per XML dialect,
for the MavLink.Net runtime (https://xdevapps.visualstudio.com/DefaultCollection/MavLink.Net).

Differences from the legacy layout (mavgen_cs.py, which is left unchanged):
  * one <basename>.g.cs per XML file, each in its own namespace (from the optional <namespace>
    element, else <root>.Dialects.<PascalCase(basename)>), with types top-level in that namespace
  * message structs implement MavLink.Net.IMavlinkMessage<T> (id, CRC_EXTRA, lengths, name)
  * every payload struct is unmanaged: array fields are private fixed-size buffers exposed as a
    public Span<T> property ([UnscopedRef]), so reading and writing never allocates and consuming
    code needs no unsafe context; char[N] fields are text: <name> is a string property and
    <name>_span the raw bytes
  * enum-typed fields are typed, with a deterministic underlying type per enum (the widest
    unsigned width any field uses); fields of another width keep their wire type and get a
    checked typed accessor
  * an enum extended by several files is emitted once per file, with only that file's entries
  * a per-dialect MessageId enum and an IMavlinkDialect, registered explicitly by the application
    (no module initializer, so registration order is call order)
  * optional --cs-namespace: the root XML file's namespace only (includes keep theirs)
  * optional --cs-check-against: warn about message ids and MAV_CMD values defined differently in
    other dialects
  * no template files are copied

Released under GNU GPL version 3 or later, as the rest of the generator. Generated output is not
covered by the generator's license.
'''

import os
import re

from . import mavparse

CS_TYPES = {
    'float': 'float',
    'double': 'double',
    'char': 'byte',
    'int8_t': 'sbyte',
    'uint8_t': 'byte',
    'int16_t': 'short',
    'uint16_t': 'ushort',
    'int32_t': 'int',
    'uint32_t': 'uint',
    'int64_t': 'long',
    'uint64_t': 'ulong',
}

INTEGER_SIZES = {'sbyte': 1, 'byte': 1, 'short': 2, 'ushort': 2, 'int': 4, 'uint': 4, 'long': 8, 'ulong': 8}
SIGNED = {'sbyte', 'short', 'int', 'long'}
UNSIGNED_BY_SIZE = {1: 'byte', 2: 'ushort', 4: 'uint', 8: 'ulong'}
SIGNED_BY_SIZE = {1: 'sbyte', 2: 'short', 4: 'int', 8: 'long'}
RANGES = {
    'sbyte': (-2**7, 2**7 - 1), 'byte': (0, 2**8 - 1),
    'short': (-2**15, 2**15 - 1), 'ushort': (0, 2**16 - 1),
    'int': (-2**31, 2**31 - 1), 'uint': (0, 2**32 - 1),
    'long': (-2**63, 2**63 - 1), 'ulong': (0, 2**64 - 1),
}

CS_KEYWORDS = {
    'abstract', 'as', 'base', 'bool', 'break', 'byte', 'case', 'catch', 'char', 'checked', 'class',
    'const', 'continue', 'decimal', 'default', 'delegate', 'do', 'double', 'else', 'enum', 'event',
    'explicit', 'extern', 'false', 'finally', 'fixed', 'float', 'for', 'foreach', 'goto', 'if',
    'implicit', 'in', 'int', 'interface', 'internal', 'is', 'lock', 'long', 'namespace', 'new',
    'null', 'object', 'operator', 'out', 'override', 'params', 'private', 'protected', 'public',
    'readonly', 'ref', 'return', 'sbyte', 'sealed', 'short', 'sizeof', 'stackalloc', 'static',
    'string', 'struct', 'switch', 'this', 'throw', 'true', 'try', 'typeof', 'uint', 'ulong',
    'unchecked', 'unsafe', 'ushort', 'using', 'virtual', 'void', 'volatile', 'while',
}

CORE = 'global::MavLink.Net'


# ---- naming ---------------------------------------------------------------------------------------

def ident(name):
    '''escape a C# keyword'''
    return '@' + name if name in CS_KEYWORDS else name


def pascal(name):
    '''common -> Common, ASLUAV -> Asluav, all_dialects -> AllDialects'''
    parts = [p for p in re.split(r'[^A-Za-z0-9]+', name) if p]
    result = ''.join(p[:1].upper() + p[1:].lower() for p in parts)
    if not result or result[0].isdigit():
        result = '_' + result
    return result


def entry_name(enum_name, entry):
    '''the legacy C# naming, kept so call sites only change by namespace'''
    name = entry.name.replace('NAV_', '')
    name = name.replace(enum_name + '_', '')
    if re.match('^[0-9]', name):
        name = '_' + name
    return ident(name)


def dialect_namespace(x, root):
    ns = getattr(x, 'namespace', None)
    if ns:
        return ns.strip()
    return '%s.Dialects.%s' % (root, pascal(x.basename))


# ---- text -----------------------------------------------------------------------------------------

def one_line(text):
    return ' '.join((text or '').split())


def xml_doc(text):
    return one_line(text).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')


def cs_string(text):
    return '"' + one_line(text).replace('\\', '\\\\').replace('"', '\\"') + '"'


def obsolete(item):
    dep = getattr(item, 'deprecated', None)
    if not dep:
        return None
    text = 'Deprecated since %s' % dep.since
    if dep.replaced_by:
        text += '; replaced by %s' % dep.replaced_by
    if one_line(dep.explanation):
        text += '. ' + one_line(dep.explanation)
    return '[global::System.Obsolete(%s)]' % cs_string(text)


# ---- the include graph ----------------------------------------------------------------------------

def norm(path):
    return os.path.normcase(os.path.abspath(path))


def include_order(xml_list):
    '''files ordered so that every file comes after the files it includes'''
    by_name = {norm(x.filename): x for x in xml_list}
    ordered, seen = [], set()

    def visit(x):
        key = norm(x.filename)
        if key in seen:
            return
        seen.add(key)
        for i in x.include:
            inc = by_name.get(norm(os.path.join(os.path.dirname(x.filename), i)))
            if inc is not None:
                visit(inc)
        ordered.append(x)

    for x in xml_list:
        visit(x)
    return ordered


# ---- enums ----------------------------------------------------------------------------------------

class EnumModel(object):
    '''one enum across all files: entries by origin, underlying type, owner namespace'''

    def __init__(self, enum):
        self.name = enum.name
        self.enum = enum
        self.entries = [e for e in enum.entry if not getattr(e, 'end_marker', False)]
        self.uses = set()
        self.underlying = 'int'
        self.owner_namespace = None

    def entries_from(self, filename):
        key = norm(filename)
        return [e for e in self.entries if norm(e.origin_file or '') == key]

    def resolve_underlying(self):
        values = [e.value for e in self.entries] or [0]
        lo, hi = min(values), max(values)

        if self.uses:
            # Deterministic, unlike the legacy generator (whose last field seen won): the widest width
            # any field uses, unsigned unless the values need a sign.
            size = max(INTEGER_SIZES[u] for u in self.uses)
            table = SIGNED_BY_SIZE if lo < 0 else UNSIGNED_BY_SIZE
        else:
            size = 4
            table = SIGNED_BY_SIZE if lo < 0 or hi <= RANGES['int'][1] else UNSIGNED_BY_SIZE

        # Widen if the XML declares values the field width cannot hold.
        while True:
            candidate = table[size]
            rlo, rhi = RANGES[candidate]
            if rlo <= lo and hi <= rhi or size == 8:
                self.underlying = candidate
                return
            size *= 2

    @property
    def is_flags(self):
        e = self.enum
        return bool(getattr(e, 'bitmask', False)) \
            or 'bitmask' in (e.description or '').lower() \
            or '_flags' in e.name.lower()


def build_enum_models(xml_list):
    models = {}
    for x in xml_list:
        for e in x.enum:
            models[e.name] = EnumModel(e)

    for x in xml_list:
        for m in x.message:
            for f in m.fields:
                if f.enum and f.array_length == 0 and f.enum in models:
                    cs = CS_TYPES[f.type]
                    if cs in INTEGER_SIZES:
                        models[f.enum].uses.add(cs)

    for model in models.values():
        model.resolve_underlying()

    return models


def assign_owners(models, ordered, root):
    '''an enum's canonical declaration is in the first file (in include order) that adds entries to it'''
    for x in ordered:
        ns = dialect_namespace(x, root)
        for model in models.values():
            if model.owner_namespace is None and model.entries_from(x.filename):
                model.owner_namespace = ns
    for model in models.values():
        if model.owner_namespace is None:
            model.owner_namespace = dialect_namespace(ordered[0], root)


# ---- emitters -------------------------------------------------------------------------------------

class Writer(object):
    def __init__(self):
        self.lines = []

    def __call__(self, line='', indent=0):
        self.lines.append(('    ' * indent + line) if line else '')

    def text(self):
        return '\n'.join(self.lines) + '\n'


def emit_enum(w, model, entries, dialect_name):
    e = model.enum
    w('/// <summary>%s</summary>' % xml_doc(e.description))
    if model.is_flags:
        w('[global::System.Flags]')
    if one_line(e.description):
        w('[%s.MavlinkDescription(%s)]' % (CORE, cs_string(e.description)))
    w('public enum %s : %s' % (model.name, model.underlying))
    w('{')
    for entry in entries:
        w('/// <summary>%s</summary>' % xml_doc(entry.description), 1)
        if one_line(entry.description):
            w('[%s.MavlinkDescription(%s)]' % (CORE, cs_string(entry.description)), 1)
        dep = obsolete(entry)
        if dep:
            w(dep, 1)
        value = entry.value
        literal = str(value) if model.underlying in SIGNED or value >= 0 else str(value)
        w('%s = %s,' % (entry_name(model.name, entry), literal), 1)
        w('', 1)
    if w.lines[-1] == '':
        w.lines.pop()
    w('}')
    w()


def field_plan(m, f, models):
    '''how one field is emitted: (declaration, accessor lines)'''
    name = ident(f.name)
    cs = CS_TYPES[f.type]

    if f.array_length:
        # The public face is a span over the struct's own storage; the fixed buffer behind it is private,
        # so `unsafe` never reaches consuming code. [UnscopedRef] lets the compiler stop the span from
        # outliving the struct variable. The property comes first so the doc comment and attributes
        # attach to it; the private buffer follows in its wire position (properties don't affect layout).
        # char[N] fields are text: the plain name is a string property, and the raw bytes are a span
        # under text_span_name().
        storage = '_' + f.name
        n = f.array_length
        if f.type == 'char':
            return [
                'public string %s' % name,
                '{',
                '    readonly get { fixed (byte* p = %s) return %s.MavlinkText.Decode(new global::System.ReadOnlySpan<byte>(p, %u)); }' % (storage, CORE, n),
                '    set { fixed (byte* p = %s) %s.MavlinkText.Encode(value, new global::System.Span<byte>(p, %u)); }' % (storage, CORE, n),
                '}',
                'private fixed byte %s[%u];' % (storage, n),
            ], text_span_accessor(m, f, storage)
        return [
            '[global::System.Diagnostics.CodeAnalysis.UnscopedRef]',
            'public global::System.Span<%s> %s => global::System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref %s[0], %u);'
            % (cs, name, storage, n),
            'private fixed %s %s[%u];' % (cs, storage, n),
        ], []

    model = models.get(f.enum) if f.enum else None
    if model is None or cs not in INTEGER_SIZES:
        return ['public %s %s;' % (cs, name)], []

    enum_type = 'global::%s.%s' % (model.owner_namespace, model.name)
    if cs == model.underlying:
        return ['public %s %s;' % (enum_type, name)], []

    # Wire width differs from the enum's: keep the wire type, add a checked typed view.
    accessor_name = f.name + '_enum'
    if accessor_name in [fld.name for fld in m.fields]:
        return ['public %s %s;' % (cs, name)], []
    return ['public %s %s;' % (cs, name)], [
        '/// <summary>%s as <see cref="%s"/>. The wire field is %s; setting a value that does not fit throws.</summary>'
        % (f.name, enum_type, cs),
        'public %s %s' % (enum_type, accessor_name),
        '{',
        '    readonly get => (%s)%s;' % (enum_type, name),
        '    set => %s = checked((%s)value);' % (name, cs),
        '}',
    ]


def text_span_name(m, f):
    '''the raw-bytes span of a char[N] field: <name>_span, or <name>_bytes if a field already uses that'''
    names = set(fld.name for fld in m.fields)
    for suffix in ('_span', '_bytes'):
        if f.name + suffix not in names:
            return f.name + suffix
    raise mavparse.MAVParseError('%s.%s: no free name for the raw text span' % (m.name, f.name))


def text_span_accessor(m, f, storage):
    '''the raw bytes of a char[N] field, for callers that need the exact wire content'''
    span = text_span_name(m, f)
    if span != f.name + '_span':
        print('WARNING: %s.%s: %s_span is a field name, so the raw text span is %s' % (m.name, f.name, f.name, span))
    return [
        '/// <summary>The raw bytes of <see cref="%s"/>: %u bytes, NUL-padded, with no terminator when full.</summary>'
        % (ident(f.name).lstrip('@'), f.array_length),
        '[global::System.Diagnostics.CodeAnalysis.UnscopedRef]',
        'public global::System.Span<byte> %s => global::System.Runtime.InteropServices.MemoryMarshal.CreateSpan(ref %s[0], %u);'
        % (span, storage, f.array_length),
    ]


def emit_message(w, m, models, dialect_name):
    w('/// <summary>%s</summary>' % xml_doc(m.description))
    if one_line(m.description):
        w('[%s.MavlinkDescription(%s)]' % (CORE, cs_string(m.description)))
    dep = obsolete(m)
    if dep:
        w(dep)
    w('[global::System.Runtime.InteropServices.StructLayout(global::System.Runtime.InteropServices.LayoutKind.Sequential, Pack = 1, Size = %u)]' % m.wire_length)
    unsafe = 'unsafe ' if any(f.array_length for f in m.fields) else ''
    struct = 'mavlink_%s_t' % m.name.lower()
    w('public %spartial struct %s : %s.IMavlinkMessage<%s>' % (unsafe, struct, CORE, struct))
    w('{')
    w('/// <summary>The message id.</summary>', 1)
    w('public static uint MsgId => %u;' % m.id, 1)
    w('/// <summary>The CRC_EXTRA seed.</summary>', 1)
    w('public static byte CrcExtra => %u;' % m.crc_extra, 1)
    w('/// <summary>The full payload length, extensions included.</summary>', 1)
    w('public static int PayloadLength => %u;' % m.wire_length, 1)
    w('/// <summary>The payload length without extensions.</summary>', 1)
    w('public static int MinPayloadLength => %u;' % m.wire_min_length, 1)
    w('/// <summary>The message name.</summary>', 1)
    w('public static string Name => "%s";' % m.name, 1)

    accessors = []
    extensions_start = m.base_fields()
    for i, f in enumerate(m.ordered_fields):
        decl, extra = field_plan(m, f, models)
        w('', 1)
        doc = xml_doc(f.description)
        if f.enum:
            doc += ' (%s)' % f.enum
        if f.units:
            doc += ' %s' % xml_doc(f.units)
        if f.type == 'char' and f.array_length:
            # Uniform, because the XML descriptions mention the limit only sometimes.
            if doc and doc[-1] not in '.!?)':
                doc += '.'
            doc += ' Text: at most %u bytes of UTF-8, NUL-padded; setting a longer value truncates it.' % f.array_length
        if i >= extensions_start and m.extensions_start is not None:
            doc += ' Extension field: zero when the sender omits it.'
        w('/// <summary>%s</summary>' % doc.strip(), 1)
        if f.units:
            w('[%s.MavlinkUnits(%s)]' % (CORE, cs_string(f.units.strip('[]'))), 1)
        if one_line(f.description):
            w('[%s.MavlinkDescription(%s)]' % (CORE, cs_string(f.description)), 1)
        for line in decl:
            w(line, 1)
        if extra:
            accessors.append(extra)

    for block in accessors:
        w('', 1)
        for line in block:
            w(line, 1)
    w('}')
    w()


def emit_dialect_class(w, x, ns, class_name, messages, commands):
    w('/// <summary>The <c>%s</c> dialect: the messages and commands %s.xml defines.</summary>' % (x.basename, x.basename))
    w('/// <remarks>Not registered automatically: call <c>MavlinkRegistry.RegisterDialect(%s.Instance)</c>. Registration order is call order; the last registration wins.</remarks>' % class_name)
    w('public sealed class %s : %s.IMavlinkDialect' % (class_name, CORE))
    w('{')
    w('private %s()' % class_name, 1)
    w('{', 1)
    w('}', 1)
    w('', 1)
    w('/// <summary>Gets the single instance.</summary>', 1)
    w('public static %s Instance { get; } = new();' % class_name, 1)
    w('', 1)
    w('/// <inheritdoc/>', 1)
    w('public string Name => "%s";' % x.basename, 1)
    w('', 1)
    w('/// <inheritdoc/>', 1)
    w('public string Namespace => "%s";' % ns, 1)
    w('', 1)
    w('/// <inheritdoc/>', 1)
    w('#pragma warning disable CS0618 // deprecated messages are still on the wire and still registered', 1)
    w('public global::System.Collections.Generic.IReadOnlyList<%s.MavlinkMessageInfo> Messages { get; } =' % CORE, 1)
    w('[', 1)
    for m in messages:
        w('%s.MavlinkMessageInfo.Of<mavlink_%s_t>("%s"),' % (CORE, m.name.lower(), x.basename), 2)
    w('];', 1)
    w('#pragma warning restore CS0618', 1)
    w('', 1)
    w('/// <inheritdoc/>', 1)
    w('public global::System.Collections.Generic.IReadOnlyList<%s.MavlinkCommandInfo> Commands { get; } =' % CORE, 1)
    w('[', 1)
    for name, value, desc in commands:
        w('new(%u, "%s", "%s", %s),' % (value, name, x.basename, cs_string(desc) if one_line(desc) else 'null'), 2)
    w('];', 1)
    w('}')


def generate_file(x, xml_list, models, root, directory):
    ns = dialect_namespace(x, root)
    class_name = pascal(x.basename) + 'Dialect'
    w = Writer()

    w('// <auto-generated>')
    w('//   Generated by pymavlink mavgen (C#, split layout) from %s.xml.' % x.basename)
    w('//   Wire protocol %s. Do not edit: regenerate instead.' % x.wire_protocol_version)
    w('// </auto-generated>')
    w('#nullable enable')
    w('')
    w('namespace %s;' % ns)
    w('')

    # enums: only the entries that originate in this file
    commands = []
    for model in sorted(models.values(), key=lambda mm: mm.name):
        entries = model.entries_from(x.filename)
        if not entries:
            continue
        emit_enum(w, model, entries, x.basename)
        if model.name == 'MAV_CMD':
            commands = [(entry_name(model.name, e).lstrip('@'), e.value, e.description) for e in entries]

    # message ids
    messages = sorted(x.message, key=lambda mm: mm.id)
    if messages:
        w('/// <summary>The ids of the messages the <c>%s</c> dialect defines.</summary>' % x.basename)
        w('public enum MessageId : uint')
        w('{')
        for m in messages:
            w('/// <summary>%s</summary>' % m.name, 1)
            w('%s = %u,' % (m.name, m.id), 1)
        w('}')
        w('')

    for m in messages:
        emit_message(w, m, models, x.basename)

    emit_dialect_class(w, x, ns, class_name, messages, commands)

    path = os.path.join(directory, '%s.g.cs' % x.basename)
    with open(path, mode='w', encoding='utf-8', newline='\n') as f:
        f.write(w.text())
    print('Generated %s (%s, %u messages)' % (path, ns, len(messages)))


# ---- collisions with other dialects --------------------------------------------------------------

def parse_closure(path, wire_protocol, parsed):
    '''parse an XML file and everything it includes, once each'''
    key = norm(path)
    if key in parsed:
        return
    x = mavparse.MAVXML(path, wire_protocol)
    parsed[key] = x
    for i in x.include:
        parse_closure(os.path.join(os.path.dirname(path), i), wire_protocol, parsed)


def check_against(targets, xml_list, paths):
    '''warn, never fail, when a generated message id or MAV_CMD value is defined differently in another
    dialect. The registry resolves such conflicts at run time (last registration wins), but frames from
    a peer that uses the other definition then fail CRC, so a build warning is the right level.
    Returns the number of warnings.'''
    if not paths:
        return 0
    ours = set(norm(x.filename) for x in xml_list)
    others = {}
    for p in paths:
        parse_closure(p, xml_list[0].wire_protocol_version, others)

    their_msgs, their_cmds = {}, {}
    for key, x in others.items():
        if key in ours:
            continue  # a shared include (e.g. common.xml) is the same definition by construction
        for m in x.message:
            their_msgs.setdefault(m.id, (m, x))
        for e in x.enum:
            if e.name == 'MAV_CMD':
                for entry in e.entry:
                    if norm(entry.origin_file or x.filename) not in ours:
                        their_cmds.setdefault(entry.value, (entry, x))

    warnings = 0
    for x in targets:
        for m in x.message:
            hit = their_msgs.get(m.id)
            if hit and (hit[0].name != m.name or hit[0].crc_extra != m.crc_extra):
                print('%s: warning MAVGEN001: message id %u (%s, CRC_EXTRA %u) is also %s (CRC_EXTRA %u) in %s. '
                      'Use an id unique across dialects; registering both means the last registration wins.'
                      % (x.filename, m.id, m.name, m.crc_extra, hit[0].name, hit[0].crc_extra, hit[1].filename))
                warnings += 1
        for e in x.enum:
            if e.name != 'MAV_CMD':
                continue
            for entry in e.entry:
                if norm(entry.origin_file or '') != norm(x.filename):
                    continue
                hit = their_cmds.get(entry.value)
                if hit and hit[0].name != entry.name:
                    print('%s(%u): warning MAVGEN002: MAV_CMD %u (%s) is also %s in %s. '
                          'Use a command id unique across dialects; registering both means the last registration wins.'
                          % (x.filename, entry.origin_line, entry.value, entry.name, hit[0].name, hit[1].filename))
                    warnings += 1
    return warnings


def generate(directory, xml_list, emit='all', namespace_root='MavLink.Net', check_paths=None, root_namespace=None):
    '''generate split-layout C# for the root file (emit=root) or every file in the include closure (emit=all).
    root_namespace sets the namespace of the root XML file only, overriding its <namespace> element;
    included files keep theirs (or the default rule), so an extension dialect's typed fields still
    refer to the types of the packages that generated its includes.'''
    if not os.path.exists(directory):
        os.makedirs(directory)
    if root_namespace:
        xml_list[0].namespace = root_namespace

    ordered = include_order(xml_list)
    models = build_enum_models(xml_list)
    assign_owners(models, ordered, namespace_root)

    targets = [xml_list[0]] if emit == 'root' else ordered
    check_against(targets, xml_list, check_paths or [])
    for x in targets:
        generate_file(x, xml_list, models, namespace_root, directory)
