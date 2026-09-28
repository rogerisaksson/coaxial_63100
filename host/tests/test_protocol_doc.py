"""PROTOCOL.md held to the handlers: every op, every field width."""
import io
import os
import re
import sys

from tools.dev.focus import chosen
from structure_kit import REPO, Report, _DISPATCH, _c_defines, _c_function, _c_reads, _c_source, _c_writes_in, _command_files, _shapes_agree


#: PROTOCOL.md's third answer to a reply's shape, held to the code. A row
#: `| N name | request | reply |` under `### N DEVICE` carries the widths in
#: backticks - `u8 on`, `u16 x3 ticks [, u32 periods]`, `i32 x5 x count`,
#: `u8 channel, bytes` - and a reply of `below` is the `Op N:` paragraph
#: after the table. Prose is prose; only the backticked spans are read, a
#: span after "per", "rows" or "of" is a body repeated an unknown number of
#: times, and a cell whose prose carries widths outside backticks is skipped
#: and named rather than half-read.
_DOC_SECTION = re.compile(r'^### (\d+) ([A-Z_]+),', re.M)


_DOC_ROW = re.compile(r'^\| (\d+) [a-z_ ]+ \| (.*?) \| (.*?) \|\s*$', re.M)


_DOC_SPAN = re.compile(r'`([^`]*)`')


_DOC_WIDTH = re.compile(r'\b([ui](?:8|16|32)|bytes|str)\b')


_DOC_REPEATED = re.compile(r'(?:\bper(?: \w+){1,2}|\brows|\bof)\s*$')


def _doc_widths(cell):
    """The widths a document cell states, or None when its prose carries
    widths the backticks do not."""
    outside = _DOC_SPAN.sub(' ', cell)
    if _DOC_WIDTH.search(outside) or 'below' in cell:
        return None
    out = []
    for m in _DOC_SPAN.finditer(cell):
        before = cell[max(0, m.start() - 24):m.start()]
        body = []
        optional = False
        for item in m.group(1).split(','):
            w = _DOC_WIDTH.search(item)
            if w is None:
                optional = optional or '[' in item
                continue
            # a bracket before the width opens the optional part here; one
            # after it opens it for the fields that follow
            opened = item.find('[')
            optional = optional or (0 <= opened < w.start())
            later = opened > w.start()
            item = item.replace('[', ' ').replace(']', ' ')
            width = w.group(1)
            if width in ('bytes', 'str'):
                body.append(('rest',))
                continue
            times = re.search(r'\bx(\d+)\b', item)
            counted = re.search(r'\bx count\b|\bx n\b', item)
            fields = [('?', width) if optional else width] * (int(times.group(1)) if times else 1)
            if counted:
                body.append(('*', tuple(fields)))
            else:
                body.extend(fields)
            optional = optional or later
        if _DOC_REPEATED.search(before) and body:
            out.append(('*', tuple(body)))
        else:
            out.extend(body)
    return out


def _doc_ops():
    """{(PREFIX, op number): (request widths, reply widths)} off PROTOCOL.md,
    None for a cell this reader cannot take whole."""
    text = io.open(os.path.join(REPO, 'docs', 'PROTOCOL.md'), encoding='utf-8').read()
    found = {}
    sections = list(_DOC_SECTION.finditer(text))
    for k, sec in enumerate(sections):
        end = sections[k + 1].start() if k + 1 < len(sections) else len(text)
        chunk = text[sec.start():end]
        prefix = sec.group(2).replace('_', '')
        paragraphs = {int(m.group(1)): m.group(2) for m in re.finditer(
            r'^Op (\d+): (.*?)(?:\n\n|\Z)', chunk, re.M | re.S)}
        for m in _DOC_ROW.finditer(chunk):
            op = int(m.group(1))
            request = [] if m.group(2).strip() == '-' else _doc_widths(m.group(2))
            reply_cell = m.group(3)
            if 'below' in reply_cell:
                reply = _doc_widths(paragraphs[op]) if op in paragraphs else None
            else:
                reply = [] if reply_cell.strip() == '-' else _doc_widths(reply_cell)
            found[(prefix, op)] = (request, reply)
    return found


def _c_ops():
    """{(PREFIX, op number): [(reads, writes), ...]} off the dispatch
    tables.
    """
    found = {}
    for rel in _command_files():
        own = re.sub(r'/\*.*?\*/', ' ',
                     io.open(os.path.join(REPO, *rel.split('/')), encoding='utf-8').read(), flags=re.S)
        text = _c_source(rel)
        defines = _c_defines(text)
        for prefix, op, handler, first in _DISPATCH.findall(own):
            number = defines.get('%s_OP_%s' % (prefix, op))
            if number is None:
                continue
            reads = _c_reads(text, handler, defines) if first == 'in' else []
            body = _c_function(text, handler)
            writes = _c_writes_in(text, body, defines) if body is not None else []
            if body is None:
                # a handler reading and writing: the writes parser wants `out`
                # first, so read its writes off the `in, out` form
                m = re.search(r'^static\s+\w+\s+%s\s*\(\s*rd_t\s*\*\s*in\s*,\s*wr_t\s*\*\s*out'
                              % re.escape(handler), text, re.M)
                if m is not None:
                    writes = _c_writes_in(text, text[text.index('{', m.end()):], defines)
            found.setdefault((prefix, number), []).append((reads, writes))
    return found


def _same_shape(code, doc):
    """A document's widths against the code's, either side repeating."""
    return _shapes_agree(list(code), list(doc))[0]


def test_protocol_agrees(r):
    """PROTOCOL.md's op tables say what the handlers read and write."""
    doc = _doc_ops()
    code = _c_ops()
    for prefix in sorted({p for p, _ in doc if any(q == p for q, _ in code)}):
        wrong, compared, skipped = [], 0, []
        for (p, op), (request, reply) in sorted(doc.items()):
            if p != prefix or (p, op) not in code:
                continue
            for reads, writes in code[(p, op)]:
                for side, stated, actual in (('request', request, reads), ('reply', reply, writes)):
                    if stated is None:
                        skipped.append('%d %s' % (op, side))
                        continue
                    compared += 1
                    if not _same_shape(actual, stated):
                        wrong.append('op %d %s: C %s, document %s' % (
                            op, side, ' '.join(str(w) for w in actual) or 'nothing',
                            ' '.join(str(w) for w in stated) or 'nothing'))
        r.check("PROTOCOL.md's %s table says what the handlers read and write" % prefix,
                not wrong, '; '.join(wrong[:3]) or '%d cells%s' % (
                    compared, (', prose: ' + ', '.join(skipped)) if skipped else ''))


ROSTER = (test_protocol_agrees,)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
