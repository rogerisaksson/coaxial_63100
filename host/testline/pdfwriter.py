"""A minimal PDF writer. No dependencies, on purpose.

A production line at a contract manufacturer runs on whatever PC is bolted to
the bench. Asking that machine to pip-install reportlab, and to keep it working
through the next OS image, is a support burden that buys nothing here: a test
report is text, rules and tables, and PDF renders all three from the base
fourteen fonts every viewer already has.

So this module emits PDF 1.4 directly. It supports what a report needs and
nothing more: Helvetica in three weights, Courier for data, filled rectangles,
lines, and left/right/centre aligned text.
"""

FONTS = {
    'regular': 'Helvetica',
    'bold': 'Helvetica-Bold',
    'oblique': 'Helvetica-Oblique',
    'mono': 'Courier',
    'mono-bold': 'Courier-Bold',
}

# A4 because an EMS anywhere outside the US will print on it, and a US shop can
# still read it.
A4 = (595.28, 841.89)

_W = {' ': .278, '!': .278, '"': .355, '#': .556, '$': .556, '%': .889,
      '&': .667, "'": .191, '(': .333, ')': .333, '*': .389, '+': .584,
      ',': .278, '-': .333, '.': .278, '/': .278, ':': .278, ';': .278,
      '<': .584, '=': .584, '>': .584, '?': .556, '@': 1.015, '[': .278,
      ']': .278, '^': .469, '_': .556, '`': .333, '{': .334,
      '|': .260, '}': .334, '~': .584}
_W[chr(92)] = .278
for _c in '0123456789abcdefghijklmnopqrstuvwxyz':
    _W[_c] = .556
for _c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ':
    _W[_c] = .667
for _c, _v in zip('ijlftrIJ', (.222, .222, .222, .278, .278, .333, .278, .500)):
    _W[_c] = _v
for _c in 'mw':
    _W[_c] = .833
for _c in 'MW':
    _W[_c] = .889


def text_width(text, size, font='regular'):
    """Approximate width in points. Courier is monospaced at 0.6 em."""
    if font.startswith('mono'):
        return len(text) * 0.6 * size
    return sum(_W.get(ch, .556) for ch in str(text)) * size


def _escape(text):
    """Escape for a PDF literal string: parentheses and the backslash."""
    special = "()" + chr(92)
    out = []
    for ch in str(text):
        if ch in special:
            out.append(chr(92) + ch)
        elif ord(ch) < 32:
            out.append(' ')
        elif ord(ch) < 256:
            out.append(ch)
        else:
            out.append('?')          # outside Latin-1; a report is ASCII anyway
    return ''.join(out)


class Page:
    """One page's content stream, built up in drawing order."""

    def __init__(self, size=A4):
        self.width, self.height = size
        self.ops = []

    def text(self, x, y, value, size=9, font='regular', align='left', gray=0.0):
        value = str(value)
        if align == 'right':
            x -= text_width(value, size, font)
        elif align == 'center':
            x -= text_width(value, size, font) / 2.0
        self.ops.append('BT /%s %.2f Tf %.3f g %.2f %.2f Td (%s) Tj ET'
                        % (font, size, gray, x, y, _escape(value)))
        return self

    def line(self, x1, y1, x2, y2, width=0.5, gray=0.0):
        self.ops.append('%.2f w %.3f G %.2f %.2f m %.2f %.2f l S'
                        % (width, gray, x1, y1, x2, y2))
        return self

    def rect(self, x, y, w, h, gray=0.9):
        self.ops.append('%.3f g %.2f %.2f %.2f %.2f re f' % (gray, x, y, w, h))
        return self

    def stream(self):
        return '\n'.join(self.ops).encode('latin-1', 'replace')


class Document:
    """A PDF being built: a title, an author and the pages in order. Held in
    memory until build() numbers the objects and serialises the lot."""

    def __init__(self, title='Test report', author='testline'):
        self.title = title
        self.author = author
        self.pages = []

    def page(self, size=A4):
        page = Page(size)
        self.pages.append(page)
        return page

    def build(self):
        """Serialise to PDF bytes. Objects are numbered as they are emitted."""
        objects = []

        def add(body):
            objects.append(body)
            return len(objects)          # 1-based object number

        font_ids = {}
        for alias, base in FONTS.items():
            font_ids[alias] = add(
                b'<< /Type /Font /Subtype /Type1 /BaseFont /%s '
                b'/Encoding /WinAnsiEncoding >>' % base.encode())

        resources = b'<< /Font << ' + b' '.join(
            b'/%s %d 0 R' % (alias.encode(), num)
            for alias, num in font_ids.items()) + b' >> >>'

        # Page objects need a /Parent reference to the page tree, which is
        # written after them. Predict its id and assert the prediction below
        # rather than emit a file that opens in some viewers and not others.
        pages_id = len(objects) + 1 + 2 * len(self.pages)

        page_ids = []
        for page in self.pages:
            data = page.stream()
            content_id = add(b'<< /Length %d >>\nstream\n%s\nendstream'
                             % (len(data), data))
            page_ids.append(add(
                b'<< /Type /Page /Parent %d 0 R /MediaBox [0 0 %.2f %.2f] '
                b'/Resources %s /Contents %d 0 R >>'
                % (pages_id, page.width, page.height, resources, content_id)))

        kids = b' '.join(b'%d 0 R' % pid for pid in page_ids)
        actual = add(b'<< /Type /Pages /Kids [%s] /Count %d >>'
                     % (kids, len(page_ids)))
        assert actual == pages_id, ('page tree id moved: predicted %d, got %d'
                                    % (pages_id, actual))

        catalog_id = add(b'<< /Type /Catalog /Pages %d 0 R >>' % actual)
        info_id = add(b'<< /Title (%s) /Producer (%s) >>'
                      % (_escape(self.title).encode('latin-1', 'replace'),
                         _escape(self.author).encode('latin-1', 'replace')))

        out = bytearray(b'%PDF-1.4\n%\xe2\xe3\xcf\xd3\n')
        offsets = []
        for number, body in enumerate(objects, start=1):
            offsets.append(len(out))
            out += b'%d 0 obj\n' % number + body + b'\nendobj\n'

        xref_at = len(out)
        out += b'xref\n0 %d\n' % (len(objects) + 1)
        out += b'0000000000 65535 f \n'
        for offset in offsets:
            out += b'%010d 00000 n \n' % offset
        out += (b'trailer\n<< /Size %d /Root %d 0 R /Info %d 0 R >>\n'
                b'startxref\n%d\n%%%%EOF\n'
                % (len(objects) + 1, catalog_id, info_id, xref_at))
        return bytes(out)

    def save(self, path):
        with open(path, 'wb') as handle:
            handle.write(self.build())
        return path
