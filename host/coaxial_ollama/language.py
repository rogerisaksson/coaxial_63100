"""Which language a question is in, decided here rather than by the model."""
import ctypes
import locale
import re
import unicodedata
from contextlib import suppress

# Ranges that settle it without counting anything.
SCRIPTS = (
    ('Japanese', ((0x3040, 0x309F), (0x30A0, 0x30FF))),      # hiragana, katakana
    ('Korean',   ((0xAC00, 0xD7AF), (0x1100, 0x11FF))),
    ('Thai',     ((0x0E00, 0x0E7F),)),
    ('Greek',    ((0x0370, 0x03FF),)),
    ('Hebrew',   ((0x0590, 0x05FF),)),
    ('Arabic',   ((0x0600, 0x06FF),)),
    ('Chinese',  ((0x4E00, 0x9FFF), (0x3400, 0x4DBF))),      # Han, after kana
    ('Russian',  ((0x0400, 0x04FF),)),                       # Cyrillic
)

# Words common enough to appear in one sentence and rare enough elsewhere.
STOPWORDS = {
    'Swedish':    ('och', '\u00e4r', 'f\u00f6r', 'inte', 'att', 'det', 'som', 'p\u00e5',
                   'med', 'vad', 'hur', 'kortet', 'l\u00e4s', 'jag', 'kan', 'ska',
                   '\u00f6ver', 'fr\u00e5n', 'en', 'de'),
    # 'a' is in English's list for the same reason 'en' and 'de' are in
    # Swedish's: Portuguese claims it too, and a word both languages own has to
    # sit in both or it decides the score for whichever list happens to have
    # it.
    'English':    ('the', 'and', 'is', 'what', 'how', 'does', 'are', 'of',
                   'to', 'read', 'why', 'can', 'this', 'board', 'a'),
    'German':     ('und', 'ist', 'nicht', 'das', 'der', 'die', 'was', 'wie',
                   'f\u00fcr', 'mit', 'ich', 'kann', 'auf', 'eine'),
    'Danish':     ('og', 'er', 'ikke', 'det', 'hvad', 'hvordan', 'jeg', 'kan',
                   'p\u00e5', 'med', 'til', 'som'),
    'Norwegian':  ('og', 'er', 'ikke', 'det', 'hva', 'hvordan', 'jeg', 'kan',
                   'p\u00e5', 'med', 'til', 'som'),
    'Dutch':      ('en', 'is', 'niet', 'het', 'wat', 'hoe', 'ik', 'kan',
                   'met', 'voor', 'van', 'de'),
    'French':     ('et', 'est', 'pas', 'le', 'la', 'les', 'que', 'quoi',
                   'comment', 'pour', 'avec', 'je', 'une'),
    'Spanish':    ('y', 'es', 'no', 'el', 'la', 'los', 'que', 'qu\u00e9', 'c\u00f3mo',
                   'para', 'con', 'una'),
    'Italian':    ('e', '\u00e8', 'non', 'il', 'la', 'che', 'cosa', 'come',
                   'per', 'con', 'una'),
    'Finnish':    ('ja', 'on', 'ei', 'ett\u00e4', 'mik\u00e4', 'miten', 'kuinka',
                   'voi', 'sen', 't\u00e4m\u00e4'),
    'Polish':     ('i', 'jest', 'nie', 'co', 'jak', 'to', 'na', 'dla',
                   'czy', 'si\u0119'),
    'Portuguese': ('e', '\u00e9', 'n\u00e3o', 'o', 'a', 'que', 'como', 'para',
                   'com', 'uma', 'qual'),
}

# How far ahead the winner has to be: strictly ahead, no more.
MARGIN = 1

WORD = re.compile(r"[^\W\d_]+", re.UNICODE)


def _script(text):
    counts = {}
    for character in text:
        point = ord(character)
        for name, ranges in SCRIPTS:
            if any(low <= point <= high for low, high in ranges):
                counts[name] = counts.get(name, 0) + 1
                break
    if not counts:
        return None
    best = max(counts, key=lambda k: counts[k])
    # A stray CJK quotation mark in an English sentence is not Chinese.
    letters = sum(1 for c in text if unicodedata.category(c).startswith('L'))
    if letters and counts[best] * 4 >= letters:
        return best
    return None


def detect(text):
    """The language of `text` as an English name, or None when unsure."""
    text = (text or '').strip()
    if not text:
        return None

    script = _script(text)
    if script:
        return script

    words = [w.lower() for w in WORD.findall(text)]
    if not words:
        return None

    scores = {}
    for name, markers in STOPWORDS.items():
        hit = sum(1 for w in words if w in markers)
        if hit:
            scores[name] = hit
    if not scores:
        return None

    ranked = sorted(scores.items(), key=lambda kv: kv[1], reverse=True)
    if len(ranked) > 1 and ranked[0][1] - ranked[1][1] < MARGIN:
        return None
    return ranked[0][0]


def instruction_for(name):
    """The line to append to the system prompt, for an already-known
    language name (or None to fall back to mirroring the question) - the
    part `instruction()` and a session's locked language both need, kept
    in one place so the wording never drifts between the two callers.
    """
    if not name:
        return ('Answer in the language the question was asked in. Channel '
                'names, units and register names stay exactly as the board '
                'prints them.')
    return ('The session language is %s. Answer in %s and in no other '
            'language, whatever language the tool output and the documents '
            'are in - unless the operator asks for another, which is the '
            'one thing that overrides this. Channel names, units and '
            'register names stay exactly as the board prints them.'
            % (name, name))


def instruction(text):
    """The line to append to the system prompt for this one question, with
    the language detected fresh from `text` - the one-shot case, where
    there is no session to have locked anything in the first place."""
    return instruction_for(detect(text))


# Language names as they would actually appear when someone asks for one by
# name - the English word and, for the two this loop is mostly spoken in, the
# Swedish word too.
LANGUAGE_NAMES = {
    'Swedish':    ('swedish', 'svenska'),
    'English':    ('english', 'engelska'),
    'German':     ('german', 'tyska'),
    'Danish':     ('danish', 'danska'),
    'Norwegian':  ('norwegian', 'norska'),
    'Dutch':      ('dutch', 'nederl\u00e4ndska'),
    'French':     ('french', 'franska'),
    'Spanish':    ('spanish', 'spanska'),
    'Italian':    ('italian', 'italienska'),
    'Finnish':    ('finnish', 'finska'),
    'Polish':     ('polish', 'polska'),
    'Portuguese': ('portuguese', 'portugisiska'),
    'Russian':    ('russian', 'ryska'),
    'Chinese':    ('chinese', 'kinesiska'),
    'Japanese':   ('japanese', 'japanska'),
    'Korean':     ('korean', 'koreanska'),
    'Thai':       ('thai', 'thail\u00e4ndska'),
    'Greek':      ('greek', 'grekiska'),
    'Hebrew':     ('hebrew', 'hebreiska'),
    'Arabic':     ('arabic', 'arabiska'),
}
_NAME_TO_LANGUAGE = {alias: name for name, aliases in LANGUAGE_NAMES.items()
                     for alias in aliases}

# ISO code -> the name above, for every language this module can name.
_LOCALE_CODES = {
    'sv': 'Swedish', 'en': 'English', 'de': 'German', 'da': 'Danish',
    'nb': 'Norwegian', 'nn': 'Norwegian', 'no': 'Norwegian', 'nl': 'Dutch',
    'fr': 'French', 'es': 'Spanish', 'it': 'Italian', 'fi': 'Finnish',
    'pl': 'Polish', 'pt': 'Portuguese', 'ru': 'Russian', 'el': 'Greek',
    'zh': 'Chinese', 'ja': 'Japanese', 'ko': 'Korean', 'th': 'Thai',
    'he': 'Hebrew', 'iw': 'Hebrew', 'ar': 'Arabic',   # iw: the old code for he
}


def system_language(default='English'):
    """The language this host is set up in."""
    candidates = []
    with suppress(ValueError, TypeError):
        candidates.append(locale.getlocale()[0] or '')
    with suppress(Exception):
        buffer = ctypes.create_unicode_buffer(85)
        if ctypes.windll.kernel32.GetUserDefaultLocaleName(buffer, 85):
            candidates.append(buffer.value)
    for text in candidates:
        low = str(text).lower()
        code = re.split(r'[-_]', low)[0]
        if code in _LOCALE_CODES:
            return _LOCALE_CODES[code]
        for name, aliases in LANGUAGE_NAMES.items():
            if low.startswith(aliases[0]):
                return name
    return default


# One line, in the operator's own language: who is answering, and where the
# rest is.
GREETINGS = {
    'Swedish': 'Jag \u00e4r %s och \u00e4r experten i det h\u00e4r projektet. Skriv /help.',
    'English': "I'm %s, the expert on this project. Type /help.",
    'German':  'Ich bin %s, der Experte f\u00fcr dieses Projekt. /help f\u00fcr mehr.',
    'Danish':  'Jeg er %s og eksperten i dette projekt. Skriv /help.',
    'Norwegian': 'Jeg er %s og eksperten i dette prosjektet. Skriv /help.',
    'Dutch':   'Ik ben %s, de expert in dit project. Typ /help.',
    'French':  "Je suis %s, l'expert de ce projet. Tapez /help.",
    'Spanish': 'Soy %s, el experto de este proyecto. Escribe /help.',
    'Italian': 'Sono %s, l\'esperto di questo progetto. Scrivi /help.',
    'Finnish': 'Olen %s, t\u00e4m\u00e4n projektin asiantuntija. Kirjoita /help.',
    'Polish':  'Jestem %s, ekspertem w tym projekcie. Wpisz /help.',
    'Portuguese': 'Sou %s, o especialista deste projeto. Escreva /help.',
    'Russian': '\u042f %s, \u044d\u043a\u0441\u043f\u0435\u0440\u0442 \u043f\u043e \u044d\u0442\u043e\u043c\u0443 \u043f\u0440\u043e\u0435\u043a\u0442\u0443. \u0412\u0432\u0435\u0434\u0438\u0442\u0435 /help.',
    'Greek': '\u0395\u03af\u03bc\u03b1\u03b9 \u03c4\u03bf %s, \u03bf \u03b5\u03b9\u03b4\u03b9\u03ba\u03cc\u03c2 \u03c3\u03b5 \u03b1\u03c5\u03c4\u03cc \u03c4\u03bf \u03ad\u03c1\u03b3\u03bf. \u03a0\u03bb\u03b7\u03ba\u03c4\u03c1\u03bf\u03bb\u03bf\u03b3\u03ae\u03c3\u03c4\u03b5 /help.',
    'Chinese': '\u6211\u662f %s\uff0c\u8fd9\u4e2a\u9879\u76ee\u7684\u4e13\u5bb6\u3002\u8f93\u5165 /help\u3002',
    'Japanese': '\u79c1\u306f %s\u3001\u3053\u306e\u30d7\u30ed\u30b8\u30a7\u30af\u30c8\u306e\u5c02\u9580\u5bb6\u3067\u3059\u3002/help \u3068\u5165\u529b\u3057\u3066\u304f\u3060\u3055\u3044\u3002',
    'Korean': '\uc800\ub294 %s, \uc774 \ud504\ub85c\uc81d\ud2b8\uc758 \uc804\ubb38\uac00\uc785\ub2c8\ub2e4. /help \ub97c \uc785\ub825\ud558\uc138\uc694.',
    'Thai': '\u0e1c\u0e21\u0e04\u0e37\u0e2d %s \u0e1c\u0e39\u0e49\u0e40\u0e0a\u0e35\u0e48\u0e22\u0e27\u0e0a\u0e32\u0e0d\u0e02\u0e2d\u0e07\u0e42\u0e1b\u0e23\u0e40\u0e08\u0e01\u0e15\u0e4c\u0e19\u0e35\u0e49 \u0e1e\u0e34\u0e21\u0e1e\u0e4c /help',
    'Hebrew': '\u05d0\u05e0\u05d9 %s, \u05d4\u05de\u05d5\u05de\u05d7\u05d4 \u05d1\u05e4\u05e8\u05d5\u05d9\u05e7\u05d8 \u05d4\u05d6\u05d4. \u05d4\u05e7\u05dc\u05d3 /help.',
    'Arabic': '\u0623\u0646\u0627 %s\u060c \u0627\u0644\u062e\u0628\u064a\u0631 \u0641\u064a \u0647\u0630\u0627 \u0627\u0644\u0645\u0634\u0631\u0648\u0639. \u0627\u0643\u062a\u0628 /help.',
}


# Host-authored text that reaches the screen, keyed by the English it is
# written as at the call site.
PHRASES = {
    'Swedish': {
        'AFE OFF - the ADC reference is unpowered. These are the codes '
        'the converter returned, not measurements: every channel sits '
        'near mid-scale, and the degC and volts below are arithmetic on '
        'that - not a temperature, not a bus voltage. Call afe_power on to '
        'measure.':
            'AFE AV - ADC-referensen \u00e4r str\u00f6ml\u00f6s. Detta \u00e4r koderna omvandlaren '
            'returnerade, inte m\u00e4tv\u00e4rden: varje kanal ligger n\u00e4ra mittskalan, '
            'och grader och volt nedan \u00e4r r\u00e4knade p\u00e5 den - ingen temperatur, '
            'ingen bussp\u00e4nning. Sl\u00e5 p\u00e5 afe_power f\u00f6r att m\u00e4ta.',

        'link is down, not answered: %s':
            'l\u00e4nken \u00e4r nere, obesvarad: %s',
        'no reading taken this turn - ask again.':
            'ingen avl\u00e4sning gjordes denna tur - fr\u00e5ga igen.',
        'the reading above is all that came back - ask again.':
            'avl\u00e4sningen ovan \u00e4r allt som kom tillbaka - fr\u00e5ga igen.',
        'the last run_python/run_command call failed, nothing was done: %s':
            'det senaste run_python/run_command-anropet misslyckades, inget '
            'gjordes: %s',
        '[cut off at --words %s. Ask again with more, or ask for fewer '
        'channels.]':
            '[avklippt vid --words %s. Fr\u00e5ga igen med fler, eller be om f\u00e4rre '
            'kanaler.]',
        'budget of %d tokens is spent; /clear or raise --budget':
            'budgeten p\u00e5 %d tokens \u00e4r f\u00f6rbrukad; /clear eller h\u00f6j --budget',
        'board: nothing answered on %s - still on %s':
            'kort: inget svarade p\u00e5 %s - kvar p\u00e5 %s',
        'board: %s':
            'kort: %s',
        # The block headings, not the column names: the columns stay as the
        # board prints them, same rule as a channel name.
        'analog: %d channels':
            'analog: %d kanaler',
        'analog: %d channel':
            'analog: %d kanal',
        'digital: %d channels':
            'digital: %d kanaler',
        'digital: %d channel':
            'digital: %d kanal',
        'reserved: %d pins':
            'reserverade: %d pinnar',
        '%d samples @%.0fHz':
            '%d sampel @%.0fHz',
        'read as asked, with corrections: %s':
            'l\u00e4st som efterfr\u00e5gat, med r\u00e4ttelser: %s',
        'From %s:':
            'Fr\u00e5n %s:',
        'every node on %s':
            'varje nod p\u00e5 %s',
        'link re-established':
            'l\u00e4nken \u00e5teruppr\u00e4ttad',
        ' -> check the board is powered, and that a JTAG programmer or '
        'a dedicated serial adapter is connected between it and this PC':
            ' -> kontrollera att kortet har str\u00f6m, och att en JTAG-programmerare '
            'eller en seriell adapter sitter mellan kortet och den h\u00e4r datorn',
        'unknown channel %r; names are %s':
            'ok\u00e4nd kanal %r; namnen \u00e4r %s',
        'channel %r could be %s - say which':
            'kanal %r kan vara %s - s\u00e4g vilken',

        'this session is on a simulated board - there is no port to check. '
        '/board auto looks for a real one, debug probe first; /board COM4 '
        'tries one by name.':
            'den h\u00e4r sessionen k\u00f6r mot ett simulerat kort - det finns ingen '
            'port att kontrollera. /board auto letar upp ett riktigt, '
            'debugproben f\u00f6rst; /board COM4 provar en port vid namn.',
        '--no-board this run: every board tool refuses. /board auto looks '
        'for a real one.':
            '--no-board denna k\u00f6rning: varje kortverktyg v\u00e4grar. /board auto '
            'letar upp ett riktigt.',
        '1. Target power (ST-Link/SWD): could not check - %s':
            '1. M\u00e5lsp\u00e4nning (ST-Link/SWD): kunde inte kontrolleras - %s',
        '1. Target power (ST-Link/SWD): %.2fV - no power sensed. Check the '
        'ST-Link USB cable is connected, and that the board itself is '
        'powered. Nothing past this point can work without it.':
            '1. M\u00e5lsp\u00e4nning (ST-Link/SWD): %.2fV - ingen sp\u00e4nning k\u00e4nns av. '
            'Kontrollera att ST-Linkens USB-kabel sitter i och att kortet har '
            'str\u00f6m. Inget efter denna punkt kan fungera utan det.',
        '1. Target power (ST-Link/SWD): %.2fV - powered, cable seated.':
            '1. M\u00e5lsp\u00e4nning (ST-Link/SWD): %.2fV - sp\u00e4nning finns, kabeln '
            'sitter.',
        '2. COM ports Windows sees: %s':
            '2. COM-portar Windows ser: %s',
        "   Nothing is enumerating as a serial device - check the ST-Link or "
        "serial adapter's driver.":
            '   Ingenting r\u00e4knas upp som seriell enhet - kontrollera '
            'drivrutinen f\u00f6r ST-Link eller seriell adapter.',
        "3. Configured port %s: not among the ports above - the cable may be "
        "unplugged from this PC's side, or the driver did not enumerate it.":
            '3. Konfigurerad port %s: finns inte bland portarna ovan - kabeln '
            'kan vara urdragen p\u00e5 PC-sidan, eller s\u00e5 r\u00e4knade drivrutinen inte '
            'upp den.',
        '3. Configured port %s: present.':
            '3. Konfigurerad port %s: finns.',
        '4. Board answers on %s right now: yes - the link is up.':
            '4. Kortet svarar p\u00e5 %s just nu: ja - l\u00e4nken \u00e4r uppe.',
        '4. Board answers on %s right now: no.':
            '4. Kortet svarar p\u00e5 %s just nu: nej.',
        '   Powered and the port is right, so check nothing else has %s open, '
        'and that the last programmer run ended with --start, not -hardRst (a '
        'halted core answers nothing).':
            '   Sp\u00e4nning finns och porten st\u00e4mmer, s\u00e5 kontrollera att inget '
            'annat har %s \u00f6ppen, och att den senaste programmeringen '
            'avslutades med --start, inte -hardRst (en stoppad k\u00e4rna svarar '
            'inte).',
        '5. Tried every other port: %s answered as this board - it may have '
        'moved there. /reconnect after changing --port to it.':
            '5. Provade varje annan port: %s svarade som detta kort - det kan '
            'ha flyttat dit. /reconnect efter att ha \u00e4ndrat --port till den.',
        '5. Tried every other port (%s): none answered.':
            '5. Provade varje annan port (%s): ingen svarade.',
    },
}

# A %-spec in one of those templates.
_SPEC = re.compile(r'%(?:\.\d+)?[a-z]')


def _matcher(template):
    parts = [re.escape(p) for p in _SPEC.split(template)]
    return re.compile('(.+?)'.join(parts))


_MATCHERS = {}


def localise(text, name=None):
    """Host-authored English in `text`, replaced with `name`'s version."""
    table = PHRASES.get(name or '')
    if not table or not text:
        return text
    for english in sorted(table, key=len, reverse=True):
        matcher = _MATCHERS.get(english)
        if matcher is None:
            matcher = _MATCHERS[english] = _matcher(english)
        translated = table[english]
        text = matcher.sub(
            lambda m: _fill(translated, m.groups()), text)
    return text


def _fill(translated, values):
    """The translation with the captured values back in its own %-slots."""
    parts = _SPEC.split(translated)
    out = parts[0]
    for value, part in zip(values, parts[1:]):
        out += value + part
    return out


def greeting(model, name=None, encoding=None):
    """The one line a session opens with, in `name` or this host's own
    language.
    """
    name = name or system_language()
    text = (GREETINGS.get(name) or GREETINGS['English']) % model
    if encoding:
        try:
            text.encode(encoding)
        except (UnicodeEncodeError, LookupError):
            return GREETINGS['English'] % model
    return text

# One word, in the language just asked for.
OKAY = {
    'Swedish': 'Okej', 'English': 'Okay', 'German': 'In Ordnung',
    'Danish': 'Okay', 'Norwegian': 'Greit', 'Dutch': 'Ok\u00e9',
    'French': "D'accord", 'Spanish': 'De acuerdo', 'Italian': 'Va bene',
    'Finnish': 'Selv\u00e4', 'Polish': 'Dobrze', 'Portuguese': 'Est\u00e1 bem',
    'Russian': '\u0425\u043e\u0440\u043e\u0448\u043e', 'Greek': '\u0395\u03bd\u03c4\u03ac\u03be\u03b5\u03b9', 'Chinese': '\u597d\u7684',
    'Japanese': '\u308f\u304b\u308a\u307e\u3057\u305f', 'Korean': '\uc54c\uaca0\uc2b5\ub2c8\ub2e4', 'Thai': '\u0e15\u0e01\u0e25\u0e07',
    'Hebrew': '\u05d1\u05e1\u05d3\u05e8', 'Arabic': '\u062d\u0633\u0646\u064b\u0627',
}


def okay(name, encoding=None):
    """The acknowledgement for a bare language switch."""
    text = OKAY.get(name or '') or OKAY['English']
    if encoding:
        try:
            text.encode(encoding)
        except (UnicodeEncodeError, LookupError):
            return OKAY['English']
    return text


# A language's own name has to sit next to one of these to count as a request
# rather than a mention - "the German firmware bug" is not a request for
# German, and this is what keeps it from reading as one.
_REQUEST_VERBS = (
    'svara', 'svarar', 'f\u00f6rklara', 'skriv', 'skriva', 'beskriv',
    'ber\u00e4tta', '\u00f6vers\u00e4tt', 'sammanfatta', 's\u00e4g',
    'answer', 'respond', 'reply', 'explain', 'write', 'describe', 'tell',
    'translate', 'summarise', 'summarize', 'say',
    'antworte', 'antworten', 'erkl\u00e4re', 'schreibe',
    'reponds', 'r\u00e9pondre', 'explique', '\u00e9cris',
    'responde', 'explica', 'escribe', 'rispondi', 'spiega',

    'byt', 'byta', 'v\u00e4xla', 'tala', 'prata',
    'switch', 'change', 'speak', 'wechsle', 'changer', 'cambia',
)


def requested_language(text):
    """A language named outright in `text` - "svara pa engelska", "byt sprak
    till svenska" - independent of what language `text` itself is written
    in.
    """
    words = [w.lower() for w in WORD.findall(text or '')]
    named = None
    for word in words:
        if word in _NAME_TO_LANGUAGE:
            named = _NAME_TO_LANGUAGE[word]
            break
    if named is None:
        return None
    if any(w in _REQUEST_VERBS for w in words):
        return named
    return named if detect(text) is None else None


# What can sit around a language name without the message being about anything
# else: the switch itself, the word "language", and the politeness.
_SWITCH_FILLER = (
    'spr\u00e5k', 'spr\u00e5ket', 'language', 'sprache', 'langue', 'idioma', 'lingua',
    'till', 'to', 'p\u00e5', 'in', 'auf', 'en', 'a', 'nu', 'now', 'igen',
    'again', 'tillbaka', 'back', 'tack', 'please', 'sn\u00e4lla', 'bitte',
    'du', 'you', 'kan', 'can', '\u00e4r', 'is', 'det', 'the', 'mitt', 'ditt',
)


def bare_switch(text):
    """The language `text` asks for, when it asks for nothing else."""
    named = requested_language(text)
    if not named:
        return None
    for word in (w.lower() for w in WORD.findall(text or '')):
        if not (word in _NAME_TO_LANGUAGE or word in _REQUEST_VERBS
                or word in _SWITCH_FILLER):
            return None
    return named
