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
    'Swedish':    ('och', 'är', 'för', 'inte', 'att', 'det', 'som', 'på',
                   'med', 'vad', 'hur', 'kortet', 'läs', 'jag', 'kan', 'ska',
                   'över', 'från', 'en', 'de'),
    # 'a' is in English's list for the same reason 'en' and 'de' are in
    # Swedish's: Portuguese claims it too, and a word both languages own has to
    # sit in both or it decides the score for whichever list happens to have
    # it.
    'English':    ('the', 'and', 'is', 'what', 'how', 'does', 'are', 'of',
                   'to', 'read', 'why', 'can', 'this', 'board', 'a'),
    'German':     ('und', 'ist', 'nicht', 'das', 'der', 'die', 'was', 'wie',
                   'für', 'mit', 'ich', 'kann', 'auf', 'eine'),
    'Danish':     ('og', 'er', 'ikke', 'det', 'hvad', 'hvordan', 'jeg', 'kan',
                   'på', 'med', 'til', 'som'),
    'Norwegian':  ('og', 'er', 'ikke', 'det', 'hva', 'hvordan', 'jeg', 'kan',
                   'på', 'med', 'til', 'som'),
    'Dutch':      ('en', 'is', 'niet', 'het', 'wat', 'hoe', 'ik', 'kan',
                   'met', 'voor', 'van', 'de'),
    'French':     ('et', 'est', 'pas', 'le', 'la', 'les', 'que', 'quoi',
                   'comment', 'pour', 'avec', 'je', 'une'),
    'Spanish':    ('y', 'es', 'no', 'el', 'la', 'los', 'que', 'qué', 'cómo',
                   'para', 'con', 'una'),
    'Italian':    ('e', 'è', 'non', 'il', 'la', 'che', 'cosa', 'come',
                   'per', 'con', 'una'),
    'Finnish':    ('ja', 'on', 'ei', 'että', 'mikä', 'miten', 'kuinka',
                   'voi', 'sen', 'tämä'),
    'Polish':     ('i', 'jest', 'nie', 'co', 'jak', 'to', 'na', 'dla',
                   'czy', 'się'),
    'Portuguese': ('e', 'é', 'não', 'o', 'a', 'que', 'como', 'para',
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
    'Dutch':      ('dutch', 'nederländska'),
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
    'Thai':       ('thai', 'thailändska'),
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
    'Swedish': 'Jag är %s och är experten i det här projektet. Skriv /help.',
    'English': "I'm %s, the expert on this project. Type /help.",
    'German':  'Ich bin %s, der Experte für dieses Projekt. /help für mehr.',
    'Danish':  'Jeg er %s og eksperten i dette projekt. Skriv /help.',
    'Norwegian': 'Jeg er %s og eksperten i dette prosjektet. Skriv /help.',
    'Dutch':   'Ik ben %s, de expert in dit project. Typ /help.',
    'French':  "Je suis %s, l'expert de ce projet. Tapez /help.",
    'Spanish': 'Soy %s, el experto de este proyecto. Escribe /help.',
    'Italian': 'Sono %s, l\'esperto di questo progetto. Scrivi /help.',
    'Finnish': 'Olen %s, tämän projektin asiantuntija. Kirjoita /help.',
    'Polish':  'Jestem %s, ekspertem w tym projekcie. Wpisz /help.',
    'Portuguese': 'Sou %s, o especialista deste projeto. Escreva /help.',
    'Russian': 'Я %s, эксперт по этому проекту. Введите /help.',
    'Greek': 'Είμαι το %s, ο ειδικός σε αυτό το έργο. Πληκτρολογήστε /help.',
    'Chinese': '我是 %s，这个项目的专家。输入 /help。',
    'Japanese': '私は %s、このプロジェクトの専門家です。/help と入力してください。',
    'Korean': '저는 %s, 이 프로젝트의 전문가입니다. /help 를 입력하세요.',
    'Thai': 'ผมคือ %s ผู้เชี่ยวชาญของโปรเจกต์นี้ พิมพ์ /help',
    'Hebrew': 'אני %s, המומחה בפרויקט הזה. הקלד /help.',
    'Arabic': 'أنا %s، الخبير في هذا المشروع. اكتب /help.',
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
            'AFE AV - ADC-referensen är strömlös. Detta är koderna omvandlaren '
            'returnerade, inte mätvärden: varje kanal ligger nära mittskalan, '
            'och grader och volt nedan är räknade på den - ingen temperatur, '
            'ingen busspänning. Slå på afe_power för att mäta.',

        'link is down, not answered: %s':
            'länken är nere, obesvarad: %s',
        'no reading taken this turn - ask again.':
            'ingen avläsning gjordes denna tur - fråga igen.',
        'the reading above is all that came back - ask again.':
            'avläsningen ovan är allt som kom tillbaka - fråga igen.',
        'the last run_python/run_command call failed, nothing was done: %s':
            'det senaste run_python/run_command-anropet misslyckades, inget '
            'gjordes: %s',
        '[cut off at --words %s. Ask again with more, or ask for fewer '
        'channels.]':
            '[avklippt vid --words %s. Fråga igen med fler, eller be om färre '
            'kanaler.]',
        'budget of %d tokens is spent; /clear or raise --budget':
            'budgeten på %d tokens är förbrukad; /clear eller höj --budget',
        'board: nothing answered on %s - still on %s':
            'kort: inget svarade på %s - kvar på %s',
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
            'läst som efterfrågat, med rättelser: %s',
        'From %s:':
            'Från %s:',
        'every node on %s':
            'varje nod på %s',
        'link re-established':
            'länken återupprättad',
        ' -> check the board is powered, and that a JTAG programmer or '
        'a dedicated serial adapter is connected between it and this PC':
            ' -> kontrollera att kortet har ström, och att en JTAG-programmerare '
            'eller en seriell adapter sitter mellan kortet och den här datorn',
        'unknown channel %r; names are %s':
            'okänd kanal %r; namnen är %s',
        'channel %r could be %s - say which':
            'kanal %r kan vara %s - säg vilken',

        'this session is on a simulated board - there is no port to check. '
        '/board auto looks for a real one, debug probe first; /board COM4 '
        'tries one by name.':
            'den här sessionen kör mot ett simulerat kort - det finns ingen '
            'port att kontrollera. /board auto letar upp ett riktigt, '
            'debugproben först; /board COM4 provar en port vid namn.',
        '--no-board this run: every board tool refuses. /board auto looks '
        'for a real one.':
            '--no-board denna körning: varje kortverktyg vägrar. /board auto '
            'letar upp ett riktigt.',
        '1. Target power (ST-Link/SWD): could not check - %s':
            '1. Målspänning (ST-Link/SWD): kunde inte kontrolleras - %s',
        '1. Target power (ST-Link/SWD): %.2fV - no power sensed. Check the '
        'ST-Link USB cable is connected, and that the board itself is '
        'powered. Nothing past this point can work without it.':
            '1. Målspänning (ST-Link/SWD): %.2fV - ingen spänning känns av. '
            'Kontrollera att ST-Linkens USB-kabel sitter i och att kortet har '
            'ström. Inget efter denna punkt kan fungera utan det.',
        '1. Target power (ST-Link/SWD): %.2fV - powered, cable seated.':
            '1. Målspänning (ST-Link/SWD): %.2fV - spänning finns, kabeln '
            'sitter.',
        '2. COM ports Windows sees: %s':
            '2. COM-portar Windows ser: %s',
        "   Nothing is enumerating as a serial device - check the ST-Link or "
        "serial adapter's driver.":
            '   Ingenting räknas upp som seriell enhet - kontrollera '
            'drivrutinen för ST-Link eller seriell adapter.',
        "3. Configured port %s: not among the ports above - the cable may be "
        "unplugged from this PC's side, or the driver did not enumerate it.":
            '3. Konfigurerad port %s: finns inte bland portarna ovan - kabeln '
            'kan vara urdragen på PC-sidan, eller så räknade drivrutinen inte '
            'upp den.',
        '3. Configured port %s: present.':
            '3. Konfigurerad port %s: finns.',
        '4. Board answers on %s right now: yes - the link is up.':
            '4. Kortet svarar på %s just nu: ja - länken är uppe.',
        '4. Board answers on %s right now: no.':
            '4. Kortet svarar på %s just nu: nej.',
        '   Powered and the port is right, so check nothing else has %s open, '
        'and that the last programmer run ended with --start, not -hardRst (a '
        'halted core answers nothing).':
            '   Spänning finns och porten stämmer, så kontrollera att inget '
            'annat har %s öppen, och att den senaste programmeringen '
            'avslutades med --start, inte -hardRst (en stoppad kärna svarar '
            'inte).',
        '5. Tried every other port: %s answered as this board - it may have '
        'moved there. /reconnect after changing --port to it.':
            '5. Provade varje annan port: %s svarade som detta kort - det kan '
            'ha flyttat dit. /reconnect efter att ha ändrat --port till den.',
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
    'Danish': 'Okay', 'Norwegian': 'Greit', 'Dutch': 'Oké',
    'French': "D'accord", 'Spanish': 'De acuerdo', 'Italian': 'Va bene',
    'Finnish': 'Selvä', 'Polish': 'Dobrze', 'Portuguese': 'Está bem',
    'Russian': 'Хорошо', 'Greek': 'Εντάξει', 'Chinese': '好的',
    'Japanese': 'わかりました', 'Korean': '알겠습니다', 'Thai': 'ตกลง',
    'Hebrew': 'בסדר', 'Arabic': 'حسنًا',
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
    'svara', 'svarar', 'förklara', 'skriv', 'skriva', 'beskriv',
    'berätta', 'översätt', 'sammanfatta', 'säg',
    'answer', 'respond', 'reply', 'explain', 'write', 'describe', 'tell',
    'translate', 'summarise', 'summarize', 'say',
    'antworte', 'antworten', 'erkläre', 'schreibe',
    'reponds', 'répondre', 'explique', 'écris',
    'responde', 'explica', 'escribe', 'rispondi', 'spiega',

    'byt', 'byta', 'växla', 'tala', 'prata',
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
    'språk', 'språket', 'language', 'sprache', 'langue', 'idioma', 'lingua',
    'till', 'to', 'på', 'in', 'auf', 'en', 'a', 'nu', 'now', 'igen',
    'again', 'tillbaka', 'back', 'tack', 'please', 'snälla', 'bitte',
    'du', 'you', 'kan', 'can', 'är', 'is', 'det', 'the', 'mitt', 'ditt',
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
