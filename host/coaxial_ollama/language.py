"""Which language a question is in, decided here rather than by the model.

Asking a model to "answer in the language of the question" makes it do two
things at once: work out what language that was, and then answer. The first is
where it drifts. Reported from this bench with `qwen2.5:14b` - a model whose
training leans heavily Chinese - answers came back in Chinese, Japanese and
Thai to questions that were in none of them. A larger model gets it right more
often, which is not the same as getting it right.

So the host decides and the prompt says it plainly: *The user writes in
Swedish. Answer in Swedish.* That turns an introspection into an instruction,
and it is the same trick as everywhere else in this package - do the part a
program is reliable at in the program, and leave the model the part that needs
a model.

The detection is deliberately small. It has to separate the languages actually
spoken at a bench, not every language there is, and a wrong guess must degrade
to the old behaviour rather than to a confident instruction in the wrong
language. Two stages:

  * Script. Chinese, Japanese, Korean, Thai, Greek, Cyrillic, Hebrew and Arabic
    are decided by the characters alone and cannot be confused with each other.
  * Stop words, for the Latin ones. Counting `och`, `är`, `för` against `the`,
    `and`, `is` separates Swedish from English in one short sentence, which is
    the length a bench question actually has.

Below a margin, this says nothing and the prompt falls back to "answer in the
language the question was asked in". An unsure detector that guesses is worse
than one that abstains: the model mirroring the question is right most of the
time, and a wrong instruction is right none of it.
"""
import re
import unicodedata

# Ranges that settle it without counting anything. Order matters only in that
# Japanese kana are checked before Han: a Japanese sentence contains both.
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
# Kept short on purpose: a longer list is not more accurate on bench-length
# questions, and every entry is a chance to collide with another language.
#
# 'en' and 'de' are in Swedish's own list even though Dutch also claims them,
# and that repetition is the point, not an oversight. Measured on this bench:
# "ger du mig en tabell over de analoga matvardena?" has only one word from
# the rest of the Swedish list ('over') against two from Dutch's ('en', 'de'),
# so Dutch outscored Swedish outright and the model answered in a Dutch/
# Norwegian mix. A word missing from Swedish's list does not make a sentence
# less Swedish - it just leaves Swedish's score lower than it should be
# whenever that word is the one doing the work. Adding the same word to both
# lists cancels it as a discriminator rather than leaving it to favour
# whichever list happened to claim it first.
STOPWORDS = {
    'Swedish':    ('och', 'är', 'för', 'inte', 'att', 'det', 'som', 'på',
                   'med', 'vad', 'hur', 'kortet', 'läs', 'jag', 'kan', 'ska',
                   'över', 'från', 'en', 'de'),
    'English':    ('the', 'and', 'is', 'what', 'how', 'does', 'are', 'of',
                   'to', 'read', 'why', 'can', 'this', 'board'),
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

# How far ahead the winner has to be: strictly ahead, no more. A bench question
# is one short sentence, so the winning margin is often a single word - "Vad ar
# en NTC-termistor?" scores Swedish 2 against Dutch 1, because `en` is a Dutch
# word too. Demanding two would abstain on most real questions.
#
# What this still catches is the tie, which is the case the margin exists for:
# Danish and Norwegian share almost all of this list and score identically, and
# telling a Dane to answer in Norwegian is worse than saying nothing.
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
    best = max(counts, key=counts.get)
    # A stray CJK quotation mark in an English sentence is not Chinese. Ask for
    # a real share of the letters before believing it.
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
    part `instruction()` and a session's locked language both need, kept in
    one place so the wording never drifts between the two callers.

    Names the language when it is known, and says what must *not* follow it:
    the board's own words. A model told to answer in Swedish will otherwise
    translate `DCbus` and `NTC` too, and a translated channel name is one
    nobody can grep for in the CSVs.
    """
    if not name:
        return ('Answer in the language the question was asked in. Channel '
                'names, units and register names stay exactly as the board '
                'prints them.')
    return ('The session language is %s. Answer in %s and in no other '
            'language, whatever language the tool output and the documents '
            'are in. Channel names, units and register names stay exactly '
            'as the board prints them.' % (name, name))


def instruction(text):
    """The line to append to the system prompt for this one question, with
    the language detected fresh from `text` - the one-shot case, where
    there is no session to have locked anything in the first place."""
    return instruction_for(detect(text))


# Language names as they would actually appear when someone asks for one by
# name - the English word and, for the two this bench is mostly spoken in,
# the Swedish word too. Deliberately not exhaustive: this is a trigger for
# "answer in French" / "svara pa franska", not a translation table, and a
# name missing from this dict just means that request falls back to being
# detected from the language it was written in instead, same as always.
LANGUAGE_NAMES = {
    'Swedish':    ('swedish', 'svenska'),
    'English':    ('english', 'engelska'),
    'German':     ('german', 'tyska'),
    'Danish':     ('danish', 'danska'),
    'Norwegian':  ('norwegian', 'norska'),
    'Dutch':      ('dutch', 'nederlandska'),
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
    'Thai':       ('thai', 'thailandska'),
    'Greek':      ('greek', 'grekiska'),
    'Hebrew':     ('hebrew', 'hebreiska'),
    'Arabic':     ('arabic', 'arabiska'),
}
_NAME_TO_LANGUAGE = {alias: name for name, aliases in LANGUAGE_NAMES.items()
                     for alias in aliases}

# A language's own name has to sit next to one of these to count as a
# request rather than a mention - "the German firmware bug" is not a
# request for German, and this is what keeps it from reading as one.
_RESPONSE_VERBS = ('svara', 'svarar', 'answer', 'respond', 'reply',
                   'antworte', 'antworten', 'reponds', 'répondre',
                   'responde', 'rispondi')


def requested_language(text):
    """A language named outright in `text`, next to a word for "answer" -
    "svara pa engelska", "please answer in English" - independent of what
    language `text` itself is written in. This is what lets a session
    written in Swedish ask for an English answer without that one message
    being mistaken for a language switch by `detect()` alone, which only
    ever looks at the words actually used - and the response-verb check is
    what keeps "the German firmware has a bug" from reading as a request
    for one, just because it names a language in passing.

    None means no language was requested, not that none could be detected -
    callers fall back to `detect()` for that.
    """
    words = [w.lower() for w in WORD.findall(text or '')]
    if not any(w in _RESPONSE_VERBS for w in words):
        return None
    for word in words:
        if word in _NAME_TO_LANGUAGE:
            return _NAME_TO_LANGUAGE[word]
    return None
