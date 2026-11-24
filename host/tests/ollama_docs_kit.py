"""The ollama tools suite's documents, reachable from a prompt at the bench (`test_docs`)."""
import io
import os

from coaxial_ollama import tools as toolmod
from coaxial_ollama.sandbox import Scope
from ollama_support import ScriptedModel, SimulatedSession, call


# ---- the documents are reachable from a prompt ----------------------------

def test_docs(report):
    """The one reader who could not open docs/ was the model at the bench."""
    from coaxial_mcp import docs as docmod
    from coaxial_mcp.tools import HANDLERS
    from coaxial_mcp.schema import TOOLS

    report.check('docs is a tool the model can call',
                 'docs' in HANDLERS
                 and any(spec['name'] == 'docs' for spec in TOOLS))

    index = docmod.docs()
    for name in ('README', 'CLAUDE', 'ARCHITECTURE', 'PROTOCOL', 'HARDWARE',
                 'FINDINGS', 'MODELS'):
        report.check('the index lists ' + name, name in index)
    report.check('the index is headings, not documents',
                 len(index) < 4500, '%d chars' % len(index))
    report.check('the index says how to go deeper',
                 'section=' in index and 'find=' in index)

    # Headings are read out of the document rather than spelled here.
    import re as _re
    text = str(io.open(os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), 'docs', 'MODELS.md'),
        encoding='utf-8').read())
    heads = [(len(m.group(1)), m.group(2).strip())
             for m in _re.finditer(r'^(#{2,3}) (.+)$', text, _re.M)]
    # The first chapter that has a subsection - the first chapter in
    # the file need not, and asserting a parent carries children against one
    # with none proves nothing.
    at = next((i for i, (lvl, _) in enumerate(heads)
               if lvl == 2 and i + 1 < len(heads) and heads[i + 1][0] == 3),
              None)
    if at is None:                    # a document with no subsections at all
        at = next(i for i, (lvl, _) in enumerate(heads) if lvl == 2)
        child = None
    else:
        child = heads[at + 1][1]
    chapter = heads[at][1]
    after = next((h for lvl, h in heads[at + 1:] if lvl == 2), None)

    outline = docmod.docs(doc='MODELS')
    report.check('one document lists its own headings',
                 chapter in outline and (child or chapter) in outline,
                 '%r / %r' % (chapter, child))

    deep = next((h for lvl, h in heads if lvl == 3), chapter)
    body = docmod.docs(doc='MODELS', section=deep)
    report.check('a section returns its text', len(body) > len(deep) + 40)
    later = next((h for lvl, h in heads[[i for i, (_, x)
                                         in enumerate(heads)
                                         if x == deep][0] + 1:] if lvl == 3),
                 None)
    report.check('a section stops at the next heading of its level',
                 later is None or later not in body, '%r' % later)

    # A parent section keeps its children: asking for the chapter should not
    # silently return only its first paragraph.
    parent = docmod.docs(doc='MODELS', section=chapter)
    report.check('a parent section carries its subsections',
                 child is None or child in parent or 'clipped' in parent)
    report.check('and stops at the next chapter',
                 after is None or after not in parent, '%r' % after)

    report.check('heading matching is loose enough to be usable',
                 docmod.docs(doc='MODELS', section=deep.lower()) == body)

    # The AFE number is the one a weak model invents, so it has to be findable.
    hits = docmod.docs(find='25.00')
    report.check('search reaches the AFE-off number', 'FINDINGS' in hits
                 or 'HARDWARE' in hits or 'CLAUDE' in hits)
    report.check('search says where it found it, not just that it did',
                 len(hits.splitlines()) >= 2)
    # A hit without its chapter can say the opposite of what the document says:
    # the phase V entry that matches lives under "Ruled Out", and a model that
    # cannot see that reports a dead end as the explanation.
    refuted = next(head for _, head, _ in docmod._headings(
        docmod._read(docmod.paths()['FINDINGS'])) if 'Ruled Out' in head)
    body = docmod.docs(doc='FINDINGS', section=refuted).splitlines()[2:]
    phrase = next((w.lstrip('*- ') for line in body for w in [line.strip()]
                   if len(w) > 30 and not w.startswith('#')), None)
    report.check('a search hit carries the chapter it sits under',
                 phrase is None
                 or refuted in docmod.docs(find=phrase[:40]),
                 repr(phrase and phrase[:40]))

    report.check('search is capped rather than dumping a document',
                 len(docmod.docs(find='the').splitlines()) <= docmod.FIND_HITS + 1)

    report.check('no section comes back longer than the clip allows',
                 all(len(docmod.docs(doc=name, section=head)) <=
                     docmod.CLIP + 200
                     for name in docmod.paths()
                     for _, head, _ in docmod._headings(
                         docmod._read(docmod.paths()[name]))))

    for bad, expect in ((dict(doc='NOPE'), 'no document'),
                        (dict(doc='MODELS', section='no such heading'), 'no section'),
                        (dict(find='   '), 'something to look for')):
        try:
            docmod.docs(**bad)
            report.check('a bad docs call is refused by name', False, str(bad))
        except ValueError as exc:
            report.check('a bad docs call is refused by name', expect in str(exc),
                         str(exc)[:54])

    report.check('every document named is actually on disk',
                 set(docmod.paths()) == set(docmod.NAMES),
                 ', '.join(sorted(set(docmod.NAMES) - set(docmod.paths()))) or 'all')

    # The bench prompt has to point at the tool, or nothing above matters.
    from coaxial_ollama import debug, runner
    from coaxial_ollama import words
    # Not in SYSTEM: the language is worked out here and named in the
    # turn's system message, because a model asked to work it out itself
    # answered a European question in Chinese.
    from coaxial_ollama import language
    talk = debug.Chat.__new__(debug.Chat)
    talk.keep = 6
    for question, expect in (('Vad \u00e4r temperaturen p\u00e5 kortet?', 'Swedish'),
                             ('What is the board temperature?', 'English'),
                             ('Was macht der AFE-Schalter?', 'German')):
        talk.history = [{'role': 'user', 'content': question}]
        head = talk.trim()[0]['content']
        report.check('the turn names the language: ' + expect,
                     ('in %s' % expect) in head, head.splitlines()[-1][:56])

    # Measured on this bench: this exact question has only one word from the
    # rest of Swedish's list ('over') against two from Dutch's ('en', 'de'), so
    # Dutch outscored Swedish outright and the model answered in a Dutch/
    # Norwegian mix instead.
    report.check('a Swedish question is not lost to Dutch on "en"/"de" alone',
                 language.detect(
                     'ger du mig en tabell \u00f6ver de analoga m\u00e4tv\u00e4rdena?')
                 == 'Swedish')

    # Locked at 'German' by the loop above.
    talk.history = [{'role': 'user', 'content': 'status?'}]
    report.check('an ambiguous follow-up keeps the session lock, not a '
                 'fallback to mirroring',
                 'in German' in talk.trim()[0]['content'])

    fresh = debug.Chat.__new__(debug.Chat)
    fresh.keep = 6
    fresh.history = [{'role': 'user', 'content': 'status?'}]
    report.check('but with nothing locked yet, the same question falls '
                 'back to mirroring',
                 'language the question was asked in' in fresh.trim()[0]['content'])
    report.check('and the board keeps its own words either way',
                 'stay exactly as the board prints them'
                 in language.instruction('Vad \u00e4r temperaturen?'))

    talk.history = [{'role': 'user', 'content': 'svara pa engelska tack'}]
    talk.trim()
    report.check('naming a language outright switches the lock even though '
                 'the question itself is written in the locked language',
                 talk.language == 'English', talk.language)

    # Measured at the prompt: "forklara pa japanska..." named a language next
    # to a verb that was not in the list, so the request was missed and the
    # turn went out under *Answer in Swedish and in no other language* - the
    # host contradicting the operator in the same system prompt.
    talk.history = [{'role': 'user',
                     'content': 'f\u00f6rklara p\u00e5 japanska vad detta '
                                'projektet handlar om'}]
    head = talk.trim()[0]['content']
    report.check('a language asked for with a verb other than "answer" is '
                 'still a request', 'in Japanese' in head, talk.language)

    # ...and it is that turn's request, not a new session language: the next
    # Swedish question takes the lock straight back.
    talk.history = [{'role': 'user', 'content': 'och vad l\u00e4ser NTC:n nu?'}]
    talk.trim()
    report.check('and the next question in the session language takes it back',
                 talk.language == 'Swedish', talk.language)

    for question in ('the German firmware bug is back',
                     'varf\u00f6r \u00e4r dokumentationen p\u00e5 engelska?',
                     'vad heter skruv p\u00e5 japanska?'):
        got = language.requested_language(question)
        report.check('a language named in passing is not a request: %s'
                     % question[:34], got is None, str(got))

    # A lock with no way out is a trap.
    talk.language = 'Korean'
    talk.history = [{'role': 'user', 'content': 'byt spr\u00e5k till svenska'}]
    talk.trim()
    report.check('a locked session can be talked out of its language',
                 talk.language == 'Swedish', talk.language)

    for question, expect in (('svenska tack', 'Swedish'),
                             ('switch to Swedish please', 'Swedish'),
                             ('kan du prata svenska?', 'Swedish'),
                             ('tillbaka till engelska', 'English')):
        got = language.requested_language(question)
        report.check('asking for a language plainly: %s' % question[:30],
                     got == expect, str(got))

    # ...and the model is told the same thing, for the phrasing the host misses
    # next.
    report.check('an explicit request overrides the lock in the prompt too',
                 'unless the operator asks for another'
                 in language.instruction_for('Korean'))

    # A message that is nothing but a language request is host business.
    switch_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    switcher = debug.Chat(ScriptedModel([]), switch_box, out=io.StringIO())
    switcher.language = 'Swedish'
    said = switcher.ask('byt spr\u00e5k till japanska')
    report.check('a bare switch never reaches the model',
                 not switcher.client.prompts and not switcher.history,
                 '%d turns' % len(switcher.client.prompts))
    report.check('and is answered with one word in the language asked for',
                 said == language.OKAY['Japanese'] == '\u308f\u304b\u308a\u307e\u3057\u305f',
                 said.encode('unicode_escape').decode())
    report.check('which is what the lock moved to',
                 switcher.language == 'Japanese', switcher.language)

    # The same cp1252 case greeting() documents: Japanese renders as a row of
    # question marks on a console that cannot hold it, and "Okay" beats five of
    # those.
    narrow = io.TextIOWrapper(io.BytesIO(), encoding='cp1252')
    plain = debug.Chat(ScriptedModel([]), switch_box, out=narrow)
    report.check('an alphabet the console lacks falls back to English',
                 plain.ask('byt spr\u00e5k till japanska') == 'Okay')

    # ...but a request with a question attached is still the model's turn, in
    # the new language.
    asked_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    with_question = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': '\u3053\u306e\u30d7\u30ed\u30b8\u30a7\u30af\u30c8\u306fBLDC\u30a4\u30f3\u30d0\u30fc\u30bf\u3067\u3059\u3002'}]),
        asked_box, out=io.StringIO())
    with_question.language = 'Swedish'
    said = with_question.ask('f\u00f6rklara p\u00e5 japanska vad detta projektet '
                             'handlar om')
    report.check('a request with a question attached still goes to the model',
                 with_question.client.prompts and said.startswith('\u3053\u306e'),
                 said[:12].encode('unicode_escape').decode())

    for question, bare in (('svenska tack', 'Swedish'),
                           ('switch to Swedish please', 'Swedish'),
                           ('kan du prata svenska?', 'Swedish'),
                           ('l\u00e4s NTC:n', None),
                           ('f\u00f6rklara p\u00e5 japanska vad detta handlar om', None)):
        got = language.bare_switch(question)
        report.check('bare switch or a real question: %s' % question[:32],
                     got == bare, str(got))

    # Measured live: the "call the tool now" nudge goes into history with
    # role=='user', and its English words then flipped the language on the next
    # trim().
    blank_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    blank = debug.Chat(ScriptedModel([
        call('analog_read'),
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': 'Kortet \u00e4r en trefas BLDC-drivare.'},
    ]), blank_box, out=io.StringIO())
    said = blank.ask('beskriv h\u00e5rdvaran f\u00f6r en novis')
    report.check('a reading followed by silence is nudged into an answer',
                 said == 'Kortet \u00e4r en trefas BLDC-drivare.', repr(said))
    report.check('and the nudge asks for words, not for another reading',
                 any('in words' in (m.get('content') or '')
                     for m in blank.history), 'no such nudge')

    mute_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    mute = debug.Chat(ScriptedModel([
        call('analog_read'),
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': ''},
    ]), mute_box, out=io.StringIO())
    said = mute.ask('beskriv h\u00e5rdvaran f\u00f6r en novis')
    # Swedish, because the question was: the fallback is host-authored text
    # reaching the screen, so it goes through PHRASES like every other line the
    # operator reads.
    report.check('a model that stays silent still ends the turn with words',
                 'fr\u00e5ga igen' in said, repr(said))

    nudge_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    nudged = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': 'jag ska nu anropa analog_read'},
        call('analog_read')]), nudge_box, out=io.StringIO())
    nudged.ask('vad ar temperaturen pa kortet?')
    report.check("a nudge mid-turn does not steal the language lock from "
                 "what the operator actually typed",
                 nudged.language == 'Swedish', nudged.language)

    # A console that cannot encode the answer must not lose it.
    import io as _io
    raw = _io.BytesIO()
    narrow = _io.TextIOWrapper(raw, encoding='cp1252', newline='')
    debug._printable(narrow)
    try:
        narrow.write('resistans 6487 ohm, \u03a9 och \u0142')
        narrow.flush()
        report.check('an alphabet the console lacks costs a glyph, not the answer',
                     b'resistans 6487 ohm' in raw.getvalue())
    except UnicodeEncodeError as exc:
        report.check('an alphabet the console lacks costs a glyph, not the answer',
                     False, str(exc)[:50])
    report.check('and a stream that cannot be reconfigured is left alone',
                 debug._printable(object()) is not None)

    # Asked to measure, the model called docs and answered with HARDWARE.md's
    # own channel table instead of a reading.
    report.check('the bench prompt does not send the model to the documents',
                 'docs' not in words.SYSTEM and 'FINDINGS' not in words.SYSTEM)
    report.check('the docs warning is there for a session that does ask',
                 'never docs' in words.DOCS_HINT)
    report.check('and never to restate a result already printed above',
                 'restate' in words.SYSTEM)
    report.check('afe_power is never framed as refusable',
                 'afe_power' in words.SYSTEM and 'order to do it' in words.SYSTEM)

    # The line that taught the error.
    report.check('SYSTEM does not tell the model a list is a reading',
                 'table or list' not in words.SYSTEM,
                 [l for l in words.SYSTEM.splitlines() if 'analog_read' in l][:1])
    # The rule, not the sentence: board_info owns the map, and "list" is named
    # as the word that decides nothing.
    report.check('it names board_info as the map, and disowns "list"',
                 'board_info' in words.SYSTEM
                 and 'never "list"' in words.SYSTEM,
                 [l for l in words.SYSTEM.splitlines() if 'board_info' in l])
    report.check('and digital_read for a pin, beside analog_read',
                 'digital_read' in words.SYSTEM and 'analog_read' in words.SYSTEM)
    report.check('and a read with AFE off is never framed as impossible',
                 'AFE on or off and reports' in words.SYSTEM)
    report.check('and afe_power never fires as a side effect of a reading',
                 'never afe_power first' in words.SYSTEM)
    report.check("and mid-scale is the tool's fact to report, not the prompt's "
                 'to recite',
                 'mid-scale' not in words.SYSTEM)
    report.check('the runner tells it too',
                 'docs' in runner.SYSTEM and 'FINDINGS' in runner.SYSTEM)
    report.check('no default tool set offers docs - only `docs` and `all`',
                 not any('docs' in words.SETS[s]
                         for s in ('read', 'code', 'pins', 'build'))
                 and 'docs' in words.SETS['docs']
                 and 'docs' in words.SETS['all'])
