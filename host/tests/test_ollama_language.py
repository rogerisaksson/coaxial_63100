#!/usr/bin/env python3
"""The session language, its lock, and the phrase table.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_language.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession, 
    _flat, _unformat, io, os, toolmod)   # noqa: E402



# ---- one screen, one language ---------------------------------------------

def test_screen_language(report):
    """A Swedish question answered in Swedish, under an English warning this
    project wrote itself, is one screen in two languages."""
    from coaxial_ollama import debug, language

    banner = ('AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale, and the degC and volts below are arithmetic '
              'on that - not a temperature, not a bus voltage. Call '
              'afe_power on to measure.')
    turned = language.localise(banner, 'Swedish')
    report.check('host prose turns into the session language',
                 'AFE AV' in turned and 'unpowered' not in turned,
                 turned[:48])
    report.check('a language with no table is left in English',
                 language.localise(banner, 'German') == banner
                 and language.localise(banner, None) == banner)

    # The values are the board's, and they come back exactly as they went in.
    checklist = ('1. Target power (ST-Link/SWD): 3.27V - powered, cable '
                 'seated.\n2. COM ports Windows sees: COM4\n'
                 '3. Configured port COM4: present.')
    swedish = language.localise(checklist, 'Swedish')
    report.check('the numbers and port names survive being translated',
                 '3.27V' in swedish and swedish.count('COM4') == 2
                 and 'Målspänning' in swedish, swedish.splitlines()[0])
    report.check('and the step numbering is still the step numbering',
                 [line.split('.')[0] for line in swedish.splitlines()]
                 == ['1', '2', '3'])

    table = ('  4  NTC     SE    32768.0  +1.6500V 25.00C\n'
             '  5  DCbus   SE    32768.0  +1.6500V 39.075V bus')
    report.check('a reading is not touched - no channel name is translated',
                 language.localise(table, 'Swedish') == table)

    # Two signals, and they are not the same one. The greeting has no question
    # to read, so it takes the machine's locale; everything after it follows
    # the question. `screen` is the fallback for the case detect() abstains,
    # and it is None here on purpose - a suite that read the Windows locale
    # would pass on one machine and fail on the next.
    box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    talk = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                      out=io.StringIO())
    report.check('a test session starts in no language, not the machine one',
                 talk.language is None and talk.screen_language() is None)
    local = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                       out=io.StringIO(), session_language='Swedish')
    report.check('a real session starts in the locale build() read',
                 local.screen_language() == 'Swedish')

    # The locale is where it starts, not where it is stuck: a question in
    # another language moves it, and so does asking for one.
    local.history = [{'role': 'user', 'content': 'read all the analog channels'}]
    local.prompt_history = ['read all the analog channels']
    local.trim()
    report.check('a question in another language moves it',
                 local.language == 'English', local.language)
    local.prompt_history.append('svara på svenska')
    local.history = [{'role': 'user', 'content': 'svara på svenska'}]
    local.trim()
    report.check('and asking for one outright moves it too',
                 local.language == 'Swedish', local.language)

    # Every locale this module can name must have a greeting, or a machine
    # set to it opens in English for no reason anyone can see.
    missing = [name for name in set(language._LOCALE_CODES.values())
               if name not in language.GREETINGS]
    report.check('every locale it recognises has a greeting',
                 not missing, ', '.join(sorted(missing)) or 'all present')
    report.check('a console that cannot encode it falls back to English',
                 language.greeting('m', 'Japanese', 'cp1252')
                 == language.greeting('m', 'English')
                 and language.greeting('m', 'Japanese', 'utf-8')
                 != language.greeting('m', 'English'))

    # Every English key must exist verbatim in the source, or a call site has
    # moved on and its translation is dead text nothing will ever match.
    sources = []
    for name in ('coaxial_ollama/debug.py', 'coaxial_ollama/tools.py',
                 'coaxial_mcp/tools.py', 'coaxial_mcp/render.py'):
        with io.open(os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), name), encoding='utf-8') as handle:
            sources.append(handle.read())
    # Quotes stripped before comparing: the source splits these strings
    # across lines, so the literal run is broken by a `' '` at every wrap.
    joined = ' '.join(_flat(text) for text in sources)
    orphans = [key for key in language.PHRASES['Swedish']
               if _flat(_unformat(key)) not in joined]
    report.check('every translation still matches a line in the source',
                 not orphans, '; '.join(o[:40] for o in orphans) or 'all matched')


ROSTER = (
    (test_screen_language, ('language', 'render')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
