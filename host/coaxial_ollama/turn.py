"""One question answered: the model's rounds, the calls it asks for, the answer settled."""
import json
import re
import textwrap
from contextlib import suppress

from . import intent
from . import language
from . import replies
from . import tools as toolmod
from .capability import probe
from .client import FAULTS
from .sandbox import clip
from coaxial.errors import LINK_FAULTS, RigError
from coaxial.simulated import bus_nodes
from coaxial_ollama.words import BLANK_LINE, board_switch


ERR_CLASS = re.compile(r'^ERR (\w+):')


# Tools that actually reach the board - not 'docs', which reads local files and
# proves nothing about the link either way.
LINK_TOOLS = toolmod.LINK_TOOLS


# render.error's class name, for the calls that report through it.
CONTACT_LOST = {'ConnectError', 'NoReplyError', 'CrcError', 'FrameError',
                'PayloadError'}


# Rows of a tool result shown in full - every channel, or a whole self_test,
# without an unbounded dump from run_python.
TRACE_ROWS = 24


# Width before a row continues on the next line, and the lines one row may
# take.
TRACE_WIDTH = 96


TRACE_LINES = 3


# Tools where the same call twice means something: a reading changes with time,
# and code knows why it is running again.
REPEATABLE = {'analog_read', 'run_python', 'run_command'}


AFE_STATE = re.compile(r'^on=(\d)')


def _afe_noise(name, args, raw):
    """Whether this afe_power result is worth a line on screen."""
    if name != 'afe_power':
        return False
    if str(raw).startswith('ERR not asked for'):
        return True
    wanted = str((args or {}).get('action', 'read')).strip().lower()
    if wanted not in ('on', 'off'):
        return False
    got = AFE_STATE.match(str(raw))
    return bool(got) and (got.group(1) == '1') == (wanted == 'on')


def _wrapped(line):
    """One traced row as the lines it takes on screen, indented."""
    body = '  %s' % line.rstrip()
    if len(body) <= TRACE_WIDTH:
        return [body]
    lead = len(line) - len(line.lstrip())
    parts = textwrap.wrap(body, width=TRACE_WIDTH,
                          subsequent_indent=' ' * (4 + lead),
                          break_long_words=False, break_on_hyphens=False)
    if not parts:
        return [body[:TRACE_WIDTH]]
    if len(parts) <= TRACE_LINES:
        return parts
    parts = parts[:TRACE_LINES]
    parts[-1] += ' [...]'
    return parts


def _printable(stream):
    """Make a console survive an alphabet that is not its codepage."""
    with suppress(AttributeError, OSError, ValueError):
        if stream.isatty():
            stream.reconfigure(errors='replace')
        else:
            stream.reconfigure(encoding='utf-8', errors='replace')
    return stream


class Turn:
    """What one question accumulates while it is being answered."""

    NUDGE_LIMIT = 2        # then end the turn rather than ask nicely forever

    def __init__(self):
        self.answer = ''
        self.link_error = None      # a call that did not reach the board
        self.code_error = None      # last run_python/run_command that failed
        self.channels = None        # names in this turn's analog_read table
        self.table = None           # and that table, for --quiet
        self.maps = []              # name sets a map or a level read listed
        self.map_text = None        # and that render, for --quiet
        self.diagnosed = False      # link_diagnose ran, and was traced
        self.seen = {}              # (name, args) -> its rendered result
        self.nudges = 0

    def nudge(self, chat, say, giving_up):
        """Ask once more, or give up. None means go round again."""
        if self.nudges < self.NUDGE_LIMIT:
            self.nudges += 1
            chat.history.append({'role': 'user', 'content': say})
            return None
        return giving_up

    def remember_map(self, text):
        """The channel names a map or a level read just put on screen."""
        sets = [set(m.lower() for m in pattern.findall(text))
                for pattern in (replies.MAP_ROW, replies.DIGITAL_ROW,
                                replies.DIGITAL_SIGNAL)]
        sets = [names for names in sets if names]
        if sets:
            self.maps = sets
            self.map_text = text


class ChatTurn:

    """A question in, the calls made, an answer out that the facts hold up."""

    _traced = False

    def _probe_link(self):
        """A live, free-standing check of the link - no AFE, no sample."""
        probe = self.toolbox.call('link', {'op': 'state'})
        lost = ERR_CLASS.match(str(probe))
        self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
        if not self.link_ok:
            # Same reasoning as the main loop's own LINK_TOOLS handling: a
            # cable pulled and replugged can leave the cached board's serial
            # handle permanently dead, since a USB VCP re-enumerates on replug
            # rather than reviving the same handle.
            self.toolbox.session.reset()
            # ...and try once more, which is the whole point of the reset.
            probe = self.toolbox.call('link', {'op': 'state'})
            lost = ERR_CLASS.match(str(probe))
            self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
        self.history.append({'role': 'tool', 'tool_name': 'link',
                             'name': 'link', 'content': 'link: %s' % probe})
        return probe

    def _link_down_message(self, link_error, shown=False):
        """'link is down, not answered: ...', plus why - run here, by the
        host, every time the link is down, rather than left for the model
        to think to call link_diagnose.
        """
        text = str(link_error)
        if shown:
            # The class and its first clause: 'ERR ConnectError: cannot open
            # COM9 at 115200 baud', not that plus the port name again, the OS
            # exception's own repr and the generic advice.
            head = text.split(' -> ')[0]
            return 'link is down, not answered: %s' \
                % clip(': '.join(head.split(': ')[:2]), 120)
        message = 'link is down, not answered: %s' % text
        try:
            diagnosis = self.toolbox.call('link_diagnose', {})
        except LINK_FAULTS:
            return message
        if diagnosis and not str(diagnosis).startswith('ERR'):
            message += '\n' + str(diagnosis)
        return message

    def ask(self, question, max_calls=6):
        """One question, however many tool calls it takes."""
        # A message that is nothing but a language request never reaches the
        # model: the lock is host state, and the answer is one word.
        switch = language.bare_switch(question)
        if switch:
            self.language = switch
            answer = language.okay(switch, getattr(self.out, 'encoding', None))
            self.io_log.turn(question)
            self.io_log.answer(answer)
            return answer
        # Same rule one layer out: an order to change the board is the host's
        # to carry out, not a model's to describe.
        board = board_switch(question)
        if board:
            answer = language.localise(self._switch_board(board),
                                       self.screen_language())
            self.io_log.turn(question)
            self.io_log.answer(answer)
            return answer
        answer = language.localise(self._ask_inner(question, max_calls),
                                   self.screen_language())
        answer = self._say_node(answer)
        self.io_log.answer(answer)
        return answer

    def _say_node(self, answer):
        """Name the node above the answer, on the turn it changed."""
        session = self.toolbox.session
        here = (session.bus, session.unit)
        if here[1] is None or here == self._said_node:
            return answer
        self._said_node = here
        where = self._where(here)
        line = language.localise('From %s:' % where, self.screen_language())
        return line + '\n' + answer if answer.strip() else line

    @staticmethod
    def _where(here):
        """The node's place on its bus as the stand-in names it - the knee,
        the hip - or its number when nothing names it."""
        bus, unit = here
        if unit == 0:
            return 'every node on %s' % (bus or 'the bus')
        where = None
        with suppress(Exception):
            where = ((bus_nodes(bus).get(unit) or (None, None, None))[2]
                     if bus else None)
        return where or ('%s node %d' % (bus, unit) if bus
                         else 'node %d' % unit)

    def _compile(self, question):
        """The intent hint for this question, or '' when there is none."""
        self._intent_did = self._intent_kind = self._intent_tool = None
        if not self.compile_intent:
            return ''
        got, kind, why = intent.compile_intent(self.client, question)
        self._intent_why = why
        if got is None:
            return ''
        self._intent_did, self._intent_kind = got, kind
        self._intent_tool = intent.tool_for(got, kind)
        return intent.hint(got, kind)

    NARRATE = ("The board answered the operator's question with the output "
               "below. Write one short sentence about it in %s - what it "
               "shows, or that it was read. Never repeat the rows: they are "
               "already on the operator's screen.")

    def _run_plan(self, question, calls):
        """Make the compiled calls, then ask for a sentence about them."""
        self.history.append({'role': 'user', 'content': question})
        self.prompt_history += (question,)
        self.io_log.turn(question)
        # A planned turn never calls trim(), which is where the lock moves.
        self._lock_language(question)
        self._traced = False
        shown, channels, table = [], None, None
        for name, args in calls:
            try:
                raw = self.toolbox.call(name, args)
            except RigError as exc:
                # A planned call is the host's own, so a raise here would take
                # the turn with it rather than reaching the operator as the
                # link failure it is.
                raw = 'ERR %s: %s' % (type(exc).__name__, exc)
            text = str(raw)
            if str(raw).startswith('ERR '):
                self.io_log.call(name, args, text)
                return self._link_down_message(text, shown=False)
            self._trace(text)
            self.io_log.call(name, args, text)
            shown.append(text)
            found = replies.READING_ROW.findall(text)
            if found:
                channels, table = [m.lower() for m in found], text
                self.last_channels = channels
        self.history.append({'role': 'user',
                             'content': BLANK_LINE.join(shown)})

        answer = ''
        try:
            said = self.client.chat(
                [{'role': 'system',
                  'content': self.NARRATE % (self.language or 'English')},
                 {'role': 'user', 'content': question},
                 {'role': 'user', 'content': BLANK_LINE.join(shown)}])
            answer = (said.get('content') or '').strip()
        except FAULTS:
            answer = ''                # the rows are the answer either way
        if channels and replies.is_retype(answer, channels):
            answer = table if (self.quiet and table) else ''
        elif self.quiet and not answer:
            answer = BLANK_LINE.join(shown)
        self.history.append({'role': 'assistant', 'content': answer})
        return answer

    def _ask_inner(self, question, max_calls=6):
        if self.over_budget():
            return 'budget of %d tokens is spent; /clear or raise --budget' \
                % self.budget

        # Compile before answering.
        self.intent = self._compile(question)
        planned = intent.plan(self._intent_did, self._intent_kind)
        if planned:
            return self._run_plan(question, planned)

        self.history.append({'role': 'user', 'content': question})
        self.prompt_history += (question,)
        self.io_log.turn(question)
        self._traced = False   # nothing on screen yet, so no leading gap

        turn = Turn()
        for _ in range(max_calls + 1):
            done = self._round(turn)
            if done is not None:
                return done if isinstance(done, str) else self._settle(turn)
        return self._settle(turn)

    def _round(self, turn):
        """One model turn and the calls it asked for."""
        before = self.client.usage()
        message = self.client.chat(self.trim(), self.schemas)
        after = self.client.usage()
        self._meter(after['prompt_tokens'] - before['prompt_tokens'],
                    after['eval_tokens'] - before['eval_tokens'])
        self._notes()

        message.pop('thinking', None)
        calls = message.get('tool_calls') or []
        turn.answer = (message.get('content') or '').strip()

        # A tool call written as prose is still a tool call.
        if not calls:
            salvaged, turn.answer = replies.salvage_calls(turn.answer)
            calls = salvaged or calls
            message = (dict(message, content='', tool_calls=calls)
                       if salvaged else message)

        self.history.append(message)
        if not calls:
            return self._no_calls(turn)
        for call in calls:
            self._run_call(turn, call)
        return None

    def _no_calls(self, turn):
        """The model wrote instead of calling. None to nudge and go again."""
        # Three shapes of one problem: answering from memory instead of
        # checking, retyping the last reading instead of taking a new one, and
        # answering nothing at all.
        stale = not turn.channels and (
            not turn.answer or (self.last_channels and (
                not self.link_ok
                or replies.is_retype(turn.answer, self.last_channels))))
        if stale:
            return self._stale(turn)
        # A reading did succeed this turn and the model still wrote nothing.
        if not turn.answer:
            return turn.nudge(
                self, 'Answer the question in words now. The tool output is '
                'already on screen - do not repeat it.',
                'the reading above is all that came back - ask again.')
        # It knew exactly what to do and did not do it.
        if (replies.NAMED_TOOL.search(turn.answer)
                and turn.nudges < Turn.NUDGE_LIMIT):
            turn.nudges += 1
            self.history.append({'role': 'user', 'content':
                                 'Call the tool now - do not describe it.'})
            return None
        return True

    def _stale(self, turn):
        """An answer with no reading behind it: the link's own state, or a
        nudge to take one.
        """
        probe = self._probe_link()
        if not self.link_ok:
            return self._link_down_message(
                probe, shown=turn.diagnosed and not self.quiet)
        return turn.nudge(
            self, 'The link just answered. Answer the question now - '
            'with a fresh call if it needs one, and in words if it does '
            'not. Never reuse an old reading.',
            'no reading taken this turn - ask again.')

    def _fresh(self, turn, key, name, args):
        """One call made, and remembered under its key for this turn."""
        raw = self.toolbox.call(name, args)
        if isinstance(raw, toolmod.Reported):
            raw = 'noted: %s' % raw.note
        turn.seen[key] = raw
        return raw

    def _note_link(self, turn, text):
        """What a call that reached for the board says about the link."""
        lost = ERR_CLASS.match(text)
        self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
        turn.link_error = text if not self.link_ok else None
        if not self.link_ok:
            self.toolbox.session.reset()

    def _run_call(self, turn, call):
        """Make one call the model asked for, and record what it means."""
        name = (call.get('function') or {}).get('name', '?')
        args = toolmod.arguments(call)
        key = (name, json.dumps(args, sort_keys=True, default=str))

        # Do not spend a board round trip re-asking a question this turn
        # already has the answer to - and say so plainly rather than repeating
        # the same line, which is what asked for the repeat in the first place.
        repeated = name not in REPEATABLE and key in turn.seen
        raw = (turn.seen[key] if repeated
               else self._fresh(turn, key, name, args))
        result = ('unchanged this turn, already asked: %s' % raw
                  if repeated else raw)

        text = str(raw)
        failed = text.startswith('ERR')
        if name in LINK_TOOLS:
            self._note_link(turn, text)
        if name == 'link_diagnose' and not failed:
            # Its checklist is on screen from the trace below.
            turn.diagnosed = True
        if name in toolmod.CODE_CALLS:
            # A failed build, or a --confirm the operator declined, is a fact
            # this loop holds.
            turn.code_error = text if failed else None
        if name == 'analog_read' and not failed:
            # A fresh table replaces the last one remembered; an error leaves
            # the previous table in place rather than wiping it, since
            # link_error already takes priority below either way.
            turn.channels = set(m.lower()
                                for m in replies.READING_ROW.findall(text))
            self.last_channels = turn.channels
            turn.table = text
        if name in ('board_info', 'digital_read') and not failed:
            turn.remember_map(text)
        if not _afe_noise(name, args, raw):
            self._trace(result)
        self.io_log.call(name, args, result)         # always - see IOLog
        self.history.append({'role': 'tool', 'tool_name': name, 'name': name,
                             'content': '%s: %s' % (name, result)})

    def _settle(self, turn):
        """The answer the operator gets, after the facts the loop holds."""
        answer = turn.answer
        # A read that failed on the wire is ground truth; the model gets no
        # vote.
        if turn.link_error is not None:
            # `and not self.quiet`: with the trace off there is nothing on
            # screen above this, so the checklist has to come with the answer
            # or the operator is told the link is down and nothing else.
            return self._link_down_message(
                turn.link_error, shown=turn.diagnosed and not self.quiet)
        if turn.code_error is not None:
            return ('the last run_python/run_command call failed, nothing '
                    'was done: %s' % turn.code_error)
        # SYSTEM says not to; qwen2.5:14b did it every time across three
        # sessions.
        if turn.channels and replies.is_retype(answer, turn.channels):
            return turn.table if (self.quiet and turn.table) else ''
        if any(replies.is_retype(answer, names, minimum=2)
               for names in turn.maps):
            return turn.map_text if (self.quiet and turn.map_text) else ''
        # An answer that hit the token cap stops mid-sentence, and a table that
        # stops mid-row reads as complete to everyone except a reader counting
        # rows.
        if self.client.truncated and answer:
            answer += ('\n[cut off at --words %s. Ask again with more, or ask '
                       'for fewer channels.]'
                       % self.client.options.get('num_predict', '?'))
        return answer

    def screen_language(self):
        """Which language this session's own text prints in - the session
        language, which starts as the machine's locale and moves only when a
        question is actually in another one."""
        return self.language

    def _notes(self):
        """Say what the client had to do to the machine to answer at all."""
        notes = self.client.notes
        if not notes:
            return
        drained, notes[:] = list(notes), []
        for note in drained:
            self.io_log.write('  ! %s\n' % note)
            self._trace('! ' + note)

    def _trace(self, result):
        """Print what a call returned, as the grid render.py already built
        it.
        """
        if self.quiet:
            return
        # A blank line between blocks, once there is more than one.
        lead = ''
        if self._traced and '\n' in str(result).strip():
            lead = '\n'
        self._traced = True
        # English stays in the result the model reads, the log keeps and the
        # MCP server serves; the screen gets the operator's language.
        lines = (language.localise(str(result), self.screen_language())
                 .splitlines() or [''])
        with self.print_lock:
            if lead:
                print(file=self.out, flush=True)
            for line in lines[:TRACE_ROWS]:
                for part in _wrapped(line):
                    print(part, file=self.out, flush=True)
            if len(lines) > TRACE_ROWS:
                print('  ... [%d more rows]' % (len(lines) - TRACE_ROWS),
                      file=self.out, flush=True)
