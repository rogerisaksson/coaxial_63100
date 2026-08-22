"""A lean prompt loop for debug jobs: fewest tokens in, fewest tokens out.

    python dbg.py "the NTC reads exactly 25.00 - what is wrong?"
    python dbg.py                      # interactive
    python dbg.py -q "which channel is the DC link?"

The runner in runner.py is built for a test plan: a long system prompt, eleven
tools, a fresh conversation per step, a transcript of everything. Every one of
those is right for a report somebody signs and wrong for a question you ask
sixty times an afternoon. This module is the same board and the same tools with
the cost turned down:

  1. A system prompt of about seventy tokens instead of three hundred and fifty.
  2. Five tools by default, not eleven. The tool list is re-sent on every single
     turn, so it is the one cost that scales with turns no matter how short the
     question is - `--tools` picks the subset the job needs.
  3. `num_predict` caps the answer. Debug answers are one or two sentences; a
     reasoning model asked an open question will happily produce eight hundred
     tokens of deliberation instead.
  4. `think` off where the model supports it, which removes that deliberation
     rather than just truncating it.
  5. Old turns are stubbed, not resent whole. A tool result from four questions
     ago is worth its first line, not its forty.
  6. Slash commands run without the model at all. `/py board.afe.state()` and
     `/sh cube-cmake --build --preset Debug` cost zero tokens, and half of what
     one asks a model at a bench is really just "run this and show me".

And it prints what each turn cost, in and out, because a budget nobody can see
is a budget nobody keeps.
"""
import json
import re
import sys
import time

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

from coaxial.errors import RigError                  # noqa: E402

from . import language
from . import tools as toolmod                       # noqa: E402
from .sandbox import Scope, Shell, clip              # noqa: E402

# Deliberately terse, and every line of it earns its place. No restating the
# protocol, no channel map - board_info carries that, once, when asked.
SYSTEM = """You debug a coaxial BLDC inverter test board over a serial link.
Use tools; do not guess. Answer in one or two sentences, no preamble.
The front end switch (afe_power) also powers the ADC reference: with it off
every channel reads mid-scale and the NTC reads exactly 25.00 C, which is not a
measurement. Phase channels sit behind unknown gain, so pin volts is as far as
the data goes. Say plainly when you do not know.
The docs tool has this board's own HARDWARE and FINDINGS: read before guessing."""

# Named subsets, because a debug job knows roughly what it is about to touch.
SETS = {
    'read': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'docs'),
    'code': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'docs', 'run_python'),
    'pins': ('board_info', 'gpio_pin', 'gpio_port', 'test_gate', 'afe_power',
             'docs'),
    'all': tuple(spec['name'] for spec in toolmod.TOOLS if spec['name'] != 'report'),
    'none': (),
}

HELP = """  /py CODE      run python against the board, no model, no tokens
  /sh CMD       run an allowlisted command, no model
  /tools [set]  read|code|pins|all|none, or a comma separated list
  /ctx          what the next turn will cost
  /clear        forget the conversation - the cheapest thing here
  /cost         tokens so far
  /help  /q"""


# How long ollama holds the weights after the last turn. Two numbers, because
# the two modes want opposite things.
#
# In a prompt loop the model is about to be asked again, and the KV cache of
# the prefix is what makes turn nine as quick as turn two - worth 8 GB of VRAM.
# After a single -q question it is not: measured on this bench, a one-shot left
# 9.69 GB resident and expiring 27 minutes later at 1 % utilisation, on a card
# whose desktop then had 3.8 GB to work in. That is the cost of a cache nobody
# is going to hit.
# What the prompt loop shows. The board's name rather than the script's: the
# window this appears in is usually one of several, and 'dbg>' says which
# program is running where the useful thing to know is which bench.
PROMPT = 'Coaxial_63100'

# The dot goes round: three steps right along the floor, up, three back along
# the ceiling, down. Three columns wide and no more - the earlier eight-wide
# bounce pushed the prompt across a quarter of the line for no extra meaning.
#
# The height is a baseline dot against a raised apostrophe. Braille would give
# a true orbit inside a single cell and cannot be used here: this console
# encodes cp1252 (see _printable), where every braille character becomes a
# question mark.
ORBIT = ('.  ', ' . ', '  .', "  '", " ' ", "'  ")
TICK = 0.12         # seconds per step


def bouncing_prompt(out=None, prompt=PROMPT):
    """Draw the prompt with a dot that walks back and forth, until a key.

    Why bother: this prompt sits in the same docked panel as a PowerShell one,
    and two terminals with a `>` in them look identical at a glance. A moving
    dot says which window is waiting for a question and which is waiting for a
    command, without a banner or a colour scheme to remember.

    It stops at the first keypress and redraws the line static, so nothing is
    animating while there is text on it - an animation that repaints under
    typed characters is how a prompt eats an argument. Where there is no
    console to poll (piped input, no msvcrt, output redirected) it prints the
    static prompt once and returns, which is what a script wants anyway.
    """
    out = out or sys.stdout
    static = '%s >' % prompt

    try:
        import msvcrt
    except ImportError:
        msvcrt = None
    if msvcrt is None or not sys.stdin.isatty() or not out.isatty():
        out.write(static + ' ')
        out.flush()
        return

    width = len(ORBIT[0])
    frame = 0
    try:
        while not msvcrt.kbhit():
            out.write('\r%s %s>' % (prompt, ORBIT[frame % len(ORBIT)]))
            out.flush()
            frame += 1
            time.sleep(TICK)
    except (OSError, ValueError):
        pass
    # Static for the typing: same text every time, so what the user sees while
    # they type is what they would have seen without any of this.
    out.write('\r%s%s ' % (static, ' ' * (width + 2)))
    out.write('\r%s ' % static)
    out.flush()


KEEP_ALIVE_REPL = '30m'
KEEP_ALIVE_ONCE = '2m'


def keep_alive_for(args):
    """What the caller asked for, or what the mode implies."""
    if args.keep_alive is not None:
        return args.keep_alive
    return KEEP_ALIVE_REPL if args.repl else KEEP_ALIVE_ONCE


TOOL_TAG = re.compile(r'</?tool_call>', re.I)
CALL_JSON = re.compile(r'\{.*?"name"\s*:.*?\}\s*\}?', re.S)


def _salvage_call(text):
    """A tool call the model wrote as text, and what is left of the text.

    Deliberately narrow. It fires only when the content is *nothing but* a
    tool-call shape - tags, whitespace and one JSON object carrying `name` -
    because a legitimate answer may quote JSON and turning that into a board
    command would be far worse than printing it. Returns (call, remaining
    text): no call means the text was an answer after all.
    """
    stripped = TOOL_TAG.sub('', text).strip()
    if '"name"' not in stripped:
        # No call in it, but a bare `</tool_call>` is not an answer either:
        # hand back what is left once the tag is gone, which may be nothing.
        return None, stripped if stripped != text.strip() else text

    match = CALL_JSON.search(stripped)
    if not match:
        return None, text
    if len(stripped) - len(match.group(0)) > 8:
        # Prose around it: an answer that mentions a call, not a call.
        return None, text
    try:
        parsed = json.loads(match.group(0))
    except ValueError:
        return None, text
    name = parsed.get('name')
    if not name:
        return None, text
    return ({'function': {'name': name,
                          'arguments': parsed.get('arguments') or {}}}, '')


def _printable(stream):
    """Make a Windows console survive an answer in somebody else's alphabet.

    The model answers in the language it was asked in, and the console here
    encodes with the locale codepage - cp1252 on this bench. Swedish and German
    are inside it and render correctly; an ohm sign, a Polish l-stroke or any
    Cyrillic is not, and the default error handler turns that into a
    UnicodeEncodeError that kills the answer after the measurement was already
    taken. Replacing the character loses a glyph; raising loses the reading.

    The codepage is left alone on purpose. Forcing UTF-8 would fix the encode
    and hand a legacy console mojibake for the characters it *can* display,
    which is a worse trade for the languages actually spoken at this bench.
    """
    try:
        stream.reconfigure(errors='replace')
    except (AttributeError, OSError, ValueError):
        pass
    return stream


def approx_tokens(text):
    """Rough but honest: about four characters per token for dense ASCII."""
    return max(1, len(str(text)) // 4)


class Chat:
    """One conversation, trimmed on the way out to the model.

    The history is kept whole locally - it costs nothing on this side, and a
    stubbed line is impossible to un-stub. What goes over the wire is the trim.
    """

    def __init__(self, client, toolbox, tools='read', keep=6, budget=0,
                 quiet=False, out=None):
        self.client = client
        self.toolbox = toolbox
        self.keep = keep
        self.budget = budget
        self.quiet = quiet
        self.out = out or _printable(sys.stdout)
        self.history = []
        self.turn_cost = []
        self.set_tools(tools)

    # ---- what the model is allowed to see ----------------------------------

    def set_tools(self, wanted):
        names = SETS.get(wanted)
        if names is None:
            names = tuple(n.strip() for n in str(wanted).split(',') if n.strip())
        known = {spec['name'] for spec in toolmod.TOOLS}
        unknown = [n for n in names if n not in known]
        if unknown:
            raise ValueError('no such tool: %s. Sets: %s'
                             % (', '.join(unknown), ', '.join(SETS)))
        self.tool_names = names
        self.schemas = toolmod.schemas(
            [spec for spec in toolmod.TOOLS if spec['name'] in names]) or None
        return names

    def tool_cost(self):
        """What the tool list alone costs, every turn, before any question."""
        return approx_tokens(json.dumps(self.schemas or []))

    def trim(self):
        """System prompt, stubbed history, recent turns whole.

        Tool results are where the tokens are, so they are what gets stubbed:
        the first line of a channel table is enough to remember that it was
        read, and the model can read it again if it matters.
        """
        head = self.history[:-self.keep] if self.keep else self.history
        tail = self.history[-self.keep:] if self.keep else []

        # The language line is built per turn from the last thing the user
        # actually typed, rather than asked of the model. See language.py: told
        # to work out the language itself, qwen2.5:14b answered a European
        # question in Chinese, Japanese and Thai. Naming it removes the step
        # that was going wrong.
        #
        # It changes only when the user changes language, so the cached prefix
        # survives an ordinary conversation and pays a reload exactly when the
        # conversation genuinely switched.
        asked = ''
        for message in reversed(self.history):
            if message['role'] == 'user':
                asked = message.get('content') or ''
                break
        sent = [{'role': 'system',
                 'content': SYSTEM + '\n' + language.instruction(asked)}]
        for message in head:
            content = (message.get('content') or '').strip()
            if message['role'] == 'tool':
                first = content.splitlines()[0] if content else ''
                sent.append({'role': 'tool',
                             'content': clip(first, 80) + ' [...]'})
            else:
                # Assistant tool_calls are dropped along with the results they
                # go with: a call whose answer has been stubbed is noise.
                sent.append({'role': message['role'],
                             'content': clip(content, 200)})
        sent.extend(tail)
        return sent

    def context_cost(self):
        return approx_tokens(json.dumps(self.trim())) + self.tool_cost()

    # ---- a turn ------------------------------------------------------------

    def ask(self, question, max_calls=6):
        """One question, however many tool calls it takes. Returns the answer."""
        if self.over_budget():
            return 'budget of %d tokens is spent; /clear or raise --budget' \
                % self.budget

        self.history.append({'role': 'user', 'content': question})
        answer = ''

        for _ in range(max_calls + 1):
            before = self.client.usage()
            message = self.client.chat(self.trim(), self.schemas)
            after = self.client.usage()
            self._meter(after['prompt_tokens'] - before['prompt_tokens'],
                        after['eval_tokens'] - before['eval_tokens'])

            message.pop('thinking', None)
            calls = message.get('tool_calls') or []
            answer = (message.get('content') or '').strip()

            # A tool call written as prose is still a tool call. qwen2.5 emits
            # one as text often enough to matter - measured here, "vad ar
            # temperaturen" came back as the literal string
            #
            #     {"name": "docs", "arguments": {"find": "temperature"}}
            #     </tool_call>
            #
            # with no tool_calls field, which this loop then printed as the
            # answer. The model was right about what to do; the shape was
            # wrong. Recovering it costs a JSON parse.
            if not calls:
                salvaged, answer = _salvage_call(answer)
                if salvaged:
                    calls = [salvaged]
                    message = dict(message, content='', tool_calls=calls)

            self.history.append(message)
            if not calls:
                break

            for call in calls:
                name = (call.get('function') or {}).get('name', '?')
                args = _arguments(call)
                result = self.toolbox.call(name, args)
                if isinstance(result, toolmod.Reported):
                    result = 'noted: %s' % result.note
                self._trace('  %s %s' % (name, _short(args)), str(result))
                self.history.append({'role': 'tool', 'tool_name': name,
                                     'name': name,
                                     'content': '%s: %s' % (name, result)})

        # An answer that hit the token cap stops mid-sentence, and a table that
        # stops mid-row reads as complete to everyone except a reader counting
        # rows. Say so rather than letting the cap look like the end.
        if getattr(self.client, 'truncated', False) and answer:
            answer += ('%s[cut off at --words %s. Ask again with more, or ask '
                       'for fewer channels.]'
                       % (chr(10), self.client.options.get('num_predict', '?')))
        return answer

    # ---- the parts that cost nothing ---------------------------------------

    def command(self, line):
        """A slash command. Returns text, or None if the line was not one.

        These exist because a model is a poor way to run a line of Python you
        already know you want to run - and at a bench, most of the time, you do.
        """
        if not line.startswith('/'):
            return None
        verb, _, rest = line[1:].partition(' ')
        rest = rest.strip()

        if verb in ('q', 'quit', 'exit'):
            raise SystemExit(0)
        if verb in ('help', '?'):
            return HELP
        if verb == 'py':
            return self.toolbox.call('run_python', {'code': rest})
        if verb == 'sh':
            return self.toolbox.call('run_command', {'cmd': rest})
        if verb == 'clear':
            self.history = []
            return 'context cleared'
        if verb == 'tools':
            if rest:
                self.set_tools(rest)
            return '%s (%d tok/turn)' % (', '.join(self.tool_names) or 'none',
                                         self.tool_cost())
        if verb == 'ctx':
            return '%d messages, next turn about %d tok in, %d of it tools' \
                % (len(self.history), self.context_cost(), self.tool_cost())
        if verb == 'cost':
            return self.cost_line()
        return 'no such command. /help'

    def over_budget(self):
        usage = self.client.usage()
        return bool(self.budget) and \
            usage['prompt_tokens'] + usage['eval_tokens'] >= self.budget

    def cost_line(self):
        usage = self.client.usage()
        total = usage['prompt_tokens'] + usage['eval_tokens']
        text = '%d calls, %d in + %d out = %d tok' % (
            usage['calls'], usage['prompt_tokens'], usage['eval_tokens'], total)
        return text + (' of %d' % self.budget if self.budget else '')

    def _meter(self, prompt_tokens, eval_tokens):
        self.turn_cost.append((prompt_tokens, eval_tokens))
        if not self.quiet:
            print('  [%d in / %d out]' % (prompt_tokens, eval_tokens),
                  file=self.out, flush=True)

    def _trace(self, head, result):
        if self.quiet:
            return
        print('%s -> %s' % (head, ' | '.join(result.splitlines()[:2])[:96]),
              file=self.out, flush=True)


def _arguments(call):
    args = (call.get('function') or {}).get('arguments')
    if isinstance(args, str):
        try:
            args = json.loads(args or '{}')
        except ValueError:
            return {}
    return args if isinstance(args, dict) else {}


def _short(args):
    text = ' '.join('%s=%s' % (k, v) for k, v in (args or {}).items())
    return clip(' '.join(text.split()), 60)


class NoBoard:
    """Stands in for the session when --no-board is given.

    A question about the code or the build does not need the serial port opened,
    and opening it locks the console for whoever else wants it. Any tool that
    reaches for the board gets a plain answer instead of a timeout.
    """

    board = property(lambda self: self._refuse())
    allow_writes = False

    def _refuse(self):
        raise RigError('this run was started with --no-board')

    def info(self, refresh=False):
        self._refuse()

    def close(self):
        pass

    def reset(self):
        pass


def parse(argv):
    import argparse

    parser = argparse.ArgumentParser(
        prog='dbg', description='Ask a local model about this board, cheaply.')
    parser.add_argument('question', nargs='*', help='ask and exit; omit for a prompt')
    parser.add_argument('--repl', action='store_true',
                        help='force the prompt loop even with piped input')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='answer only: no tool trace, no token meter')
    parser.add_argument('-t', '--tools', default='code',
                        help='read|code|pins|all|none or a comma separated list')
    parser.add_argument('-m', '--model', default='gemma4:12b',
                        help="ollama tag, or 'auto' to pick one from this"
                             " machine's cores, RAM and VRAM - see"
                             " coaxial_ollama/capability.py")
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a remote daemon; off by'
                             ' default, because the question carries the board'
                             ' with it')
    parser.add_argument('--words', type=int, default=180,
                        help='cap on generated tokens per turn')
    parser.add_argument('--format', dest='fmt',
                        help="'json' to make the answer machine readable. The"
                             " board tools are unaffected - they are already"
                             " schema checked - but a model told to answer in"
                             " JSON calls fewer of them, so -t none is usually"
                             " what you want with it")
    parser.add_argument('--keep-alive', default=None,
                        help="how long ollama holds the model, and with it the"
                             " cached prompt prefix. Default depends on the"
                             " mode: %s in a prompt loop, %s for one question,"
                             " because a question already answered is rarely"
                             " followed by another within the half hour. '0'"
                             " hands the VRAM back at once."
                             % (KEEP_ALIVE_REPL, KEEP_ALIVE_ONCE))
    parser.add_argument('--num-gpu', type=int, default=None,
                        help='layers on the GPU; the rest run on the CPU.'
                             ' Set for you by -m auto and by board_prompt.ps1')
    parser.add_argument('--num-ctx', type=int, default=8192)
    parser.add_argument('--keep', type=int, default=6,
                        help='recent messages sent whole; older ones are stubbed')
    parser.add_argument('--budget', type=int, default=0,
                        help='stop asking after this many tokens')
    parser.add_argument('--think', action='store_true',
                        help='let a reasoning model think; costs a lot of tokens')
    parser.add_argument('--file', action='append', default=[],
                        help='attach a clipped file to the first question')
    parser.add_argument('--chars', type=int, default=2000,
                        help='how much of each --file to attach')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    parser.add_argument('--no-board', action='store_true')
    parser.add_argument('--allow', default='python,cube-cmake',
                        help='programs /sh and run_command may launch')
    parser.add_argument('--allow-writes', action='store_true')
    return parser.parse_args(argv)


def attach(paths, chars):
    """Files as context, clipped. A 3000 line log is not a question."""
    blocks = []
    for path in paths:
        try:
            with open(path, encoding='utf-8', errors='replace') as handle:
                text = handle.read()
        except OSError as exc:
            blocks.append('%s: unreadable (%s)' % (path, exc))
            continue
        blocks.append('--- %s (%d chars, %d attached) ---\n%s'
                      % (path, len(text), min(len(text), chars),
                         clip(text, chars)))
    return '\n'.join(blocks)


def build(args):
    from .client import Ollama
    from .tools import Toolbox

    tag, gpu_layers = args.model, args.num_gpu
    if args.model == 'auto':
        from .capability import choose, probe
        picked = choose(probe())
        tag = picked.tag
        if gpu_layers is None:
            gpu_layers = picked.options.get('num_gpu')
        if not args.quiet:
            print('model: %s  (%s)' % (tag, picked.why))

    client = Ollama(tag, host=args.ollama_host,
                    num_ctx=args.num_ctx, num_predict=args.words,
                    think=True if args.think else False,
                    remote_ok=args.allow_remote,
                    keep_alive=keep_alive_for(args), fmt=args.fmt,
                    num_gpu=gpu_layers)
    if args.no_board:
        session = NoBoard()
    else:
        from coaxial_mcp.session import Session
        session = Session(args.port, args.baud, args.unit)

    allow = [a for a in args.allow.split(',') if a.strip()]
    toolbox = Toolbox(session, shell=Shell(allow), scope=Scope(),
                      allow_writes=args.allow_writes)
    chat = Chat(client, toolbox, tools=args.tools, keep=args.keep,
                budget=args.budget, quiet=args.quiet)
    return client, session, chat


def repl(chat):
    print('%s, tools: %s (%d tok/turn). /help, /q to leave.'
          % (chat.client.model, ', '.join(chat.tool_names) or 'none',
             chat.tool_cost()))
    if not sys.stdin.isatty():
        print('(reading commands from stdin)')
    while True:
        try:
            bouncing_prompt()
            line = input().strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        try:
            done = chat.command(line)
            print(done if done is not None else chat.ask(line))
        except SystemExit:
            break
        except (RigError, ValueError) as exc:
            print('%s: %s' % (type(exc).__name__, exc))
    print(chat.cost_line())


def main(argv=None):
    from .client import OllamaError

    args = parse(argv)
    # Before anything prints: every path out of here, including the error
    # branches below, goes through a console that may not hold the alphabet
    # the answer arrives in.
    _printable(sys.stdout)
    _printable(sys.stderr)
    question = ' '.join(args.question).strip()
    if not question and not args.repl and not sys.stdin.isatty():
        # `sed -n 1,40p log | dbg` is a question about a log. Draining stdin
        # here would also swallow the prompt loop's input, so --repl skips it.
        question = sys.stdin.read().strip()

    try:
        client, session, chat = build(args)
    except OllamaError as exc:
        # A refused host or a cloud tag is a wiring mistake, not a bench fault:
        # there is no prompt loop worth opening against a model we will not use.
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    interactive = args.repl or not question
    try:
        client.model = client.require_model()
    except OllamaError as exc:
        # Fatal for one question - there is nothing else to do. Not fatal for the
        # prompt loop: /py and /sh never touch the model, and being unable to
        # reach ollama is no reason to lose the shortest path to the board.
        if not interactive:
            print('ollama: %s' % exc, file=sys.stderr)
            return 2
        print('ollama: %s' % exc, file=sys.stderr)
        print('slash commands still work; questions will not.', file=sys.stderr)

    extra = attach(args.file, args.chars) if args.file else ''
    try:
        if question and not args.repl:
            answer = chat.ask('\n'.join(filter(None, (question, extra))))
            print(answer)
            if not args.quiet:
                print(chat.cost_line(), file=sys.stderr)
        else:
            repl(chat)
    except OllamaError as exc:
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            session.close()
        except RigError:
            pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
