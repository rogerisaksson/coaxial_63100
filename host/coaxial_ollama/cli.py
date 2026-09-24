"""The command line: what `dbg.py` and `board_chat` run."""
import argparse
import json
import sys
from contextlib import suppress

from . import language
from . import spinner as spin
from .capability import choose, probe
from .client import Ollama, OllamaError
from .iolog import IOLog
from .sandbox import Scope, Shell, clip, clip_ends
from .tools import Toolbox
from coaxial.comm import session as sessionmod
from coaxial.errors import RigError
from coaxial.simulated import SimulatedSession
from coaxial_mcp import detail, render
from coaxial_ollama.debug import Chat, _printable
from coaxial_ollama.words import PROMPT


# Two numbers, because the modes want opposite things.
KEEP_ALIVE_REPL = '30m'


KEEP_ALIVE_ONCE = '2m'


def keep_alive_for(args):
    """What the caller asked for, or what the mode implies."""
    if args.keep_alive is not None:
        return args.keep_alive
    return KEEP_ALIVE_REPL if args.repl else KEEP_ALIVE_ONCE



# The most of a piped or attached input that becomes part of a question.
INPUT_LIMIT = 6000


class NoBoard:
    """Stands in for the session when --no-board is given."""

    board = property(lambda self: self._refuse())
    allow_writes = False
    #: The session's surface with nothing behind it - what every tool
    #: reads before it reaches for the board.
    port = bus = unit = attached = None
    simulated = False

    def _refuse(self):
        raise RigError('this run was started with --no-board')

    def info(self, refresh=False):
        self._refuse()

    def close(self):
        pass

    def reset(self):
        pass


def parse(argv):
    parser = argparse.ArgumentParser(
        prog='dbg', description='Ask a local model about this board, cheaply.')
    parser.add_argument('question', nargs='*', help='ask and exit; omit for a prompt')
    parser.add_argument('--no-compile', action='store_true',
                        help='skip the intent pass - one model call per turn,'
                             ' the behaviour before it existed')
    parser.add_argument('--repl', action='store_true',
                        help='force the prompt loop even with piped input')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='answer only: no tool trace, no token meter')
    parser.add_argument('-t', '--tools', default='code',
                        help='read|code|pins|build|docs|all|none or a comma '
                             'separated list')
    _model_arguments(parser)
    _turn_arguments(parser)
    _board_arguments(parser)
    _permission_arguments(parser)
    return parser.parse_args(argv)


def _model_arguments(parser):
    """Which model, where, and whether it may be far away."""
    parser.add_argument('-m', '--model', default='gemma4:12b',
                        help="ollama tag, or 'auto' to pick one from this"
                             " host's cores, RAM and VRAM - see"
                             " coaxial_ollama/capability.py")
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a remote daemon; off by'
                             ' default, because the question carries the board'
                             ' with it')


def _turn_arguments(parser):
    """What one turn may cost and how it is shaped: tokens, format, the
    model's residence, its language, the documentation each tool carries,
    the context, the history, the attachments."""
    parser.add_argument('--words', type=int, default=300,
                        help='cap on generated tokens per turn - 180 clipped '
                             'an open-ended question often enough to be '
                             'annoying; this costs a bit more per turn and '
                             'clips less often')
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
                             ' Set for you by -m auto and by board_chat.ps1')
    parser.add_argument('--lang',
                        help='answer in this language, whatever the host '
                             'is set to. Default: the Windows locale, moved '
                             'only by a question in another language or by '
                             'asking for one. /lang changes it mid-session.')
    parser.add_argument('--detail', default=detail.AUTO, choices=detail.LEVELS,
                        help='how much documentation each tool carries into '
                             'every turn. auto reads the model tag: terse for '
                             'the sizes this loop runs locally, full for '
                             'anything with room to read it. %s overrides for '
                             'the whole host.' % detail.ENV)
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


def _board_arguments(parser):
    """Which board, or no board, or an invented one."""
    parser.add_argument('--port', default='COM4',
                        help='tried first. If it is silent the debug '
                             'probe is looked for, then every other '
                             'port, then a simulated board - the '
                             'prompt tag says which answered')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    board_mode = parser.add_mutually_exclusive_group()
    board_mode.add_argument('--no-board', action='store_true',
                            help='stub the board tools out; every one refuses')
    board_mode.add_argument('--simulated', action='store_true',
                            help='board tools work, against an invented '
                                 'board that never opens a port - see '
                                 'coaxial.simulated')


def _permission_arguments(parser):
    """What the model's tools may launch, write and do unasked."""
    parser.add_argument('--allow', default='python',
                        help='programs /sh and run_command may launch. '
                             'Building and flashing does not need anything '
                             'on this list - see the build_firmware tool, '
                             "which is in the default `code` set and always "
                             'runs tools/target/build_and_flash.py regardless of '
                             '--allow.')
    parser.add_argument('--allow-writes', action='store_true')
    parser.add_argument('--confirm', action='store_true',
                        help='ask before every state change - a pin write, '
                             'run_python, run_command. Off by default, same '
                             'as board_chat without the flag; the two tools '
                             'this loop is actually built for, analog_read '
                             'and docs, are reads and never ask.')


def ask_operator(name, args):
    """The --confirm gate. Anything but y is a no, including a closed stdin."""
    print('\n  %s %s' % (name, json.dumps(args)[:400]))
    try:
        return input('  run it? [y/N] ').strip().lower() in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print('  declined')
        return False


def attach(paths, chars, limit=INPUT_LIMIT):
    """Files as context, clipped."""
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
    return clip_ends('\n'.join(blocks), limit)


def _auto_model(args, gpu_layers):
    """The tag this host runs, and its layer split unless one was asked."""
    picked = choose(probe())
    if gpu_layers is None:
        gpu_layers = picked.options.get('num_gpu')
    if not args.quiet:
        print('model: %s  (%s)' % (picked.tag, picked.why))
    return picked.tag, gpu_layers


def build(args):

    tag, gpu_layers = args.model, args.num_gpu
    if args.model == 'auto':
        tag, gpu_layers = _auto_model(args, gpu_layers)

    client = Ollama(tag, host=args.ollama_host,
                    num_ctx=args.num_ctx, num_predict=args.words,
                    think=True if args.think else False,
                    remote_ok=args.allow_remote,
                    keep_alive=keep_alive_for(args), fmt=args.fmt,
                    num_gpu=gpu_layers)
    # What the session talks to, and what the prompt says it talks to - one
    # decision, so the two cannot disagree.
    if args.no_board:
        session, origin = NoBoard(), ('no board', False)
    elif args.simulated:
        session, origin = SimulatedSession(), ('Simulated', False)
    else:
        session, found = sessionmod.open_session(args.port, args.baud, args.unit)
        origin = (found.label, found.real)

    allow = [a for a in args.allow.split(',') if a.strip()]
    toolbox = Toolbox(session, shell=Shell(allow), scope=Scope(),
                      allow_writes=args.allow_writes,
                      confirm=ask_operator if args.confirm else None)
    chat = Chat(client, toolbox, tools=args.tools, keep=args.keep,
                budget=args.budget, quiet=args.quiet,
                detail_level=args.detail,
                session_language=args.lang or language.system_language())
    chat.origin = origin
    return client, session, chat


def _greet(chat):
    """One line, in this host's language."""
    print(language.greeting(chat.client.model, chat.language,
                            getattr(sys.stdout, 'encoding', None)))
    if not ({'run_command', 'build_firmware'} & set(chat.tool_names)):
        # Printed once, here, by this host - not sent to the model, so it costs
        # nothing per turn.
        confirmed = ' and already --confirm' if chat.toolbox.confirm else \
                   ', then /confirm too, or it writes with nobody asking'
        print('  no build_firmware or run_command in this set - it cannot '
              'build or flash. /tools code (or build) switches now, no '
              'restart%s.' % confirmed)
    if not sys.stdin.isatty():
        print('(reading commands from stdin)')


def _turn(chat, face, line):
    """One line answered - a slash command, or a question to the model - and
    the answer printed under a stopped prompt.
    """
    asked = False
    try:
        done = chat.command(line)
        if done is None:
            asked = True
            # tools.py's afe_power gate, set from the question here, not in
            # ask(): a suite driving Chat.ask() keeps the permissive default.
            chat.toolbox.afe_mentioned = 'afe' in line.lower()
            chat.toolbox.asked = line
            # No note when the lock moves.
            done = chat.ask(line)
        # Stop ticking before the answer prints, not after.
        face.stop(chat.link_ok)
        print(done, file=face.out)
    except SystemExit:
        face.stop(chat.link_ok)
        return True
    except (RigError, ValueError, OllamaError) as exc:
        # A dead board and a dead model backend are the same shape of failure
        # here: something the session doesn't own crashed mid-turn.
        asked = True
        face.stop(False)
        print('%s: %s%s' % (type(exc).__name__, exc, render.hint(exc)),
              file=face.out)
    if asked:
        chat.history = []
    return False


def repl(chat, hold=False):
    _greet(chat)
    try:
        while True:
            # Read per prompt: /reconnect flips it mid-loop.
            tag, tag_ok = chat.prompt_tag()
            face = spin.prompt(PROMPT, sys.stdout, lock=chat.print_lock,
                               ok=chat.link_ok, tag=tag, tag_ok=tag_ok)
            chat.out = face.out
            try:
                line = input().strip()
            except (EOFError, KeyboardInterrupt):
                face.stop(chat.link_ok)
                print()
                break
            if not line:
                face.stop(chat.link_ok)
                continue
            face.busy()
            if _turn(chat, face, line):
                break
        print(chat.cost_line())
    finally:
        # The 30 min keep_alive serves a next turn and none comes: unload,
        # unless --keep-alive was given.
        if hold:
            chat.io_log.close()
        else:
            chat.close()               # unloads and closes the log


def ensure_pulled(client, out, pull_with=None):
    """The tag as `ollama list` spells it, pulled first when it is not
    there.
    """
    from . import pull as pulling
    try:
        return client.require_model()
    except OllamaError as exc:
        if 'not pulled' not in str(exc):
            raise
    (pull_with or pulling.pull)(client.model, host=client.host, out=out)
    return client.require_model()


def _one_question(chat, question, extra, quiet):
    """One question asked and answered on stdout, its cost on stderr."""
    full_question = '\n'.join(filter(None, (question, extra)))
    chat.toolbox.afe_mentioned = 'afe' in full_question.lower()
    chat.toolbox.asked = full_question
    print(chat.ask(full_question))
    if not quiet:
        print(chat.cost_line(), file=sys.stderr)


def main(argv=None):
    args = parse(argv)
    # Before anything prints, error branches included: the console may not
    # hold the answer's alphabet.
    _printable(sys.stdin)
    _printable(sys.stdout)
    _printable(sys.stderr)
    question = ' '.join(args.question).strip()
    if not question and not args.repl and not sys.stdin.isatty():
        # `sed -n 1,40p log | dbg` is a question about a log.
        question = clip_ends(sys.stdin.read().strip(), INPUT_LIMIT)

    try:
        client, session, chat = build(args)
    except OllamaError as exc:
        # A refused host or a cloud tag is a wiring mistake, not a bench fault:
        # there is no prompt loop worth opening against a model we will not
        # use.
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    # Real sessions only: suites call build() and write no file.
    chat.io_log = IOLog()
    # A typed sentence is the one input with ambiguity worth a second call.
    chat.compile_intent = not args.no_compile
    if args.simulated:
        # Said before the first answer; board_info says it too ("firmware":
        # "simulated"), after a tool call.
        print('SIMULATED - no port opened, every board reading is invented',
              file=sys.stderr)
    interactive = args.repl or not question
    try:
        client.model = ensure_pulled(client, sys.stderr)
    except OllamaError as exc:
        # Fatal for one question - there is nothing else to do.
        if not interactive:
            print('ollama: %s' % exc, file=sys.stderr)
            return 2
        print('ollama: %s' % exc, file=sys.stderr)
        print('slash commands still work; questions will not.', file=sys.stderr)

    # What the prompt's face shows: green once True, red once not.
    link_ok = not args.no_board
    chat.link_ok = link_ok

    extra = attach(args.file, args.chars) if args.file else ''
    try:
        if question and not args.repl:
            _one_question(chat, question, extra, args.quiet)
        else:
            repl(chat, hold=args.keep_alive is not None)
    except OllamaError as exc:
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        with suppress(RigError):
            session.close()
        # Unconditional, close() is idempotent: a one-shot question never
        # enters repl(), and `python dbg.py` with no question enters it with
        # args.repl False.
        chat.io_log.close()
    return 0
