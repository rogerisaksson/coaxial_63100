"""The command line: what `dbg.py` and `board_chat` actually run.

Argument parsing, the session and client it builds, the prompt loop, and
the one-shot question. The turn itself is `debug.Chat`; this module
decides what that object is handed and what happens around it.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import RigError                  # noqa: E402
from coaxial_mcp import detail                       # noqa: E402
from coaxial_mcp import render                       # noqa: E402
from . import language                               # noqa: E402
from . import spinner as spin                        # noqa: E402
from .client import Ollama, OllamaError              # noqa: E402
from .debug import Chat, PROMPT, _printable          # noqa: E402
from .iolog import IOLog                             # noqa: E402
from .sandbox import Scope, Shell, clip, clip_ends   # noqa: E402


# Two numbers, because the modes want opposite things. A prompt loop is about
# to be asked again and the cached prefix is worth 8 GB of VRAM; a one-shot is
# not - measured, it left 9.69 GB resident for 27 minutes at 1 % use.
KEEP_ALIVE_REPL = '30m'


KEEP_ALIVE_ONCE = '2m'


def keep_alive_for(args):
    """What the caller asked for, or what the mode implies."""
    if args.keep_alive is not None:
        return args.keep_alive
    return KEEP_ALIVE_REPL if args.repl else KEEP_ALIVE_ONCE



# The most of a piped or attached input that becomes part of a question.
# `sed -n 1,40p log | dbg` is a question about a log; `cat build.log | dbg`
# is the same command with fifty thousand lines behind it, and nothing about
# the pipe says which one arrived.
INPUT_LIMIT = 6000


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
    parser.add_argument('-m', '--model', default='gemma4:12b',
                        help="ollama tag, or 'auto' to pick one from this"
                             " machine's cores, RAM and VRAM - see"
                             " coaxial_ollama/capability.py")
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a remote daemon; off by'
                             ' default, because the question carries the board'
                             ' with it')
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
                        help='answer in this language, whatever the machine '
                             'is set to. Default: the Windows locale, moved '
                             'only by a question in another language or by '
                             'asking for one. /lang changes it mid-session.')
    parser.add_argument('--detail', default=detail.AUTO, choices=detail.LEVELS,
                        help='how much documentation each tool carries into '
                             'every turn. auto reads the model tag: terse for '
                             'the sizes this loop runs locally, full for '
                             'anything with room to read it. %s overrides for '
                             'the whole machine.' % detail.ENV)
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
    parser.add_argument('--allow', default='python',
                        help='programs /sh and run_command may launch. '
                             'Building and flashing does not need anything '
                             'on this list - see the build_firmware tool, '
                             "which is in the default `code` set and always "
                             'runs tools/build_and_flash.py regardless of '
                             '--allow.')
    parser.add_argument('--allow-writes', action='store_true')
    parser.add_argument('--confirm', action='store_true',
                        help='ask before every state change - a pin write, '
                             'run_python, run_command. Off by default, same '
                             'as board_chat without the flag; the two tools '
                             'this loop is actually built for, analog_read '
                             'and docs, are reads and never ask.')
    return parser.parse_args(argv)


def ask_operator(name, args):
    """The --confirm gate. Anything but y is a no, including a closed stdin."""
    print('\n  %s %s' % (name, json.dumps(args)[:400]))
    try:
        return input('  run it? [y/N] ').strip().lower() in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print('  declined')
        return False


def attach(paths, chars, limit=INPUT_LIMIT):
    """Files as context, clipped. A 3000 line log is not a question.

    Two limits, because --chars only ever bounded one file: ten of them at
    the default 2000 is 20k characters of attachment in front of a one-line
    question, which is the whole window before the board has been asked
    anything. The second bound is on the lot of them together.
    """
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


def build(args):
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
    # What the session talks to, and what the prompt says it talks to - one
    # decision, so the two cannot disagree. With no flag the port is probed
    # and a silent one falls back to the stand-in rather than failing every
    # call: a bench without the cable in is a session about the code, and it
    # should still run.
    if args.no_board:
        session, origin = NoBoard(), ('no board', False)
    elif args.simulated:
        from coaxial.simulated import SimulatedSession
        session, origin = SimulatedSession(), ('Simulated', False)
    else:
        from coaxial_mcp.session import open_session
        session, found = open_session(args.port, args.baud, args.unit)
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


def repl(chat, hold=False):
    # One line, in this machine's language. What the tools are, what the
    # detail level is and what a turn costs are all a /help away; printed on
    # the way in they were three lines nobody read twice.
    print(language.greeting(chat.client.model, chat.language,
                            getattr(sys.stdout, 'encoding', None)))
    if not ({'run_command', 'build_firmware'} & set(chat.tool_names)):
        # Printed once, here, by this host - not sent to the model, so it
        # costs nothing per turn. Measured: asked three times
        # running to build and flash, on the default `code` set - before it
        # carried build_firmware - the model correctly and repeatedly said
        # it could not: accurate, but a dead end with no way out of it short
        # of already knowing this flag. `code` carries build_firmware now;
        # this only still fires for `read`, `pins` or a custom list missing
        # both.
        confirmed = ' and already --confirm' if chat.toolbox.confirm else \
                   ', then /confirm too, or it writes with nobody asking'
        print('  no build_firmware or run_command in this set - it cannot '
              'build or flash. /tools code (or build) switches now, no '
              'restart%s.' % confirmed)
    if not sys.stdin.isatty():
        print('(reading commands from stdin)')
    try:
        while True:
            # Read fresh every time, not captured once: /reconnect flips this
            # mid-loop and the very next prompt is what should show it. The
            # lock is shared with Chat._trace() so a tick and a trace line
            # printed mid-question never interleave on the same stream, and
            # chat.out is pointed at the same tracked stream so the prompt
            # knows how many rows whatever _trace() prints actually add -
            # not a number decided once and trusted for the whole question.
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
            asked = False
            try:
                done = chat.command(line)
                if done is None:
                    asked = True
                    # See tools.py's afe_power gate: set from the real
                    # question text, here rather than inside ask() itself,
                    # so a scripted test driving Chat.ask() directly keeps
                    # its old, permissive default instead of needing "afe"
                    # in every unrelated fixture question.
                    chat.toolbox.afe_mentioned = 'afe' in line.lower()
                    chat.toolbox.asked = line
                    # No note when the lock moves. It used to print
                    # "sprak: bytt till Swedish (last)" above the
                    # answer - a host line, in a mix of two languages,
                    # saying what the answer itself already shows by
                    # being in the new one. A bare switch answers
                    # "Okej" and nothing else, without a model turn.
                    done = chat.ask(line)
                # Stop ticking before the answer prints, not after. stop()'s
                # own repaint climbs back to the prompt row by the same
                # newline count _paint() uses, and a long answer with no
                # embedded '\n' that the terminal itself wraps across two
                # or more rows is invisible to that count either way - the
                # difference is *when* the wrong climb can land on top of
                # the answer. Frozen first, the climb happens while nothing
                # but the prompt's own row exists below it; done after, the
                # same wrong "one row up" lands mid-answer instead, which is
                # exactly what a bench session saw: the prompt group spliced
                # into the middle of a sentence. The exception branch below
                # already stops before it prints - this makes the ordinary
                # answer match it, rather than being the odd one out.
                face.stop(chat.link_ok)
                print(done, file=face.out)
            except SystemExit:
                face.stop(chat.link_ok)
                break
            except (RigError, ValueError, OllamaError) as exc:
                # A dead board and a dead model backend are the same shape of
                # failure here: something the session doesn't own crashed
                # mid-turn. One bad turn is not a reason to lose the rest of
                # the conversation - ollama respawns llama-server on the next
                # request, same as the board answers again once reconnected.
                asked = True
                face.stop(False)
                print('%s: %s%s' % (type(exc).__name__, exc, render.hint(exc)),
                      file=face.out)
            if asked:
                # Every question starts from nothing, on purpose: a growing
                # history is a growing prompt, and a growing prompt is more
                # for llama-server's own prompt cache to hold onto right up
                # to the std::bad_alloc it has crashed with more than once
                # this session. A slash command never touched history in the
                # first place, so it is left alone here.
                chat.history = []
        print(chat.cost_line())
    finally:
        # The 30-minute keep_alive that makes turn nine as quick as turn two
        # is exactly wrong once there is no turn ten coming. Measured on this
        # bench: a session left running unattended held 9.69 GB for another
        # 27 minutes at 1% utilisation. `--keep-alive` is how to say "no,
        # really, leave it" - anything explicit there means the operator
        # already decided, and this leaves that alone.
        if hold:
            chat.io_log.close()
        else:
            chat.close()               # unload AND the log - one definition


def main(argv=None):
    args = parse(argv)
    # Before anything prints: every path out of here, including the error
    # branches below, goes through a console that may not hold the alphabet
    # the answer arrives in.
    _printable(sys.stdin)
    _printable(sys.stdout)
    _printable(sys.stderr)
    question = ' '.join(args.question).strip()
    if not question and not args.repl and not sys.stdin.isatty():
        # `sed -n 1,40p log | dbg` is a question about a log. Draining stdin
        # here would also swallow the prompt loop's input, so --repl skips it.
        #
        # Clipped, and from both ends: the pipe carries whatever the operator
        # aimed at it, and a whole build log or a captured session arrives
        # exactly as easily as forty lines do. Unbounded, it is the largest
        # single thing that can reach the daemon in one go - trim() would
        # have to clip it later anyway, and doing it here means the notice
        # says so before the model ever sees the question.
        question = clip_ends(sys.stdin.read().strip(), INPUT_LIMIT)

    try:
        client, session, chat = build(args)
    except OllamaError as exc:
        # A refused host or a cloud tag is a wiring mistake, not a bench fault:
        # there is no prompt loop worth opening against a model we will not use.
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    # Real sessions only - build() itself is what dozens of tests call
    # through, and none of them should write a file to do it. See IOLog.
    chat.io_log = IOLog()
    # A typed sentence is the one input with ambiguity worth a second call.
    chat.compile_intent = not args.no_compile
    if args.simulated:
        # Loud on purpose, before the model ever answers a thing: board_info
        # says the same ("firmware": "simulated"), but a line here means
        # nobody has to ask a tool first to find out these readings are
        # invented, not measured.
        print('SIMULATED - no port opened, every board reading is invented',
              file=sys.stderr)
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

    # What the prompt's face shows: green once True, red once not.
    # --no-board counts as not - board tools fail there by design.
    #
    # Not probed here: link_diagnose and the link_error override cover a
    # dead link better than an eager connect did, and a one-shot question
    # no longer exits before it was ever asked. docs/MODELS.md.
    link_ok = not args.no_board
    chat.link_ok = link_ok

    extra = attach(args.file, args.chars) if args.file else ''
    try:
        if question and not args.repl:
            full_question = '\n'.join(filter(None, (question, extra)))
            chat.toolbox.afe_mentioned = 'afe' in full_question.lower()
            chat.toolbox.asked = full_question
            answer = chat.ask(full_question)
            print(answer)
            if not args.quiet:
                print(chat.cost_line(), file=sys.stderr)
        else:
            repl(chat, hold=args.keep_alive is not None)
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
        # Unconditional, and close() is idempotent: repl() closes on its own
        # way out, but a one-shot question never enters repl() at all, and
        # `python dbg.py` with no question enters it despite args.repl being
        # False. Guarding on args.repl got that last case wrong - harmlessly,
        # since the second close is a no-op, but only by accident.
        chat.io_log.close()
    return 0
