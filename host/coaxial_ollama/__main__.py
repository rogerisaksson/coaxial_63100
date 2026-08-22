"""Command line entry point.

    python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
    python -m coaxial_ollama --ask "read the NTC and report degrees C"
    python -m coaxial_ollama --list-tools

Defaults: the model may read the board, run Python against it, and run programs
from an allowlist. It may not drive a pin or open the test gate until
--allow-writes, and --confirm puts a human in front of every side effect.
--read-only takes code and commands away entirely. On a board rated 63 V and
100 A that asymmetry is the point - an unattended run should not be able to
drive a pin - and it is honest about its limit: code that can reach `board` can
reach the pins, so --allow-writes is about the declarative tools, and trusting
code at all is the --read-only decision.

The exit code is the operator's: 0 when nothing failed, 1 when a step failed or
never finished, 2 when the run could not start at all.
"""
import argparse
import json
import sys

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

from coaxial.errors import RigError                  # noqa: E402
from coaxial_mcp.session import Session              # noqa: E402

from . import runner as runmod                       # noqa: E402
from .client import Ollama, OllamaError              # noqa: E402
from .plan import Plan, PlanError                    # noqa: E402
from .sandbox import Scope, Shell                    # noqa: E402
from .tools import TOOLS, Toolbox                    # noqa: E402

DEFAULT_MODEL = 'gemma4:12b'
DEFAULT_ALLOW = 'python'


def parse(argv):
    parser = argparse.ArgumentParser(prog='python -m coaxial_ollama',
                                     description=__doc__.splitlines()[0])
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--plan', help='YAML test plan')
    source.add_argument('--ask', help='one ad-hoc question, recorded not judged')

    parser.add_argument('--model', help='ollama tag; the plan may name one')
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a daemon on another machine;'
                             ' the prompts leave this bench if you do')
    parser.add_argument('--keep-alive', default='30m',
                        help='how long ollama holds the model between tasks,'
                             ' and with it the cached prompt prefix')
    parser.add_argument('--num-ctx', type=int, default=8192)
    parser.add_argument('--temperature', type=float, default=0.0)

    parser.add_argument('--port', default='COM4')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)

    parser.add_argument('--allow', default=DEFAULT_ALLOW,
                        help='comma separated programs run_command may launch')
    parser.add_argument('--allow-writes', action='store_true',
                        help='permit pin writes and the test gate')
    parser.add_argument('--read-only', action='store_true',
                        help='no code and no commands; board tools only')
    parser.add_argument('--confirm', action='store_true',
                        help='ask before every state change')
    parser.add_argument('--only', help='comma separated step ids')
    parser.add_argument('--max-turns', type=int,
                        help='override the plan, per step')
    parser.add_argument('--transcript', help='JSONL path; default data/')
    parser.add_argument('--no-transcript', action='store_true')
    parser.add_argument('--json', dest='json_out', help='write the summary here')
    parser.add_argument('--list-tools', action='store_true',
                        help='print the tool surface and exit')
    return parser.parse_args(argv)


def ask_operator(name, args):
    """The --confirm gate. Anything but y is a no, including a closed stdin."""
    print('\n  %s %s' % (name, json.dumps(args)[:400]))
    try:
        return input('  run it? [y/N] ').strip().lower() in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print('  declined')
        return False


def list_tools():
    for spec in TOOLS:
        print('%-12s %s' % (spec['name'], spec['description']))
    return 0


def build_plan(args):
    if args.plan:
        plan = Plan.load(args.plan)
    elif args.ask:
        plan = Plan.single(args.ask)
    else:
        raise PlanError('give --plan FILE or --ask "question"')

    if args.max_turns:
        for task in plan.tasks:
            task.max_turns = args.max_turns
    return plan


def main(argv=None):
    args = parse(argv)
    if args.list_tools:
        return list_tools()

    try:
        plan = build_plan(args)
    except (PlanError, OSError) as exc:
        print('plan: %s' % exc, file=sys.stderr)
        return 2

    try:
        client = Ollama(args.model or plan.model or DEFAULT_MODEL,
                        host=args.ollama_host, temperature=args.temperature,
                        num_ctx=args.num_ctx, remote_ok=args.allow_remote,
                        keep_alive=args.keep_alive)
        client.model = client.require_model()
    except OllamaError as exc:
        print('ollama: %s' % exc, file=sys.stderr)
        return 2

    allow = [a for a in args.allow.split(',') if a.strip()] + plan.allow
    session = Session(args.port, args.baud, args.unit)
    toolbox = Toolbox(session, shell=Shell(allow), scope=Scope(),
                      allow_writes=args.allow_writes or plan.allow_writes,
                      allow_code=not args.read_only,
                      confirm=ask_operator if args.confirm else None)

    path = None if args.no_transcript else (args.transcript
                                           or runmod.default_transcript_path())
    transcript = runmod.Transcript(path)
    runner = runmod.Runner(plan, client, toolbox, transcript)

    print(runmod.report_text({'plan': plan.header(), 'model': client.model,
                              'usage': client.usage(), 'counts': {},
                              'records': []}).rstrip())
    try:
        summary = runner.run(only=args.only.split(',') if args.only else None)
    except OllamaError as exc:
        print('\nollama: %s' % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('\ninterrupted; the transcript up to here is at %s' % path,
              file=sys.stderr)
        return 2
    finally:
        # Hand the UART back to the text console, or the board looks dead to
        # anyone who opens a terminal on it next.
        try:
            session.close()
        except RigError:
            pass
        transcript.close()

    print(runmod.report_text(summary))
    if path:
        print('transcript  %s' % path)
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2, default=str)
        print('summary     %s' % args.json_out)

    bad = sum(summary['counts'].get(k, 0)
              for k in ('fail', 'unfinished', 'error'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
