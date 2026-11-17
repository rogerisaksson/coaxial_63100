#!/usr/bin/env python3
"""What CI left red leads the suites that run first (`tools.dev.ci`): a tail read, the tuple laid, the
runner's work told; a job cut at its time says where it was, a suite asks the relay for its emulator, an ask of
the machine's ledger reads the machine once and a wide request keeps its turn (`tools.dev.focus`,
`tools.dev.ledger`); a job sent its emulator's cores is test_placed's.

No network: a gate's log tail as its comment holds it, and suites.py's own text. And a run fits the host it
is on (`tools.dev.machine_detector`): a suite's seconds on hosts laid here, this one measured. The gate's
workflow itself, its runner here and its parts are test_workflow's."""
import json
import os
import pathlib
import re
import sys
import tempfile
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A gate's log tail as CI's comment on a commit holds it: a suite red on a check, one crashed, the rest green.
TAIL = ('CI offline (3.14) failed. Log tail:\n\n~~~\n'
        'host: a runner, 2 cores, 4 threads, x86_64 Linux; 2.21 kernels a second a baton, 4 at once, 4.11 at 2\n'
        'test_structure.py    2557 passed, 0 failed  41.7s\n'
        'test_wasd.py         22 passed, 1 failed  479.6s\n'
        'test_sight.py        CRASHED exit=3221225477 12.0s\n'
        'test_wasd.py         22 passed, 1 failed  479.6s\n'
        'Total: ~2673  Passed: 2672, Skipped: ~0, Failed: 1, Crashed: 0, (7 of 125 suites ran)\n'
        '  test_wasd.py: no fall and no stumble on the keys from any start (13 spasms)\n~~~\n')


def test_a_log_tail_names_its_red_suites(report):
    """`ci.red`: the suites that failed a check or crashed, in their order, each once; a green tail none."""
    from tools.dev import ci
    report.check('a tail read: the suite red on a check and the one crashed, each once; the green ones not',
                 ci.red(TAIL) == ['test_wasd.py', 'test_sight.py'], ci.red(TAIL))
    report.check('a green tail names none', ci.red('test_structure.py    2557 passed, 0 failed  41.7s\n') == [])


def test_a_tail_tells_the_runner_s_work(report):
    """`ci.measured`: each suite's seconds in a tail by the pace of the runner its host line says - a crash
    by the time it ran -, the most of two runs' tails, none without a runner's pace; `ci.told`: the newest
    run that ended green or red, a cancelled one passed over."""
    from tools.dev import ci
    work = ci.measured(TAIL)
    report.check("a tail's suites by its runner's pace: 41.7 s, 479.6 s and a crash at 12.0 s at 2.21 kernels a second",
                 work == {'test_structure.py': 92.2, 'test_wasd.py': 1059.9, 'test_sight.py': 26.5}, work)
    other = TAIL + ('host: a runner, 2 cores, 4 threads, x86_64 Linux; 1.00 kernels a second a baton, 4 at once\n'
                    'test_sight.py        CRASHED exit=None 300.1s\n')
    logged = '\n'.join('offline (3.12)\tRun python tools/dev/run_tests.py --offline\t2026-11-11T16:53:34.1360273Z ' + line
                       for line in TAIL.splitlines())
    report.check("a job's whole log (gh run view --log) tells the same as its tail", ci.measured(logged) == work,
                 ci.measured(logged))
    report.check('two runs: the most each told; a tail without a runner\'s pace: nothing',
                 ci.measured(other)['test_sight.py'] == 300.1
                 and ci.measured(TAIL.replace(TAIL.splitlines()[3] + '\n', '')) == {}, ci.measured(other))
    runs = [{'conclusion': 'cancelled', 'headSha': 'a'}, {'conclusion': 'failure', 'headSha': 'b'},
            {'conclusion': 'success', 'headSha': 'c'}]
    report.check('the newest run that ended green or red, a cancelled one passed over; none of none',
                 (ci.told(runs) or {}).get('headSha') == 'b' and ci.told(runs[:1]) is None, ci.told(runs))


def test_a_job_cut_at_its_time_says_where_it_was(report):
    """`focus.run` in the relay's environment (`focus.environment`) on a job printing and then held past its
    time: what it printed comes back, not lost in a pipe's buffer - test_sight cut at 300 s on the runner
    said nothing -, on POSIX with its threads' stacks; its code None."""
    from tools.dev import focus
    job = focus.Job('held', [sys.executable, '-c', "print('-- held --'); import time; time.sleep(60)"], timeout=3.0)
    out, code, took = focus.run(job, focus.environment())
    report.check('a job cut at its time: what it printed and its code None, at its time',
                 '-- held --' in out and code is None and took < 30.0, '%r %r %.1f s' % (out[:80], code, took))
    report.check('on POSIX its stacks with it', ('most recent call first' in out) == (os.name != 'nt'), out[-200:])


def test_the_red_suites_lead_those_that_run_first(report):
    """`ci.led` on suites_failed.py's own text: CI's red suites at the head of FAILED_FIRST, the rest after in their
    order, none twice, the text about the tuple untouched; nothing laid where they lead already."""
    import ast
    from tools.dev import ci, suites, suites_failed
    with open(os.path.join(os.path.dirname(os.path.abspath(suites.__file__)), 'suites_failed.py'), encoding='utf-8') as f:
        source = f.read()
    was = list(suites_failed.FAILED_FIRST)
    lead = [was[-1], 'test_never_there.py']
    laid = ci.led(source, lead)
    now = next(ast.literal_eval(node.value) for node in ast.parse(laid).body
               if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'FAILED_FIRST')
    report.check("CI's red suites at the head of FAILED_FIRST, the others after in their order, none twice",
                 list(now) == lead + was[:-1], list(now)[:4])
    cut = lambda text: ci.FIRST.sub('', text)
    report.check('the text about the tuple untouched, its lines no wider than before',
                 cut(laid) == cut(source) and max(map(len, laid.splitlines())) <= max(map(len, source.splitlines())))
    report.check('suites that lead already, or none: the same text',
                 ci.led(source, was[:2]) == source and ci.led(source, []) == source and ci.led('x = 1\n', lead) == 'x = 1\n')


def test_a_suite_s_seconds_are_the_host_s(report):
    """`machine_detector.expected` on hosts laid here - a runner of four cores half as fast, a host twice as
    fast -: the tree's record by the pace it was run at, the table's work where it has none; and this host
    measured at two batons."""
    from tools.dev import machine_detector as m
    seconds, paces, work = {'test_own.py': 100.0, 'test_old.py': 30.0}, {'test_own.py': 8.0}, {'test_far.py': 1600.0,
                                                                                               'test_own.py': 1.0}
    slow, fast = (m.expected(seconds, paces, pace, work) for pace in (4.0, 16.0))
    report.check('run at 8 kernels a second, 100 s: 200 s on a host half as fast, 50 s on one twice as fast',
                 (slow['test_own.py'], fast['test_own.py']) == (200.0, 50.0), (slow['test_own.py'], fast['test_own.py']))
    report.check("a suite the tree has no record of: its work in the table over the host's pace; one with no pace "
                 'kept: as it stands', (slow['test_far.py'], fast['test_far.py'], slow['test_old.py']) == (400.0, 100.0, 30.0),
                 (slow['test_far.py'], fast['test_far.py'], slow['test_old.py']))
    new = m.expected(seconds, paces, 4.0, work, suites=('test_new.py', 'test_own.py'))
    report.check('a suite neither knows: as the heaviest they know till its first whole run, 400 s; one known: its own',
                 (new['test_new.py'], new['test_own.py']) == (400.0, 200.0), (new['test_new.py'], new['test_own.py']))
    here = m.host(2)
    report.check('this host measured: its cores, its threads, a pace at two batons',
                 here['cores'] >= 1 and here['threads'] >= here['cores'] and here['width'] == 2 and here['pace'] > 0.0,
                 m.said(here))


def test_the_gate_s_suites_have_their_work(report):
    """tests/work.json holds a row a suite of the offline gate - the gate a host with no record of its own
    runs, GitHub's runner: one without is planned as the heaviest suite, not as itself (laid from a gate
    stopped at its first red suite, the table had none of eleven). A new suite's row: run it whole, then
    `machine_detector.py --keep`."""
    from tools.dev import machine_detector, suites
    have = {*suites.NEEDS_BOARD, *machine_detector.table()}
    lacking = [name for name in suites.DEFAULT_SUITES if name not in have]
    report.check('every suite of the offline gate has its work in the table', not lacking, lacking)


def test_a_table_laid_keeps_what_was_not_run(report):
    """`machine_detector.keep` on a table and a record laid here: a suite the record has not keeps its row -
    read after the file was opened to be written, a run of eighteen suites left a table of 109 those
    eighteen - and a row is raised, never lowered: a tree's later run is lighter than its first."""
    from tools.dev import machine_detector as m
    record = {'seconds': {'test_own.py': 100.0, 'test_warm.py': 10.0, 'test_old.py': 30.0},
              'paces': {'test_own.py': 8.0, 'test_warm.py': 8.0}}
    with tempfile.TemporaryDirectory() as folder:
        path = pathlib.Path(folder, 'work.json')
        path.write_text(json.dumps({'test_far.py': 1600.0, 'test_own.py': 1.0, 'test_warm.py': 900.0}),
                        encoding='utf-8')
        told, held = m.keep(record, path), m.table(path)
    report.check('a suite run whole: its seconds by its pace; one with no pace: not told',
                 told == {'test_own.py': 800.0, 'test_warm.py': 80.0}, told)
    report.check('the table: the suite not run keeps its row, a heavier run raises one, a lighter run lowers none',
                 held == {'test_far.py': 1600.0, 'test_own.py': 800.0, 'test_warm.py': 900.0}, held)
    with tempfile.TemporaryDirectory() as folder:
        path = pathlib.Path(folder, 'work.json')
        path.write_text(json.dumps(held), encoding='utf-8')
        m.keep({}, path, {'test_far.py': 2000.0, 'test_ci.py': 5.0, 'test_own.py': 10.0})
        ran = m.table(path)
    report.check("what GitHub's runner measured raises a row and lays a new one, lowers none",
                 ran == {'test_far.py': 2000.0, 'test_own.py': 800.0, 'test_warm.py': 900.0, 'test_ci.py': 5.0}, ran)


def test_a_batch_fits_whose_host_it_is(report):
    """`tools.dev.background` and `machine_detector.chosen` on hosts laid here: a CI runner is nobody's desk -
    nothing kept, its cores a width and its every thread another, the one that does most work taken as
    measured; a desk keeps its user a third of its cores, four at most and two at least."""
    from tools.dev import background as b, machine_detector as m
    told = [b.runner(e) for e in ({'CI': 'true'}, {'CI': 'True'}, {'CI': '1'}, {}, {'CI': 'false'}, {'CI': ''})]
    report.check("a runner: CI true in a job's environment; a desk: unset, empty or false",
                 told == [True, True, True, False, False, False], told)
    desks = [(b.kept(cores, False), b.widths(cores, 2 * cores, False)) for cores in (2, 4, 8, 12, 32)]
    report.check('a desk keeps its user a third of its cores, four at most, two at least: one of 2, 2 of 4 and of 8, 4 of 12 '
                 'and of 32', desks == [(1, (1,)), (2, (2,)), (2, (6,)), (4, (8,)), (4, (28,))], desks)
    runners = [(b.kept(cores, True), b.widths(cores, threads, True)) for cores, threads in ((2, 4), (4, 4), (32, 64))]
    report.check('a runner keeps none: its cores a width, its every thread another where it has more',
                 runners == [(0, (2, 4)), (0, (4,)), (0, (32, 64))], runners)
    took = [m.chosen(paces) for paces in ({2: 3.38, 4: 1.95}, {2: 3.38, 4: 1.6}, {2: 3.0, 4: 1.5}, {28: 3.2})]
    report.check('the width that does most work is taken, the narrower where two do the same', took == [4, 2, 2, 28], took)
    here = m.host(1)
    report.check('this host tells whose it is as its environment does',
                 here['runner'] == b.runner() and m.said(here).startswith('host: a %s,' % ('desk', 'runner')[b.runner()]),
                 m.said(here))


def test_the_ledger_counts_a_container_s_cores(report):
    """`tools.dev.ledger`: a container's booking holds its cores in the ledger while its block runs, waited for while
    the width is taken - a second container of one core finds none left of two, and finds it once the first is gone;
    one that waits is in the queue the view lists, and takes its cores within a poll of the first letting go -;
    `focus.tally` tells the cores beside the jobs; `said` lists the machine, its containers with their tree, its
    queue; `clean` removes what a relay that is gone left: its booking, its ticket, its half-written file."""
    import threading
    from tools.dev import census, focus, ledger
    with tempfile.TemporaryDirectory() as folder:
        with ledger.reserved(2, 0.0, 'linux-3.12', ledger=folder, width=2, free=100.0) as first:
            held, cores = first.path is not None, focus.tally(folder)[3]
            with ledger.reserved(1, 0.0, 'linux-3.14', ledger=folder, width=2, wait_s=0, free=100.0) as second:
                refused = second.path is None
            lines = ledger.said(folder, width=2)
        with ledger.reserved(1, 0.0, 'linux-3.14', ledger=folder, width=2, wait_s=0, free=100.0) as third:
            again = third.path is not None
        after = focus.tally(folder)[3]
        queued, got = ledger.reserved(2, 0.0, 'linux-3.14', ledger=folder, width=2, wait_s=10.0, free=100.0), []
        with ledger.reserved(2, 0.0, 'linux-3.12', ledger=folder, width=2, free=100.0):
            runner = threading.Thread(target=lambda: got.append(queued.__enter__().path), daemon=True)
            runner.start()
            runner.join(1.5 * ledger.POLL_S)
            queue = [line for line in ledger.said(folder, width=2) if line.startswith('waiting: container linux-3.14')]
        runner.join(3 * ledger.POLL_S)
        queued.__exit__(None, None, None)
        dead = next(pid for pid in range(2 ** 22 + 1, 2 ** 22 + 1000) if not census.alive(pid))
        for name in ('.wait-%d-jobs' % dead, '.%d-8.part' % dead):   # 8: booking 7's own part file is consumed by its laying
            with open(os.path.join(folder, name), 'w', encoding='utf-8') as f:
                json.dump({'relay': dead, 'count': 3, 'since': 0.0}, f)
        focus.booked(os.path.join(folder, '%d-7' % dead), 0.3, None, dead)
        swept, left = ledger.clean(folder, client=False), sorted(os.listdir(folder))
    report.check('a container booked holds its cores; a second finds none left of two, finds them once the first is '
                 'gone; the view lists it', held and cores == 2 and refused and again and after == 0
                 and any(line.startswith('container linux-3.12') and '2 cores' in line for line in lines)
                 and lines[0].startswith('machine:') and '2 taken' in lines[0]
                 and len(queue) == 1 and '2 cores' in queue[0] and got and got[0] is not None
                 and any(os.path.basename(str(focus.ROOT.parent)) in line for line in lines if line.startswith('container'))
                 and len(swept) == 2 and left == [],
                 '%s %s %s %s %s | queue %s, got %s | swept %s, left %s' % (cores, refused, again, after, lines, queue, got, swept, left))


def test_a_suite_asks_the_relay_for_its_emulator(report):
    """`suites.asked`: the line a suite asks the relay with, read, not run - the bus suite's test on Renode a limb
    of three machines, the emulator suite's words its groups, a suite that asks none, none -, and `chosen` runs a
    suite's rest without the tests a `--but=` names."""
    from tools.dev import focus, suites
    tests = pathlib.Path(__file__).parent
    buses, emulator = suites.asked(tests / suites.BUSES), suites.asked(tests / suites.EMULATOR)
    one, two = (types.SimpleNamespace(__name__=name) for name in ('test_one', 'test_two'))
    rest = focus.chosen([one, two], ['--but=two'])
    # A line that names RENODE and reads as no ask - laid over two lines, say - would send the job no cores.
    unread = [path.name for path in sorted(tests.glob('test_*.py'))
              if re.search(r'^RENODE\b', path.read_text(encoding='utf-8'), re.M) and not suites.asked(path)]
    report.check('the bus suite asks for a limb of three by a word, the emulator suite by its groups, this suite for '
                 "none, every suite's RENODE line read; a suite's rest runs without what --but names",
                 buses == {'renode': 3} and bool(emulator) and set(emulator) <= set(suites.EMULATOR_GROUPS)
                 and suites.asked(__file__) == {} and unread == [] and rest == [one],
                 '%s %s %s, unread %s' % (buses, emulator, rest, unread))


def test_an_ask_of_the_ledger_reads_the_machine_once(report):
    """`focus.tally`: an ask of the ledger reads the machine's processes once (`census.Snapshot`), whatever is
    booked - read a booking, 28 jobs took 467 ms an ask under the ledger's lock -, and a relay asks once a second
    for all its batons that wait and when one of its jobs ends: six jobs one after another on six batons, five of
    them waiting, ask twice a job and once a second at most - a baton each, some six a job."""
    from tools.dev import census, focus
    with tempfile.TemporaryDirectory() as folder:
        for k in range(28):
            focus.booked(os.path.join(folder, '%d-%d' % (os.getpid(), k)), 0.3, os.getpid())
        before = census.Snapshot.count
        running = focus.tally(folder)[2]
        read = census.Snapshot.count - before
    jobs = [focus.Job('one after another %d' % k, [sys.executable, '-c', 'import time; time.sleep(0.5)'], 0.1, 60.0,
                      'one at a time') for k in range(6)]
    # A ledger of its own: asking the machine's from inside a job, this relay waited behind the whole gate's.
    with tempfile.TemporaryDirectory() as folder:
        before, began = census.Snapshot.count, time.monotonic()
        ended = [code for _job, _out, code, _took in focus.relay(jobs, batons=6, ledger=folder)]
        asks, took = census.Snapshot.count - before, time.monotonic() - began
    report.check('28 jobs booked read off one snapshot of the machine; a relay of six batons asks the ledger twice a '
                 'job and once a second at most, not once a baton',
                 read == 1 and running == 28 and ended == [0] * 6 and asks <= 2 * len(jobs) + took + 2,
                 'snapshots an ask %d, %d jobs told; %d asks in %.1f s, exits %s' % (read, running, asks, took, ended))


def test_a_wide_request_keeps_its_turn(report):
    """`focus.queued`: the cores the machine's wide requests wait for - a relay's job of more than one core, a
    container: their tickets, a relay's count of jobs and a dead owner's left out -, those waiting since before a
    given time alone, and without one's own; a container that asks after them takes no room they wait for
    (`ledger.reserved`) and its cores where the width holds both; the view tells how long each has waited."""
    from tools.dev import census, focus, ledger
    now = time.time()
    with tempfile.TemporaryDirectory() as folder:
        wide, box = (os.path.join(folder, '.wait-%d-%s' % (os.getpid(), name)) for name in ('wide', 'linux-3.12'))
        focus.waiting(wide, 4, now - 100.0)
        focus.waiting(box, 2, now - 50.0)
        focus.waiting(os.path.join(folder, '.wait-%d-jobs' % os.getpid()), 30, now - 100.0)
        dead = next(pid for pid in range(2 ** 22 + 1, 2 ** 22 + 1000) if not census.alive(pid))
        with open(os.path.join(folder, '.wait-%d-wide' % dead), 'w', encoding='utf-8') as f:
            json.dump({'relay': dead, 'count': 8, 'since': now - 200.0}, f)
        told = [focus.queued(folder), focus.queued(folder, mine=wide), focus.queued(folder, before=now - 75.0),
                focus.queued(folder, mine=box, before=now - 50.0)]
        with ledger.reserved(2, 0.0, 'linux-3.14', ledger=folder, width=7, wait_s=0, free=100.0) as short:
            passed_by = short.path is not None
        with ledger.reserved(2, 0.0, 'linux-3.14', ledger=folder, width=8, wait_s=0, free=100.0) as fits:
            beside = fits.path is not None
        ages = sorted(float(line.rsplit(', ', 1)[1].split()[0]) for line in ledger.said(folder, width=8)
                      if line.startswith(("waiting: a relay's wide job", 'waiting: container linux-3.12')))
    report.check("the wide requests' cores told - a relay's count of jobs and a dead owner's out -, one's own and the "
                 'later left out; a container asking after them takes no room they wait for, and its cores where the '
                 'width holds both; the view tells their ages',
                 told == [6, 2, 4, 4] and not passed_by and beside and len(ages) == 2
                 and all(abs(age - want) < 5.0 for age, want in zip(ages, (50.0, 100.0))),
                 '%s; in a width of 7 %s, of 8 %s; ages %s' % (told, passed_by, beside, ages))


ROSTER = (test_a_log_tail_names_its_red_suites, test_a_tail_tells_the_runner_s_work,
          test_a_job_cut_at_its_time_says_where_it_was, test_the_red_suites_lead_those_that_run_first,
          test_a_suite_s_seconds_are_the_host_s, test_the_gate_s_suites_have_their_work,
          test_a_table_laid_keeps_what_was_not_run, test_a_batch_fits_whose_host_it_is,
          test_the_ledger_counts_a_container_s_cores, test_a_suite_asks_the_relay_for_its_emulator,
          test_an_ask_of_the_ledger_reads_the_machine_once, test_a_wide_request_keeps_its_turn)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
