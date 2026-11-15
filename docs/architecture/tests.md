# Architecture: Tests

How the tree is held together: sizes measured, the cores built through gcc,
its structure, the relay, tiers, CI. Each subject's suites: its document.

- Sizes live in `host/tests/.counts.json` (measured each run); no document
  quotes them.
- Portable C cores are built with host gcc and driven through ctypes
  (`test_<core>_core.py`).
- `test_structure.py` holds host/ together: imports, cycles, names, shape,
  docstrings, the token budget. The wire's three answers: `test_mirrors.py`
  (`cmd.h`/`boot.h` vs `protocol.py`, host mirrors vs the C),
  `test_wire_shapes.py` and `test_wire_requests.py` (every reply and
  request vs the decoders), `test_protocol_doc.py` (PROTOCOL.md's op tables
  vs the handlers).
- Suites run on the relay, as many at once as the host's width (`--jobs`, a
  baton a physical core); bench, pace, conformance and live alone
  (`ALONE`), mcp and parity one at a time beside the rest (`PORT`).
- A suite that starts Renode asks the relay for it in a line of its own
  (`RENODE = {word: machines}`, `suites.asked`): the tests the word picks run
  as a job of their own, let in when a core a machine fits the width beside
  its own, the cores sent along with it and Renode bound to them
  (`probes.Cored`); the suite's rest runs without them (`--but=word`). An
  emulator nobody asked for (a page's, a flight's, a tool's) takes free
  cores itself or any of the batch's, never its host's one core, and books
  them in the ledger.
- Tiers: `run_tests.ps1` sells checks by percentage; suites join by
  seconds per check. `touches.TOUCHES` maps a changed path to its suites;
  `CHEAP` ones settle without asking the model.
- CI: one workflow, `gate.yml`, twenty jobs a push laid out by its `workflow`
  job (tools/dev/workflow.py): `host` runs `--offline`
  on Python 3.12 and 3.14 in eight parts each (`--part k/8`, split by
  tests/work.json); `firmware` builds both images in Debug and Release, fails
  on any warning, keeps the ELFs; `arch` runs the C parity suites on CI's
  arm64 runner (`sve2`) and
  with `avx512` where the x86 runner has it; `--linux` runs the gate in
  containers shaped as CI's runner.
- The machine's ledger (`tools/dev/ledger.py`): every relay's jobs and
  every container of the gate booked in one place a machine, admitted while
  the batch's width (the cores less two or more kept for the desk) and the
  free commit hold, else queued; `run_tests.py --ledger` shows it, and each
  start cleans what relays that are gone left behind.

```text
tools/dev/          run_tests, pick_tests, counts, host_map, target_map,
                    warm_model, lint (markdownlint + pyright, the hooks),
                    measure_first (the prompt hook: a movement described, measured),
                    ab (electronics.md), focus (a suite's tests by name, its
                    watchdog, the relay: a baton a physical core, a queue longest
                    first, as the free commit holds), ci (`--green`: the last
                    commit CI left green), linux (CI's runner as containers),
                    ledger (the machine's ledger: bookings, the queue,
                    `--ledger`), census (the machine's cores, free commit and
                    processes, read once an ask), workflow (the gate laid out
                    over its machines),
                    gate (the offline gate on a commit, its own worktree),
                    pinned and depends (`constraints.txt` laid; what a machine
                    needs written where setup reads it, held to the lock),
                    relay_first (the hook: suites through the relay), named
                    (sentences opening on It or Its), atlas
tools/cores/        build: the portable cores' gcc build; each core's ctypes
                    harness in its subject's document
tests/              suites, .counts.json (measured sizes)
```
