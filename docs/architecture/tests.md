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
- Tiers: `run_tests.ps1` sells checks by percentage; suites join by
  seconds per check. `touches.TOUCHES` maps a changed path to its suites;
  `CHEAP` ones settle without asking the model.
- CI: `host.yml` runs `--offline` on Python 3.12 and 3.14; `firmware.yml`
  builds both images in Debug and Release, fails on any warning, keeps the
  ELFs; `arch` runs the C parity suites on CI's arm64 runner (`sve2`) and
  with `avx512` where the x86 runner has it; `--linux` runs the gate in
  containers shaped as CI's runner.

```text
tools/dev/          run_tests, pick_tests, counts, host_map, target_map,
                    warm_model, lint (markdownlint + pyright, the hooks),
                    measure_first (the prompt hook: a movement described, measured),
                    ab (electronics.md), focus (a suite's tests by name, its
                    watchdog, the relay: a baton a physical core, a queue longest
                    first, as the free commit holds), ci (`--green`: the last
                    commit CI left green), linux (CI's runner as containers),
                    gate (the offline gate on a commit, its own worktree),
                    pinned and depends (`constraints.txt` laid; what a machine
                    needs written where setup reads it, held to the lock),
                    relay_first (the hook: suites through the relay), named
                    (sentences opening on It or Its), atlas
tools/cores/        build: the portable cores' gcc build; each core's ctypes
                    harness in its subject's document
tests/              suites, .counts.json (measured sizes)
```
