# Architecture: Tests

How the tree is held together: sizes measured, the cores built through gcc,
its structure, the relay, tiers, CI. Each subject's suites: its document.

- Sizes live in `host/tests/.counts.json` (measured each run); no document
  quotes them.
- Portable C cores are built with host gcc and driven through ctypes
  (`test_<core>_core.py`).
- `test_structure.py` holds host/ together and the wire's three answers to
  each other: `cmd.h`/`boot.h` vs `protocol.py`, every fixed reply and
  request vs the decoders, PROTOCOL.md's op tables vs the handlers, host
  mirrors of firmware constants vs the C.
- Suites that touch only the stand-in run four at a time (offline gate
  ~142 s); mcp, parity, bench, conformance, live run alone after.
- Tiers: `run_tests.ps1` sells checks by percentage; suites join by
  seconds per check. `touches.TOUCHES` maps a changed path to its suites;
  `CHEAP` ones settle without asking the model.
- CI: `host.yml` runs `--offline` on Python 3.12 and 3.14; `firmware.yml`
  builds both images in Debug and Release, fails on any warning, keeps the
  ELFs.

```text
tools/dev/          run_tests, pick_tests, counts, host_map, target_map,
                    warm_model, lint (markdownlint + pyright, the hooks),
                    measure_first (the prompt hook: a movement described, measured),
                    ab (electronics.md), focus (a suite's tests by name, its
                    watchdog, the relay: a baton a physical core, a queue longest
                    first, as the free commit holds)
tools/cores/        build: the portable cores' gcc build; each core's ctypes
                    harness in its subject's document
tests/              suites, .counts.json (measured sizes)
```
