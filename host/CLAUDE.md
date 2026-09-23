# host/

- Map first, files second: `python tools/host_map.py [--api] [dir..]` (one
  line per module; `--api` adds public signatures). A module opens on a
  one-line brief of at most 100 characters; `test_structure` holds it.
- Front door: `Coaxial63100` (`coaxial/rig.py`); `device.daq`, `.imu`,
  `.angle`, `.thermal`, `.gates`, `.drive`, `.motion`, `board.boot`.
  `simulated_device=True` needs no cable.
- Interfaces `Acquisition`, `PolledSensor`, `GateControl`, `BootControl`:
  real + simulated implementations; add a method to both or neither.
- Refusals are the board's words (`u8 took` + text); the host validates only
  what stops a request being formed.
- Suites: `python -X utf8 tests/<suite>.py`; `.\run_tests.ps1` (~25 %,
  `-All`, `-Structure`, `-Scope X.py`); offline gate
  `python tools/run_tests.py --offline` (~2.5 min). Sizes live in
  `tests/.counts.json`; no document quotes them.
- A missing cable is not a failing suite (`open_session()` falls back to the
  stand-in). The model is loaded once per run; Ctrl+C is STOPPED (exit 130).
- Local model: `python dbg.py -m auto -q "..."`; `ollama ps` before loading
  (16 GB card); never a second client at another `num_ctx`.
- Bash blocked by the classifier: write `claude_do_it.ps1`, the user's
  `claude_watch.ps1` runs it, read `claude_do_it.log`; comment out physical
  steps once run.

## Terminal and UX

- `python -m terminal` (`coaxial_tty.ps1`): `terminal/loader.py` lists
  `terminal/pages/`. A page is one module - `HEADLINE KEY WHAT ORDER NAME`,
  optional `ITEMS`, `run(args, name)`; a new view is a new page, nothing
  else lists it.
- Views are `tools/show_*.py` on `stage.frame_of`, `stage.hud` and
  `screen.run_view`; their drawings are `coaxial/` (`machine dial gauges
  desk thermalmap`) and `coaxial/graphics/` (the board renderer).
- The drawing's top-left corner carries the frame rate and one frame's
  cost in ms (`stage.Corner`, `stage.rate_of(console)`), no box of its
  own; `run_view` ticks it, a page with its own loop (`menu.py`,
  `show_render.py`) ticks it after each update.
- Braille is judged in a raster, then by the bench: `COLUMNS=200 LINES=60
  python tools/show_X.py --simulated --frames 1 > f.txt` (the rotor
  observer and BOARD ATTITUDE take `--width --height`), then
  `python tools/ansi2png.py f.txt f.png`. Check the nominal size and a
  large terminal.
