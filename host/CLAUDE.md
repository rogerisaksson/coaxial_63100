# host/

- Map first, files second: `python tools/dev/host_map.py [--api] [dir..]` (one
  line per module; `--api` adds public signatures; `--layers` the import
  graph by package, `--deps` per module). A module opens on a one-line
  brief of at most 100 characters; `test_structure` holds it.
- Domains, imports one way: `motor/` (the PMSM; imports nothing here),
  `machine/` (the executive over IO nodes; imports no board), `coaxial/`
  (the inverter; imports both). Machine means the executive, motor the PMSM,
  host this computer (docs/ARCHITECTURE.md).
- Front door: `Coaxial63100` (`coaxial/rig.py`); on a real board `open()` first
  makes it run this host's build (`coaxial.devices.boot.load`, docs/BOOT.md);
  `device.daq`, `.imu`, `.angle`, `.thermal`, `.gates`, `.drive`, `.motion`,
  `board.boot`. `execution_mode=` HARDWARE (the board on `port`), SIMULATED
  (the stand-in), EMULATED (this host's image on Renode, `emulator://`);
  the last two need no cable. Any URL port (`fakeboard://`, `native://`,
  `emulator://`, `socket://`) is this process's own: no broker. `Machine.discover(type,
  execution_mode=VIRTUAL)`: no board, each joint where it is told
  (`machine.virtual`); `DYNAMIC`: the gynoid's figure in MuJoCo, each
  joint a drive (`machine.physics`), set every ms by `machine.walker`; the
  HUMANOID page runs her in her own process (`machine.running`).
- 3D pages raster on the GPU (`coaxial.graphics.gpu`, wgpu) where a card
  answers, else the process crew; `COAXIAL_GPU=0` forces the CPU.
- Interfaces `Acquisition`, `PolledSensor`, `GateControl`, `BootControl`:
  real + simulated implementations; add a method to both or neither.
- Refusals are the board's words (`u8 took` + text); the host validates only
  what stops a request being formed.
- Suites: `python -X utf8 tests/<suite>.py`; `.\run_tests.ps1` (~25 %,
  `-All`, `-Structure`, `-Scope X.py`); offline gate
  `python tools/dev/run_tests.py --offline` (~2.5 min). Sizes live in
  `tests/.counts.json`; no document quotes them.
- A missing cable is not a failing suite (`open_session()` falls back to the
  stand-in). `tests/test_emulator.py` runs the image on Renode (skips without
  it; `setup.ps1` installs it); `tools/emu/emulator.py --nodes N` a limb.
  The model is loaded once per run; Ctrl+C is STOPPED (exit 130).
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
- Views are `terminal/views/show_*.py` (a long one keeps its parts in a
  folder beside it: `rotor/`, `session/`) on `terminal/ui/`: `stage`
  (`frame_of`, `hud`), `screen` (`run_view`, `say`), `console` (`Keys`),
  `scroll`, `marquee`, `rate`, `aspect`. Their drawings are `coaxial/draw/`
  (`cross_section dial gauges desk thermalmap`) and `coaxial/graphics/` (the board
  renderer).
- The drawing's top-left corner carries the frame rate and one frame's
  cost in ms (`rate.Corner`, `rate.rate_of(console)`), no box of its
  own; `run_view` ticks it, a page with its own loop (`menu.py`,
  `show_render.py`) ticks it after each update.
- `coaxial_tty.ps1 -Emulated` runs the terminal on the emulated image (chip
  EMULATOR); `python tools/render/page.py PAGE --port emulator:// --png f.png`
  renders a page's last frame on it.
- Braille is judged in a raster, then by the bench: `COLUMNS=200 LINES=60
  python terminal/views/show_X.py --simulated --frames 1 > f.txt` (the rotor
  observer and BOARD ATTITUDE take `--width --height`), then
  `python tools/render/ansi2png.py f.txt f.png`. Check the nominal size and a
  large terminal.
