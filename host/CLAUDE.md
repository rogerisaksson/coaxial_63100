# host/

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
