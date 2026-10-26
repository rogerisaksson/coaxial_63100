# Her layers live

`machine.introspect` serves her loop's three layers each law pass - latent
(her state's rows by mode, her latent out), impedance (the modal ask, the
step's ask), wrench (the step's answer: tau, qacc, bears, cop, ..) - as JSON
lines on 127.0.0.1 to every client attached (`show_humanoid --telemetry
PORT`, `python -m machine.introspect --attach PORT`, `introspect.read`).
Every field named from her code's own constants (`introspect.layout`: the
run of module-level slices ending at a vector's size, in rising offsets -
the ask's 37 to `modal.ASK`, latent out's 14 to `latent.OUT`, a row's 12 to
`ortho.P`, each gapless), her modes and manifolds by `ortho.MODES` and
`ortho.MANIFOLDS`. Paced on her walk, the fastest of interleaved 1 500-pass
chunks: 615 us a pass without the hook, 604 with it and none attached (a
check at its first line), 875 with a client reading every law pass - the
snapshot and its JSON 0.26 ms a pass, under real time's 1 000. A slow client
is dropped behind, never waited on (64 queued).
