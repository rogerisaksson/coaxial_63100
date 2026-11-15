# The robot's layers live

`machine.process.introspect` serves the robot's loop's three layers each law
pass - latent (its state's rows by mode, its latent out), impedance (the modal
ask, the step's ask), wrench (the step's answer: tau, qacc, bears, cop, ..) - as
JSON lines on 127.0.0.1 to every client attached (`show_humanoid --telemetry
PORT`, `python -m machine.process.introspect --attach PORT`, `introspect.read`).
Every field named from the robot's code's own constants (`introspect.layout`:
the run of module-level slices ending at a vector's size, in rising offsets -
the ask's 37 to `modal.ASK`, latent out's 14 to `latent.OUT`, a row's 12 to
`ortho.P`, each gapless), a field's entries by the tuple its own definition
counts (`introspect.counted`: `len(SKIN)` in TOUCH's, through `latent.N` to
`ortho.MODES`), none named from its width alone. Paced on the robot's walk, the
fastest of interleaved 1 500-pass chunks: 615 us a pass without the hook, 604
with it and none attached (a check at its first line), 875 with a client reading
every law pass - the snapshot and its JSON 0.26 ms a pass, under real time's
1 000. A slow client is dropped behind, never waited on (64 queued).

## Reduced for a reader with little room

`machine.process.watch` runs the robot's analyzers (`machine/analyzers`, a file
each: `laid(meta)`, `step(state, snap) -> events`) beside its loop over
the snapshots - its law's layers named only as an analyzer reads them
(`introspect.Snap`, 11 us where 134) - and hands on only their events; its
folder read again each second, a file saved reloaded, one removed dropped, one
that raises told and held off while the rest fire on. Between snapshots
`introspect.Tap` holds every pass's fastest joint (a peak detector): a snapshot
a law pass alone read the tip's fastest 249.7 deg/s where every pass's 270.7;
held, each closed phase's fastest is every pass's to the digit (kneel right
elbow 333.1, tip right gripper 270.7 - the probes' reduction, test_introspect).
Triggers are the robot's body's own - a phase's flip, a doing back after a blip
under its natural time (chatter), its soles' share crossing half
and holding there its natural time (a landing's bounce about half fired
15 times in 0.1 s undebounced), a fall, a joint past its drive's rated speed, a
say refused, an errand's phase; those marked capture a gate's triggers (below).
On the page as the user runs it (`--telemetry`, the prompt typed): 'lay on
floor' 8 events in 16 s, the probe's phases and spans (kneel 2.502 s, tip 1.5);
'do a backflip' through the local model, a fall at 3.011 s, its fastest the
right foot 956 deg/s at 3.071; 'sit in the chair' sat at 16.97 in one run and
fell in another at 6.1 s after a turn mid-walk, the robot's right hip 0.94 of
its drive's rating walking (TODO).

## Gated

`machine.rig.gate` records the robot's streams on their own clocks while a gate
is armed (`streams.STREAMS`): the commands heard; its edge's reads - IMU,
encoders and drives' torques each pass, soles and skin each law pass; its things
at its registry's refresh (9 ms); its law's latent, impedance and wrench each
law pass; its drives each pass; the world the pass left; its wear as told; its
events. A trigger - an analyzer's event named, else one marked capture - takes
half the robot's leg's swing before it and after (`takes.ENTER_S`, 0.88 s); the
gate armed again till its count, a trigger in an open window held off. The
recorder's file is gate after gate: a metadata line (the trigger's name, tag and
event, the window, each stream's maker and words, kind, period, the most it
fits, its fields' offsets and the names their entries are), its records in time,
the streams interleaved and one instant's in the robot's pass's order, a trailer
(counts, held off, a cut). Against a harness recording each subsystem where it
runs - the robot's edge's reads, its law's control, its drives,
the world's step, its registry's look, its wear, the commands -
every gated section of the 11 streams the same to the bit in two gates of the
robot's floor lie; of 10 bugs planted 8 were caught, the 2 passing equivalent
(the robot's law's ask and answer new each law pass), a reused buffer aliased
(the robot's latent state) caught (test_gate). On the page, held against the
snapshot stream of the same run: angles, peaks between snapshots, latent rows,
the modal ask and the step's answer the same in 1 511 snapshots of three gates
of 'lay on floor'. A gate is 16 MB: 1 761 records of each kHz stream, 587 of
each law stream. A gate's JSON encoded beside the robot's loop held the robot's
passes 0.4 s a gate; a writer's process fed through a pipe takes it in 36 ms.
Armed, a pass costs 15 % more in-process; the page's pace 0.74 of real time
idle, 0.58 gating, the robot's world's steps unchanged. The robot's page's
worker is a daemon process: a multiprocessing writer raised in its pass
and ended the page; a fault of its telemetry is now told to its clients,
its gates dropped. A small tour of the cabin with a gate armed from a shell: 93
lines streamed in 90 s, a second behind the robot; its fall at 47.784 s standing
up from the floor gated, 5 671 records, 11.6 MB - its law's and drives' streams
end at the fall, its edge's reads and the world run on. The robot's fingers,
held a fist at 80 deg (its build), were laid at the stand's 18 and
snapped in at 893.5 deg/s on every lay, its wrists held at 0 laid at 5
at 69.1 - every scene's first phase's fastest joint in the tour's stream; the
world lays a pose within each joint's stops now (`physics.World.reset`): 0.4 and
3.8 deg/s, its walk as it was (0.70 m/s, 408 J/m, 2.50 off a woman's);
the robot's held standing jump steps 0.59 m from its landing where 0.24 - its
crouch taps the robot's soles off (scenarios 6).

## The robot's observer and its calls

A snapshot names the robot's observer (`observer.named`): each manifold's
estimate against its skin (`observer.skin`), its rows' told, weighed, latch and
gate, each point's push, prior and speed, its barrier's corners. Traced
(`calls.Calls`, `sys.monitoring`), it carries the functions and C cores the
robot's law called since the last: 201 code objects on its walk, 1 678
us a pass where 1 146. On the floor scoot, the first calls of the robot's
shield's QP (`schema._evaded`) came at 6.53 s; MuJoCo had its thighs on its
torso from 3.20 s, 1 724 N at 6.42.

## The page's errands

- Said 'sit in the chair' at the page's start the robot turned mid-walk at
  5.9 s, braked and fell at 6.1 in one run and sat at 16.97 in the next -
  its right hip 905 deg/s walking, 0.94 of its drive's rating.
- The page's tour (`machine.way.tour`) says rise_chair 8 s after sit_chair
  and rise_bed 9 s after lie_bed while their errands still walk: both
  refused, 0.5 m and 1.0 m off their seats.
