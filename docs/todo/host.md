# Host

- **Branches out of her code** (asked): `if`/`else`/ternaries left, by module -
  the law's: wrench 14/4/15 (`wbc_wrench.c`'s Python: a sole's stance, the
  rounds, held contacts, the torque limit), wbc 11/0/4, leap 10/0/6, se2
  4/0/6, situation 5, borne 2, reach 1; the outer loop's: worker 54/1/23,
  tour 20/0/21 (the errand a table, `_on_rows`), sprung 11/2/4, room 9.
  DOD: flat tables and arithmetic a reader follows, a raise where she
  cannot; test_structure holds the law's modules at 0.
- **CI red since the emulator's update**: test_sysid_live 3 since 1f1d039d
  (her insoles' weight 305.2 for 305.5 N at 0.315 N m rms,
  the moments under her pelvis 0.57 and 0.53 N m rms, her mass 31.15 for
  31.14 kg); the gate here 5347 of 5582 with these and the chair and snow
  wobbles at -2e-4 (scenarios items 1 and 3); on CI's Linux the chair at
  0 rad too (fell at 10.9 s rising, held here) and test_errands 4 of 18
  (the chair borne 0.34, the jamb, the viewport's chair and bed; 18 here).
  DOD: both workflows green on main.
- **native://**: a limb's world on a fixed mount as board/emu's. A frame on
  an idle limb of four 32 ms, the wire's 10: three boards awake t3.5 and each
  chain's 2 ms recovery at 1 us substeps (emulation.md). DOD: the body's
  balance and gait from the SIL.
- **Identification on the robot** (identification.md): her tree's campaign
  (`machine.sysid`) through her buses once the SIL moves her figure; the
  hips' yaw and roll armature untold on the stand; a link's inertia about
  its centre as built under a correction. As it is: a drive on its board
  (`coaxial.control.identify`, `tools/bench/sysid_drives.py`) - held, swung
  by the board's own loop, its DAQ's currents and A1335 the rows - reads its
  J, b and the gravity on its shaft against the world's. DOD: her tree's
  campaign on emulated boards end to end; a bench drive on COM4; a link's
  tensor corrected under a body's constraints.
- **Her law compiled from her Python** (asked): numpy's own steps traced on
  her arrays, each emitted in C11 on `wbc_np.h`'s primitives at its layout and
  held bit for bit as it is built (`tools.cores.held`). As it is: the
  regressor, her estimate, her sensing, her grip's feel and floor, her
  situation, her drive's cap, her ask's poses, her sensing flat and her
  soles' touch generated, the rest of her law ported by hand or numpy's -
  its `npmax`
  and `npmin` Python's tie of zeros where they stand for numpy's maximum and
  minimum (110 calls). DOD: latent.step, support.couple and impedance.ask
  generated, bit for bit, as fast as their ports.
- **The crafts' laws compiled from their Python** (asked): the helicopter's,
  the aerobat's and the ornithopter's `Core.step` scalar Python - dicts,
  attributes, branches, a namedtuple - that the numpy tracer refuses; 18.4,
  30.3 and 20.7 ms of 113, 128 and 114 a flown second, the drives' 50 kHz
  ticks 43 %; their hot helpers ported by hand (`aero/test/harness.c`'s
  `aeh_*`). The quad's law is the firmware's `aero.c`. DOD: the three steps
  generated from their Python, bit for bit, the hand ports gone.
- **The papers on the stand-in against the emulator** (asked,
  `make_notebooks.py --compare`): the motion paper's stop ramp through the
  sensorless crossover trips the stand-in at overcurrent, 2 runs of 4 before
  the night and 4 of 4 now; the applications paper's servo and velocity
  cells trip the emulator, a cell further or nearer run to run, on the
  night's tree as now. DOD: every paper alike on both but where a world's
  physics says otherwise (the machines paper's, on the stand-in).
- **Her simulator's pace against real time** (asked): her loop as the page
  runs it (`worker.stepped`) and bare (`tools/sim/ortho.py`) in simulated s a
  wall s, MuJoCo's step alone beside them; the bottlenecks profiled and cut,
  the code as readable, nothing of MuJoCo's state written for it. DOD: the
  numbers in docs/findings/body.md, the smoke and every suite the pass reaches
  green, her walk's measures unchanged.
