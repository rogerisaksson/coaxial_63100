# Host

- **Her loop's Python round MuJoCo** (asked): profiled walking and cut,
  each step bit for bit ([pace](../findings/pace.md)). Left, the largest
  first: `bodied`'s glue round her nine compiled parts (sixteen arrays fresh
  a law pass rebound, her sensing's 33 views laid a call), `modal.given` and
  the solve's wrapper (the ask a dict, copied into the step's struct),
  `borne.read` with nothing on her, the page worker's wear window (11 ms
  parts). Each sits in her law's own files or changes how a compiled part's
  result lives. DOD: her loop at the solve's and MuJoCo's cost alone, the
  glue under 10 % of a pass.
- **Branches out of her code** (asked): `if`/`else`/ternaries left, by module -
  the law's: wrench 14/4/15 (`wbc_wrench.c`'s Python: a sole's stance, the
  rounds, held contacts, the torque limit), wbc 11/0/4, leap 10/0/6, se2
  4/0/6, situation 5, borne 2, reach 1; the outer loop's: worker 54/1/23,
  tour 20/0/21 (the errand a table, `_on_rows`), sprung 11/2/4, room 9.
  DOD: flat tables and arithmetic a reader follows, a raise where she
  cannot; test_structure holds the law's modules at 0.
- **CI red, the host workflow**: test_sysid_live's sway, 3 checks (her
  insoles 0.315 N m rms apart, the moments 0.57 and 0.53, no first moment
  trusted) - at the sway's extreme her bare right sole lies 2.0 mm over the
  floor, `leap.TOUCH_M`, is let go at 3.10 s, lifted 15 mm and lands at
  3.34 s on 172 N (1f1d039d^: 1.9 mm, stood; red since 387bf4e6): the parry
  steps it in place, her capture point 2.5 cm past the one sole left; the
  band at the sole's give, a press, a pelvis list, the parry's support by
  drive or give and the clearance by reach each measured and not kept
  ([feet](../findings/feet.md), [stunts](../findings/stunts.md)); open: a
  let-go sole counting in the parry's support by its grip. The gate here the
  chair and snow wobbles at -2e-4 (scenarios items 1 and 3). On CI's Linux alone:
  test_errands' viewport chair (fell at 14.4 s) and bed (ends braking), on
  3.12 the chair borne 0.34 and the door at z 3.35 too; test_scenarios'
  chair at 0 rad (fell at 10.905 s, 3.14). DOD: both workflows green on
  main.
- **The autotuner at runtime** (asked): her loop and a tuner on another
  core with no lock between them - her samples on a ring, a double bank of
  compiled rows in shared memory -, a bank laid at her lateral voice's
  crossing with both soles locked or at rest, blended minimum-jerk (C2); the
  text's step from windows of her running walk; a converged step written into
  her texts. DOD: her walk tuned while she walks; with the tuner attached and
  no step taken her trajectory bit for bit and her pass's time as without
  it; a converged step written and committed.
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
- **Her surety field reads `out[latent.ZMP]`** (found): `latent.ZMP` indexes a
  parameter row (`ortho.P`), not `latent.OUT`; in `out` it lands on the
  drive mode's phase, so `w_air` is 1 - psi. Kept as is, bit for bit, by the
  C port. DOD: the read placed (the row's ZMP, or out's HELD), her walk and
  the scenarios measured before and after.
- **The orientation page is sluggish on Renode** (asked): the A1335 read back
  to back was the load, paced to 1 kHz with the drive and the DAQ off:
  Renode 2.2-3.5 -> 1.48 wall s a virtual s, the page's vectors 52 -> 78 a
  second (FINDINGS, A1335). Left: main() spinning for the keepalive's 200 kHz
  edges while AFE_ON is up, 1.34-1.48 of the 1.0 an idle board makes in WFI.
  PA10 is TIM1_CH3: a timer-driven pump re-armed by main() would let it sleep
  with the front end on - a change to the STO chain, HARDWARE.md's, to be
  measured on the probe first. DOD: the page at real time on Renode, Clevel
  on the probe as before.
