# Host

- **The host workflow red on main**: on GitHub's runner the on-line
  identification (test_sysid_live) rises past its 0.4 N m at the right
  knee, +0.58; the gate stops there. DOD: both workflows green on main.
- **wbc_blas's matmul against numpy 2.5**: at matrix @ matrix numpy copies an
  operand BLAS cannot take and calls gemm; wbc_matmul runs its plain loop
  there, so a hand core handing it such a view parts from numpy in the last
  bits (wbc_step.c, wbc_rounds.c). The tracer copies as numpy does. DOD:
  wbc_matmul copies as numpy, held by a test on such views.
- **The robot's loop's Python round its compiled parts and MuJoCo**
  (asked): the glue between the solve, the law's compiled parts and
  MuJoCo's step, and the law's own statements between them
  ([pace](../findings/pace.md)). DOD: the loop at the solve's and MuJoCo's
  cost alone, the glue under 10 % of a pass; the walk's pass under 400 us
  with every step in; the trajectory's sha1 the same over the walk, the
  lie and a chair.
- **Branches out of the robot's code** (asked): `if`/`else`/ternaries
  left in the law's modules (wrench, wbc, leap, se2, situation, borne,
  reach) and the outer loop's (worker, tour, sprung, room). DOD: flat
  tables and arithmetic a reader follows, a raise where it cannot;
  test_structure holds the law's modules at 0.
- **The autotuner's write** (asked): no tuned step kept since one broke
  test_errands' viewport bed ([autotuner](../findings/autotuner.md)).
  DOD: a write held on the errands too (`tools/sim/gate.py`: the suites
  its files reach), written and committed.
- **native://**: a limb's boards turn a world on a fixed mount, as
  board/emu's; the robot's body is not wired to them (`modes.TIERS`), and
  an idle limb pays more a frame than the wire (emulation.md). DOD: the
  robot's balance and gait from the SIL.
- **Identification on the robot** (identification.md): the tree's
  campaign (`machine.identify.sysid`) through its buses once the
  SIL moves its figure; the hips' yaw and roll armature untold on the
  stand; a link's inertia about its centre as built under a correction.
  DOD: the tree's campaign on emulated boards end to end; a bench drive on
  COM4; a link's tensor corrected under a body's constraints.
- **The robot's law compiled from its Python** (asked): numpy's own steps
  traced on its arrays, emitted in C11 on `wbc_np.h`'s primitives at its
  layout and held bit for bit as built (`tools.cores.held`); the rest of
  its law ported by hand (`tools.cores.motion`). DOD: latent.step,
  support.couple and impedance.ask generated, bit for bit, as fast as
  their ports.
- **The crafts' laws compiled from their Python** (asked): the
  helicopter's, the aerobat's and the ornithopter's `Core.step`, scalar
  Python the numpy tracer refuses (dicts, attributes, branches, a
  namedtuple), their hot helpers ported by hand (`aero/test/harness.c`'s
  `aeh_*`); the quad's law is the firmware's `aero.c`. DOD: the three
  steps generated from their Python, bit for bit, the hand ports gone.
- **The papers on the stand-in against the emulator** (asked,
  `make_notebooks.py --compare`): the motion paper's stop ramp through the
  sensorless crossover trips the stand-in at overcurrent; the applications
  paper's servo and velocity cells trip the emulator (emulation.md). DOD:
  every paper alike on both but where a world's physics says otherwise
  (the machines paper's, on the stand-in).
- **The gate under coverage cut short** (`cover.py --readme`): it stops at
  its first red suite, one red a run, and docs/RESULTS.md carries an old
  run's tables. DOD: `cover.py --readme` whole, docs/RESULTS.md written by
  it.
- **The surety field reads `out[latent.ZMP]`** (found): `latent.ZMP`
  indexes a parameter row (`ortho.P`), not `latent.OUT`; in `out` it lands
  on a drive mode's phase (PSI), read beside out's HELD and kept bit for
  bit by wbc_surety.c. DOD: the read placed (the row's ZMP) or gone, the
  robot's walk and the scenarios measured before and after.
- **The orientation page slow on Renode** (asked): with AFE_ON up main()
  spins for the keepalive's 200 kHz edges where an idle board sleeps in
  WFI (emulation.md); a pump off TIM1_CH3 (PA10) changes the STO chain
  (HARDWARE.md; [bench](bench.md)'s STO chain). DOD: the page at real time
  on Renode, Clevel on the probe as before.
- **The buses' errors told apart** (asked): BUSES' modbus row counts `crc`
  alone on the SIL; on the emulated limb a refused arm and a write timed
  out or refused share `lnk` (`EmulatedLimb.badge`). DOD: the row names
  what each count is - frames refused by CRC, writes timed out or refused,
  arm refused - and none reads as a drive's trip, overcurrent or knock.
- **A corrupt request costs the bystanders more** (found): on Renode one
  request's bit flipped is that request alone unanswered, but each node of a
  limb of three counts 3-7 comm errors against 0 in a clean run
  ([emulation](../findings/emulation.md)). DOD: the reason in `modbus_rtu.c`'s
  framing after a bad frame, one count a node a corrupt frame.
- **A slot taken on a full map** (found): with every slot live, a thing
  matching none takes the least salient one (`registry.seen`), and the
  look's later things meet it by its old kind, place ahead and variance, its new
  half sizes alone - a new thing matched as the one it replaced. Reached at 256
  live slots only (the lodge's 24 never). DOD: a slot taken anew met as its new
  thing, held by a test on a full map.
- **The robot's prompt on the page's model** (MODELS.md): on gemma4:e4b
  test_live_gynoid passes 7 of 8, 'res dig' lying picking lie_floor; no
  case asks 'stand up from the chair' seated. DOD:
  tests/test_live_gynoid.py green 3 of 3 runs on `gynoid.MODELS`' model,
  a seated 'stand up from the chair' among its cases.
