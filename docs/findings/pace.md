# Findings: pace

Its pace against real time on this laptop: a pass of its loop, MuJoCo's step
in it, its law's parts ported to C bit for bit, the page's tiers by the
machine ([body](body.md), [emulation](emulation.md),
[compiler](compiler.md)).

- Its worker's pass walking, the laptop, two trees interleaved by process
  time: 1 680 -> 1 460 us, 0.60 -> 0.69 of real time, its trajectory's sha1
  a pass the same over 9 s and its walk's smoke the same to the digit. Its
  wear's window had stood its 0.9-1.3 s every 4 s: 8 000 steps of np.cross
  on 3-vectors (`wear._hems`), in floats 250 ms, sysid's rows the rest. The
  referee's and its edge's contact loops are arrays - a contact's force its
  rows of `efc_force`, the cone elliptic -, the props' ids and its skin's
  looked up once, its heat's state arrays where lists went to and fro each
  step, `paired`'s index once, the shove's rows zeroed only while it acts.
  MuJoCo's step is 92 us of the pass; its law 2.7 ms every third
  (`impedance.ask` 1.0, the wrench step 0.6, its sensing 0.4, its heat
  0.2). The C step through its ctypes edge: 1 355 us a pass, 0.74; the law
  every fourth pass 1 180, 0.85; both 1 060, 0.94. CPython's JIT: slower.
- The pass's glue in C (`wbc/src/wbc_pass.c`, `tools.cores.wbc`, the arrays
  laid once and pointed to): its drives told (`law.drives`), its world's
  torques, pairs and meters (`physics.advance`, `drawn`), its IMU, encoders,
  soles and skin read (`edge.felt`, `edge.read`: a contact's force its rows
  of `efc_force`), its drives' heat (`heat.Heat`) - numpy's arithmetic
  operation for operation, its clip and its pairwise sum: the trajectory the
  same bit for bit, 1 530 -> 1 260 us a pass, 0.65 -> 0.79. The C wrench
  step's ctypes edge 180 -> 60 us (laid buffers, no np.cross); as the
  default, 0.83, it sat the chair 0 of 3 wobbles (bore 0.35 s of 1), fell
  in snow 3 of 3, hopped 0.85 m/s where 0.9 and fell turning at 0.69 m/s:
  the python step stays. libm here gives numpy's bits for sin, cos, tan,
  exp, log, atan2, sqrt, acos, asin, hypot and pow, tanh an ulp off.
- Its ask in C (`wbc/src/wbc_imp.c`, `wbc_stride.c`, `tools.cores.motion`):
  `impedance.ask` with modal's reading, poise, stride and leap.standing
  under it, numpy's orders in `wbc/inc/wbc_np.h` - (3,3)@(3,) and (2,3)@(3,)
  dgemv_t's tail contracted, fma(a2, x2, fma(a0, x0, a1 x1)), (2,2)@(2,)
  fma(a0, x0, a1 x1), dgemm's fma chain, a vector @ a stack and a 1-d dot a
  plain sum, einsum 'nia,ia->n' SSE's two lanes (20 000 draws each). 47 880
  asks over 14 scenarios, still in half of them, the same bit for bit; 1 280
  -> 8 us an ask; its pass 1 130 -> 785 us, 0.88 -> 1.27 of real time, its
  trajectory's sha1 its python's. The law core's own C ask (`tools.cores`),
  as fast, left its python's bits from 0.5 s on: cut. Its latent step and
  its upper body's coupling on numpy's own OpenBLAS (`wbc_law.c`,
  `wbc_blas.c`: numpy's matmul dispatch - gemm, syrk, gemv, ddot or its own
  loop by shapes and strides - and its linalg's dgesv on Fortran copies,
  4 000 random shapes the same): 41 022 steps and 41 011 couplings bit for
  bit. Bit for bit where numpy is the C's (`law.exact_host`: its orders and
  its libm, here UCRT's - whose pow(x, 2) is not x x in 1 of 2 000, numpy's
  for a scalar's square, its x x an array's); CI's runners parted 3 asks of
  9 463 on one - gcc's sincos for a sin and cos of one angle, glibc's own
  path at 0.855 < |x| < 2.43, built apart since -, 3 709 of 9 668 on
  another, numpy's kernels theirs: there within 1e-9. Its whole-body step
  too (`wbc_step.c`, `wbc_rounds.c`, the
  generated core's rounds cut): numpy's a[:, idx] is F-ordered, its .T C -
  `J6[:, act].T @ w` is dgemv_t, a sum's other order -, and its mass is read
  at each solve, its wear moving it as it finds a load; every solve of 12
  scenarios bit for bit, its pass 825 -> 645 us, 1.55 of real time. Left in
  the pass: its sensing, the wear's window 0.23 s every 4 s - it feeds its
  law (`wear.loaded`).
- Its pace against real time, this laptop, nothing else running: MuJoCo's
  step alone 74 us standing (nv 46, 17 contacts), 13.6 of real time; its loop
  (`sprung.passed`) 727 us a pass, 1.38, walking 690 and 1.45; the page's
  worker 731, 1.37, its wear 3 %. Of a pass MuJoCo 91 us, its ask and
  sensing 391, its drives' pass 162 - its surety field in python 155 us a
  call, four calls in three passes -, the world's glue 19, its drives told
  over the SIL buses 36. The field in C (`wbc_surety.c`, `tools.cores.surety`:
  CPython's vector_norm for math.hypot, C's for numpy's; pow for a scalar's
  square, x x for an array's), bit for bit at every call of its law over
  12 s of its walk (test_compiled): its drives' pass 162 -> 20 us. The
  whole-body step's pointed write taken with M rescaled - `dict(s, M=M)` had
  sent 28 arrays through ascontiguousarray a solve -: 579 us a pass, 1.73,
  walking 520 and 1.92; the worker 535, 1.87. Its walk's smoke the same to
  the digit. MuJoCo's own timers over its walk: a step 88 us - the constraint
  solver 40, collision 15 over 74 geoms, the rest kinematics, inertia and
  make. Its drives told over the SIL buses 24 us a call, the C's five frames
  19 of it. Its suites run its flat out (`worker.session`, `sprung.laid`);
  real time is `worker.run`'s, the page's. The page's tier by the machine
  (`modes.TIERS`, said at boot; the BUSES box's row, its headroom from
  `apart.paced`'s time stepped against the wall's): on 16 cores its 27
  boards in-process; native would take 3.0 and fit once its body is wired to
  it (todo/host.md); Renode 113.
- Its touch's filter in C (`wbc_touch.c`, `tools.cores.touch`; its memory one
  array, `touch.laid`, where seven keys of `g` were): bit for bit at 4 001
  passes of its law over 12 s of its walk, 120 -> 9.5 us a call; its soles'
  arithmetic (`touch.soles`) the C's own, the python's beside it in the
  test. Its loop 579 -> 440 us a pass, 1.73 -> 2.27 of real time, walking
  2.28; the worker 487, 2.05. The robot's walk's smoke the same to the digit.
- Its twist's step in C (`wbc_twist.c`, `tools.cores.twist`: the floor under it,
  its heading, its rhythm flipped, its gait modulated, its look; its memory
  one array, its words with `g` another, `se2.twist_laid`, `words_laid`):
  bit for bit at 4 001 passes over 12 s of its walk once the floor's point
  took numpy's order for a fancy-indexed matrix, F-ordered, the (2, 2) @ (2,)
  fma; 118 -> 8.7 us a call. The robot's loop 440 -> 399 us a pass, 2.51 of real
  time, walking 2.50; the worker 455, 2.20; MuJoCo's step 85 of the 399.
- Its table's waits in C (`wbc_wait.c`, `tools.cores.wait`: the rows that halt
  its rhythm and the rows that raise it, held a pass; its table given at
  each call, 512 rows at most): bit for bit at 8 002 calls over 12 s of its
  walk, 60 -> 7.2 us a call. The robot's loop 399 -> 368 us a pass, 2.72 of real
  time, walking 2.69; the worker 418, 2.39; MuJoCo's step 84 of the 368.
- Its surety's brace on the solve in C (`wbc_brace.c`, `tools.cores.brace`:
  the four dofs' inertia scaled off the solve's M, the tuck's torques and
  the cervical blend over its answer, its drives' inertia, its legs'
  stiffness; its matmuls numpy's own BLAS on the gathered submatrices in
  numpy's memory order, `M[rows, :][:, rows]` F-ordered): bit for bit at
  4 001 passes over 12 s of its walk, 96 -> 14.6 us a blend. Its loop 368 ->
  329 us a pass, 3.04 of real time, walking 3.02; the worker 381, 2.62;
  MuJoCo's step 82 of the 329. The hand ports' tests a suite of their own,
  test_ported.
- The rows that wake its rhythm in the waits' C (`wbc_wait_woken`,
  `waits.woken`: the last numpy of its latent step; their python the
  reference in `machine.law.waits`, the waits split off `motion`): bit for bit
  at 4 001 calls over 12 s of its walk, 36 -> 6.0 us a call. Its loop 329 ->
  325 us a pass, 3.08 of real time, walking 3.02; the worker 382, 2.62;
  MuJoCo's step 84 of the 325.
- The copper's heat metered in the world's motor step in C (`wbc_pass_motor`,
  numpy's ddot), bit for bit over 12 001 steps; the drive frames' and the
  solve's pointers read off their arrays (`tools.cores.wbc.address`) where
  `.ctypes` built an object a call. Its loop 327 -> 311 us a pass, 5.0 %: two
  trees interleaved, each run's fastest 1 500 passes - another session's
  bursts doubled a whole run's own.
- The page worker's referee and averages in C (`wbc_pass_paired` on MuJoCo's
  own contact array, `wbc_pass_averaged`), their numpy's at every pass of a
  walk and a shove that throws it (test_gynoid): a fall's test 7.2 -> 2.6
  us, the averages 4.8 -> 1.4 us, 8 us of a worker pass's 380.
- CPython 3.14's experimental JIT (`PYTHON_JIT=1`, in this host's 3.14.7): its
  loop 307 -> 319 us a pass, 4 % slower over three runs each interleaved; it
  stays off.
- The lodge's still boxes laid once off the model's own kinematics
  (`room.props`, `_laid_props`: a box welded to the world, the door read each
  call): 269 -> 25 us a call, identical at each of 1 556 calls of the page's
  worker walking; its map refreshes about 120 times a second on the page.
- The robot's shield's QP built only when a row is short (`schema._evaded`): the
  same values where it runs, a short row on 2 of 13 334 law passes walking and
  19 in the page's worker; `shielded` 307 -> 244 us a call, ~21 us of a world
  pass. Held at every call of its walk, the worker, a chair, the doorway and
  the closing scene.
- The robot's map's refresh cut (`registry.seen`, `_boxes`, `boxes`): the match
  metric laid once a look, the box stacks at once; the page's worker 968 ->
  904 us a pass, 1.04 -> 1.12 of real time, two trees interleaved. Every call
  of the worker and its bare walk and 4 x 48 000 fuzzed calls the same.
- `bodied`'s glue taken in one generated call a part (its leaves, its pins,
  its body's constants; the leaves seen last call skipped in C) measured
  nothing on its loop, +0.7 % wall: the 65 us a pass a loaded profile gave
  `fits` is not there unloaded; not kept.
- The robot's invariant EKF compiled from its numpy (`inekf.points` on its body,
  `inekf.step` on its state, laid in g): its loop 773 -> 693 us a pass, 1.32
  -> 1.45 of real time, two trees interleaved. The filter's whole state the
  same at each of 26 336 law passes of the played walk, turns, moves and
  strafes, the chair and the bed, both parts held.
- The robot's capture point's barrier compiled, a ring's length a part
  (`barrier.ringed` through `bodied.on_shapes`, the hull on Python's floats):
  its loop 681 -> 648 us a pass, 1.48 -> 1.54 of real time. Its run the same
  at each law pass of the chair's sit and rise, the bed, the hop down and the
  played walk.
- The robot's DC link's step compiled from its numpy at every world step
  (`dclink.compiled`, `stepped`), its field's seek (`field.seek`, its table's
  scan `due` apart) and bemf's constants laid as arrays: its loop 701 -> 671
  us a pass, 1.44 -> 1.49 of real time, two trees interleaved. Its run the
  same at each law pass of the chair, the bed, the hop down and the walk; its
  walk's smoke the same to the digit.
- What bears it weighed and its forefeet compiled from their numpy
  (`observer.weighed`, `forefoot` on its sense's feet: the observer's own
  `mj_kinematics` a law pass gone), its points' masks and its edge's
  pointers laid once: its loop 610 -> 589 us a pass, 1.63 -> 1.70 of real
  time. Its run, the observer's forces and the invariant EKF's whole state
  the same at each of 20 335 law passes of the played walk and turns, the
  chair, the bed and the hop down.
- Its shield's clearances compiled from their numpy (`capsules.felt` on the
  shield's own data, `capsules.Own`: posed, every pair's nearest points and
  clearance Jacobian, shapes fixed; `shielded` takes its near rows of them),
  the tracer an einsum on a constant operand and its order read on draws
  where its own result fits more than one: its loop 594 -> 539 us a pass,
  1.70 -> 1.88 of real time. Its run and every pair's clearance the same at
  each law pass of the five scenes. The brace's python beside its C
  (`machine.law.brace`), motion at its cap.
- The robot's wayfinder's gap compiled from its numpy a room's size a part
  (`wayfinder.gap` on every box, those out of the band masked, the same as
  compacting them in 100 000 fuzzed rooms): its loop 527 -> 502 us a pass,
  1.90 -> 2.02 of real time - its walk's room empty, its gap 100 us of
  numpy on nothing. The tracer's reduction of none its start alone (a min
  from its initial had read past the empty source), one from an initial of
  its state or one numpy refuses Untraced. The robot's errands through the lodge
  (the door's gauge on, the chair, the bed), the five scenes and 14 001 gaps
  beside numpy the same.
- Its state told to the page at 60 Hz pickled 866 us, 820 of it the room's
  53 props: tuples of numpy scalars, each pickled through numpy's
  reconstructor. Their boxes in Python floats (`floor.props`, `room._box_of`,
  `.tolist()`, the same values): the pickle 54 us, 24 -> 16 kB, told 403 ->
  294 us; the page's worker with its state told and pickled 722 -> 658 us a
  pass, 1.39 -> 1.54 of real time. Its run in the five scenes and its errands
  through the lodge the same.
- MuJoCo's step 84 us walking, 11.9 of real time alone, 102 in the lodge;
  nothing in its options unused (no sensors, no energy; the Newton solve at
  1e-8 on elliptic cones its physics). From fc3996cb, two trees interleaved:
  its loop 634 -> 497 us a pass, 1.59 -> 2.02 of real time; the page's
  worker, its state told and pickled, 859 -> 646, 1.18 -> 1.56. A pass
  without its law 176 us: MuJoCo 79, its C 30, its Python round them 66. Its
  ask's surety weights compiled (two parts, the same bits) measured nothing:
  not kept. The step's ctypes edge 66 -> 53 us a solve (np.broadcast_to).
- The glue round its compiled parts 77 us a pass walking, their C itself 1.5-3
  us a call, `fits` 45 of it - a loop walking each leaf's path 1 us a leaf.
  Each build's checks and walk written out once (`tools.cores.glue`, beside
  its result's `maker`): the glue 66 us, its loop 500 -> 492 us a pass (-3.1 %
  cpu), 2.06 of real time; the five scenes and the errands the same, every
  part held. What is left of it fresh arguments each call (`modal_sensed`'s
  loads a list, its hips, arms and prop new arrays) and `sense`'s 40 views.
- The lodge's lamp switch 15 us a world step in python: the switch's geom
  against a hand's now paired in C on MuJoCo's contacts (`wbc_pass_paired`,
  the referee's), the same pair at each of the lodge's 17 424 geom pairs; the
  page's worker 657 -> 644 us a pass, its errands through the lodge the same.
  numpy's float64 tanh (SVML's, X86_V3 here) transcribed with fma, its 16
  intervals' table, matches np.tanh at 2.1 million values - the C runtime's
  parts at 20 %: a C twin of its gearboxes may use it.
- Its walk's pass across its history (the fastest 1 500 passes by process
  time, a worktree a commit): 312 us before the kernel's steps; its momentum
  observer in python +240 (in C +42), its capture point's barrier +62, its DC
  link +21, the right-invariant EKF +125, the body schema's 18 capsules +136,
  its shield on the room and the wayfinder +104, the noisy buses +21 - 896 at
  the steps' end, 531 with the DC link and the ischia compiled, 573-646 since:
  one commit's runs 10 % apart under the other sessions' load. In a pass (its
  parts wrapped after laying): its law 367 us of 670, its sensing 83, its
  shield 41 - a solve on its mass matrix, 12 of 13 pairs near walking -, the
  C step 37; the compiled parts' wrappers 60 us a law pass of their 113, timed
  bare - a timer at each stage had read 295. Cut, its trajectory's sha1 the
  same over its walk, its lie and a chair: its pelvis's pitch alone where its
  law took the whole of `modal._turns` for it (43.5 -> 12.1 us), `np.isin` of
  two constants (13.8 us) and its hips' indices laid once, its arms' modes an
  array, its drives' clip in place, its touch's Jacobians laid on its body,
  its IMU's sums one array (`inekf.IMU`): its pass 573 -> 510 us, -10.9 %,
  its walk's smoke the same.
