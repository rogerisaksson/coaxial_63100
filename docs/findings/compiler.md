# Findings: her law compiled

Her numpy compiled to C and held bit for bit (`tools.cores.trace`, `emit`,
`held`, `bodied`): numpy's own orders, her parts compiled, what they cost in
time and in memory. The board's findings: [FINDINGS](../FINDINGS.md).

- Her numpy compiled to C (`tools.cores.trace`, `emit`, `held`): traced on her
  arrays, each statement a loop nest at its layout, each einsum summed in the
  order numpy's own result shows (the regressor's 'nij,nj->ni' SSE's two
  lanes, its other 222 a plain sum), held bit for bit on the arrays traced and
  on fresh ones. One state's program at one state over any number: the
  regressor 0 of 5.35 M entries apart, its C 5.95 ms at -O3 for 400 states
  (8.0 at -O2, 8.5 at -O1, each level bit for bit), gcc 18 s and 0.79 GB.
  Built in the walking process its trace held her world 0.88 s: built in a
  process of its own, numpy's own meanwhile, kept by what its C is made of.
- Her sensing compiled on her body (`tools.cores.bodied`): `wbc.sense` traced
  on a twin of her body, her data's arrays and her scratch externs, each
  MuJoCo call mujoco.dll's by its prototype in mujoco.h, each matmul numpy's
  BLAS through `wbc_matmul`. 130 -> 32 us a call, 0 of 40 configurations
  apart; her walk 765 -> 713 us a pass, the two in one process pass by pass,
  her trajectory the same to the bit over 6000 passes. Traced and written
  10 ms at her law's first pass, built 0.9 s in a thread, held on the twin
  4.5 ms; a new library's first load 40 ms, loaded in the thread. Her step
  pointed at its block in one write, its 24 pointers from the block's base
  (`law.POINTED`): the step's call 142 -> 98 us, its Python 92 -> 47.
- numpy 2.5's reductions, read off its bits: a sum from +0.0 (`np.sum([-0.0])`
  is 0.0), a run innermost in memory and summed pairwise - eight running sums
  from 8 terms, halves past 128 -, an axis not innermost added in order; min
  and max from the first in order; a mean the sum over its count. Its maximum
  and minimum keep the second operand at a tie of zeros at every length (maxpd,
  minpd), Python's max and min the first, its clip x; `wbc_np.h`'s `npmax`
  and `npmin` Python's, `npmaximum` and `npminimum` numpy's. Compiled, 38
  statements of them bit for bit (`test_compiled`).
- Her law's functions compiled on her memory (`bodied.on_args`): each array
  given an extern in its own memory - `grip.felt`'s `seen` and
  `law.gripped`'s `out` written where Python writes them -, a dict's entry
  rebound written back, a number traced, the rest pinned; an einsum over two
  summed indices numpy's (`kjab,kjb->ka`: j outer, each run over b in SSE's
  two lanes, 500 of 500 draws). `grip.felt` 51 -> 16 us a call; her walk 797
  -> 696 us a pass with her sensing, felt, gripped and the step's pointers,
  the two in one process pass by pass, her trajectory the same to the bit.
- Her estimate, her grip's floor and her situation compiled beside them
  (`law.laid`): `np.array` of her scalars laid as numpy lays it, a product
  along an axis from one in turn; `estimate.step`'s and `grip.outside`'s
  Python max and float numpy's, the same bits. Her walk 780 -> 648 us a pass
  with the seven, p99 2.32 -> 1.76 ms, the two in one process pass by pass,
  her trajectory the same to the bit.
- Her ask's arithmetic its own function, compiled (`modal.poses`; `given` the
  ask of it by her stance). A compiled call 16 -> 6.75 us: an argument the
  same array as the last call's keeps its address, one converted written
  into a buffer of its own, an address read off numpy's array head. Her
  drives' step pointed only at the law's new arrays (the median pass 251 ->
  198 us); `borne.read` over the pushed segments alone. Her walk 792 -> 600
  us a pass, p99 2.26 -> 1.49 ms, the two in one process pass by pass, her
  trajectory the same to the bit.
- Her sensing flat compiled (`modal.sensed`): a dict argument lazy - an
  entry an extern as her code reads it, its presence pinned -, a float in it
  traced; one another part's last result addressed off that part's block in
  one write, 19 of her 24 sensed arrays (`sources`); an output that is a
  whole argument handed back as the caller's own array, as Python returns it.
  21 -> 10.6 us a call; `wbc.pointer` an address read off the array's head.
  Her walk 773 -> 576 us a pass, p99 2.25 -> 1.42 ms, the two in one process
  pass by pass, her trajectory the same to the bit.
- The regressor's two einsums of no element (`nhi,nic->nhc`, h of 0) took
  no order in C and shifted every later one by two; once an order carried
  its letters its build apart died on it, her wear's window ran numpy's
  regressor (parts 27 -> 54 ms) and `test_sysid` held numpy to numpy. Each
  takes its own now; numpy's orders read off random states - her rest pose
  twice read SSE's lanes as a plain sum -; a build apart keeps what it raised
  as its refusal; the test holds that its C was built. Her compiled parts
  traced and held 5 ms apart: her first pass 47.6 -> 11.1 ms, none past 20.
  A ctypes field's view in numpy's own dtype (`wbc.plain`): as_array's equal
  one made numpy copy the step's arrays at each `ascontiguousarray`, her
  drives re-pointed each pass (29 -> 23 us a pass).
- `check_contacts`' arithmetic its own function, compiled (`touch.soles`):
  each sole's pushes and their sizes as numpy's 1-d norms, its toes' lowest
  tip, the floor under it; `check_contacts` keeps her state, its branch and
  Python's hypot. Her walk 710 -> 491 us a pass with ten parts, p99 2.17 ->
  1.32 ms, the two in one process pass by pass, her trajectory the same to
  the bit.
- The step's struct held a numpy view of itself (`np.frombuffer(k)` on k): an
  ndarray is not the collector's, so struct, view and her sensing's compiled
  part with its twin and builds were never freed - 34 MB a session,
  `test_scenarios` out of memory at 522 s. The view lies in the step's views.
  A twin (16.4 MB, its MjData's arena 15.7) let go once its part is held: a
  session 84 -> 34-52 MB before the collector, numpy's own 34.
- A compiler's change leaves every kept build cold: under a relay of eight,
  each suite with her wear window began its own regressor build apart (1.5
  GB) - 17 in two minutes, five in one second - outside the relay's gate, and
  the laptop's commit ran out (`test_scenarios` and VS Code down). A build
  apart begins through `focus.admitted` (the ledger's room) and one a build
  machine-wide (its marker); gcc one a library machine-wide (`build._claimed`,
  the others waiting) and one at a time a process. The same run after: free
  commit 5.14 GB at its least.
