# The somatic dual-domain kernel's protocol

Seven steps on her loop and five tests in the lodge (`tools/sim/house.py`):
her loop the real-time domain, a tuner apart the background one. Its texts
as given: host/machine/ortho_kernel/, a module a file (the two master
texts' 26). Her figure, drives and pack are bound from `machine.figure`,
`machine.drives` and her build; the lodge's door, table, chair, stool, bed
and loft stair (5 steps of 0.15 m along its right wall) unknown to her at
t = 0; no fingers grip. Each mechanism's numbers are its text's
(`dsp.read(dsp.KERNEL)`); what is done: [kernel](../findings/kernel.md).

1. **Step 2, the insert's inner stages**: its Butterworth D-channel at 42 Hz
   and its backlash bridge, each alone felled her turning walk (scenarios
   item 14). The D-channel's lag against her measured margins (kernel
   findings): 45.6 deg at 22 Hz takes her hip's walking 79 to 33, its 66 at
   15 Hz to 36, under the text's 38; standing with her gearboxes in, 24.7 to
   under 0. 46 Hz keeps 38 walking; standing needs the rollback (item 2) or
   the text's zero-phase compensation. DOD: both on her loop at their text's
   numbers, every suite as green.
2. **Step 2, the discrete phase margin on her loop**: `machine.margin`
   measures her loops' return ratio (`tools/sim/margin.py`); her gearboxes
   in, her hip's crosses 0 dB at 22 Hz with 25 deg standing (rollback 0.65);
   walking (`--walk`, the probe's own response, the same passes with it off
   taken out) no loop is past -3 dB under 38 deg - the DOD's measure met by
   the tool. Left: the monitor on her loop as she goes, where her gait's
   own torque swamps the probe (with it off the lock-in reads -1) and no
   twin session takes it out; its rollback on her law's K_d and her barrier's
   gamma for the standing play case; the probe's amplitude through her
   gearboxes' play. DOD: the monitor on her loop reading what the tool
   reads, standing and walking, and her walk as solved under it.
3. **Step 3's leak walking**: her chest's yaw asked walking moves her
   centre of mass 0.12-0.15 m/s^2 and her ankles 3.4-4.6 N m where her
   soles' bounds bind. DOD: under 1e-4 of the chest's acceleration walking,
   as standing; her walk as solved.
4. **Step 4, reachability**: rho = |hip - sole| / her leg's 845 mm each
   pass (`machine.somatic`); from 0.86 to 0.98 her sole's centre of pressure
   onto its ball by the furniture's told share (standing 0.998, nothing
   borne; sat on the chair 0.70-0.75). Left: her ankle's 0.58 rad
   plantarflex slew and the achilles' series elastic, past 0.99 her soles'
   weight to 0 and her legs let go into damped pendulums (a stool in the
   lodge, Test 3). DOD: on a seat over her knees her feet hang let go;
   stepping down her forefoot catches.
5. **Step 4, the Coulomb slide**: a thigh's and the seat's patch sliding at
   mu_k F_z, F_z under 0.55 of her weight, the text's 65 Hz dither, inside
   her capture point's barrier. Her chair's sit lands on its front edge for
   want of it: `seated.landed`'s buttock cap and thigh clearance let go she
   sits 0.29 legs back and upright, and falls rising (seat findings, her
   sit's dock). The regime is an ask of her step (`slide`, `fz_cap`: the
   contact's tangential rows freed, its friction kinetic and given, its load
   capped; seat findings, the Coulomb slide): without a driver it slides
   her off the edge, her centre of mass driven back alone 0.043 m deeper in
   2 s. Her text says it (`(slide v)`, `ortho.SLIDE` -> `latent.SLID` -> the
   step's ask); said with her hips asked 0.18 m deeper her feet planted and
   her knee held moved her 2 mm. Left, first: a bearing patch as one
   wrench of her step - its contacts (`machine.borne`, a point a segment)
   grouped by the text's manifold registry (`observer.POINTS`), one force,
   its centre of pressure within the points' spread as a sole's within its
   room - where two thigh points held still apart chatter and fell the chair
   laid 2e-4 rad off (seat findings, her seat as two points); then the
   slide's driver - the deeper posture with her soles let onto the forefoot
   (item 4's equinus) or slid along -, in the sit's and the rise's phases
   (`acts`), the 65 Hz dither, the ischial centring's stop on that CoP. DOD:
   the chair laid 2e-4 rad either way sat and risen; she slides onto and
   off a seat over her knees, forward to its edge before her rise, and 180
   mm deeper onto the bed.
6. **Step 5, the body schema on the room**: `machine.schema` holds her
   pairs; her arms held off the room's boxes by their elbows alone spared no
   strike at a jamb and halved her shoulder's yield (kernel findings) -
   undone. Left: the text's null space whole (her balance bus for her trunk
   and shoulders, the limbs' synergies she has), the affordances' windows,
   the three slabs, the 4 payload slots spliced, in C. DOD: an arm swung at
   a table corner in her run stops short of it, 0 N m to her ankles, the
   jamb's yield kept.
7. **Step 6, the wayfinder's rest**: `machine.wayfinder` twists her trunk
   through the gap 0.6 s ahead. Left: her flow bent by the rows a shield
   holds against the room (item 6), a box held past 140 ms baked into her
   route's metric, a gap under her contorted width marked impassable, the
   crural foothold guard over swing 0.05-0.38. DOD: no oscillation before a
   doorway; a foot placed off a chair's crossbar.
8. **Step 6, her map's rest**: her right-invariant EKF (`machine.inekf`)
    and her registry (`machine.registry`: 45 of the lodge's 53 boxes after
    her bed errand, blind spots kept 45 s); her eyes' things are its looks
    under `physics.SIGHT` (`sight.told`, `sprung.LOOKS`): 65 things told made
    96 slots where the stand-in's boxes made 33 - her map's own lumps told
    by colour, 13 'chair' up to 2.15 m from it, not the gate (sight
    findings). Left: her map's lumps told as the room's things
    (`sight.classify`, `Map.things`: one lump a thing), the three SDF bands
    at her knee, navel and crown, the kernel's affordances and funnels on it,
    the mover's wake to the wayfinder. DOD: her eyes' map as the stand-in's;
    the funnels' sit and lie chosen from it.
9. **Step 7, the lowering and the tuner**: `machine.lowering` folds a
    gait into a bank of the text's ABI (wbc_soa.h), its biquads seeded, a
    bank carried over her link's table. Left: her loop's channels through a
    bank's biquads and its fields, a bank taken at her barrier (`tap.due`)
    in her run of the room's schedule (`tools/sim/runtime.py`); its gaits
    are refused on her loop by name till item 1 - their insert's slew 850
    N m/s and jerk 12 000 rad/s^3 where hers says her drives' current loop.
    DOD: the room's schedule swapped in her run with no step in her
    torques; the tuner converged on it.
10. **Test 1, door and sill, its rest**: the door ajar at 0.80 of her
    shoulders passed (`tools/sim/doorway.py`: her waist 28 deg at it, her
    arms 55 mm clear). Left: a 40 mm bar in the doorway, a table's corner
    inside. DOD: her chest yawed ~60 deg and her elbows tucked 600 ms before
    the door, her swing over the bar by the sub band; past the table her
    trailing elbow clear of its corner, 0 N m to her ankles.
11. **Test 2, a chair shoved**: a chair across her way to the stool, two legs
    on a rug, no fingers. DOD: her forearms or carpal heels on it at 18 N
    or more (`ulnar-carpal-shove-bilateral` read against her skin), its
    quadric in her cone and her capsules, lifted 25 mm where the rug holds it
    past 45 N, her shins clear of its legs, parked out of her corridor.
12. **Test 3, the stool and the pulled seat**: a stool 0.75-0.82 m; sat on
    and risen from; pulled back mid-descent. Pulled as the seat touches her
    (2.0 s of the chair scene) her sit's rows straighten her trunk past her
    heels and she falls. DOD: on, her forefoot, her thighs perched, her
    carpal heels braced, slid back till her ischia centre, her feet let go;
    off, slid forward till her forefoot catches before her ischia unload;
    pulled, the seat's weight 0, a capture step to xi = c + c'/omega_0, her
    gaze level, her torques inside their back-EMF bounds.
13. **Test 4, the bed**: from the stool to the bed, slid 180 mm deeper,
    rolled, high-Z, risen with a carpal heel on the table 650 mm off. DOD:
    the mattress's f0 within 180 ms of contact, its notch, no rebound; 0 A in
    her phases from w_dorsal 0.96; her knee and hip peaks 20 % under her
    unbraced rise's. Her side-lie (side_lie.lisp, `maneuver._pendulum`, refused
    by name): her latent has no roll of her base onto her trochanter with her
    legs off the floor. DOD: her back to the bed, perched on its edge, her
    trunk down onto her hip and her legs up along the bed, lying on her side,
    the bed bearing 0.45 of her or more, her neck under her walk's 110 deg/s.
14. **Test 5, the loft stair up and down**: five steps each way, a forearm
    skimming the wall. DOD: up, flat-foot strikes on concentric work; down,
    her stance knee eccentric, her pelvis dropped, her forefoot catching
    each step, shin and heel 32 mm or more clear of each nosing; V_bus
    under 60 V.
