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
   her capture point's barrier. Done (seat findings): her seat one patch of
   her step, its contacts' split pinned as felt; the regime as the step
   feels it (`rounds.pushed`, past SLIDE_M_S), the text's `(slide v)` its cap
   (0.55 deeper, 0.70 forward, `somatic.CAPS`), what bears her capped as
   told and no lower than the lever's rule; the rise's lean panned to her
   soles at the handover's 0.82 with the egress slide said, her legs'
   springs back with it, her neck braced seated, the slide's governor
   capping a sliding contact alone: the chair laid 2e-4 rad either way sat
   and risen (roll 12-18 deg, one sole through the lean), the dock and the
   chair errand stood. Left: her chair's sit lands on its front edge; deep
   (`seated.landed`'s buttock cap and thigh clearance let go, 0.29 legs,
   upright as a human) it is a perch (seat findings): the seat 5 cm over her
   knees, her ischia hover 2 mm and her thighs bear on the edge 0.10 m from
   her hips, and at -2e-4 one thigh bears, her soles unload and she rolls
   off her pelvis's one point, the step standing on soles in the air. Her
   fists reach no seat under 0.56 m wide; asked deeper from rest she moves
   17 mm (a sole held whole, the regime needing slip, the seat's grip the
   floor's; her soles let go she falls at every wobble, the patch's split
   running away; drawn along the said slide 1.4 cm). The deep sit is a
   tip onto a seat she knows is there: her thighs and ischia meeting it
   together with her shanks upright (0.40 legs at a dock 0.10 m further
   out, her centre of mass 9 cm behind her heels before it bears) - her
   step keeps her centre of mass over her soles whatever the ask (asked the
   posture she squats in front of the chair): a standing sole is held flat
   and whole, so a tip is a regime of her law to have - a sole rolled onto
   its heel's edge a contact of her patch set, its line held and its pitch
   free, its stance let go, the leg on its posture (item 4's equinus the
   same on its ball), with the kernel's capture step for the seat pulled
   away (item 13). Built up instead (branch `deep-sit-build`, seat
   findings): the deep landing, her pelvis as two ischia, a bearing point's
   room its capsule's radius, her soles let go where a rest asks nothing of
   them: on seven wobbles she sits deep 5 of 7, upright and rolled 0-1 deg,
   and rises 3 of 7 - the lean lurches her off the deep seat, the kernel's
   egress slide to bring her hips to the edge first; the sits lost at -1
   and -2e-4 a sole lifting 16 cm as the seat strikes before the rest lets
   it go; the handover wanting her hips asked up
   off the seat as the pan goes rather than the seat's force capped under
   what it gives (the pass taken apart, the points' rows 13 900), the ingress
   deeper after the crossfade needing her feet along (item 4's equinus or
   their own slide: asked 0.18 m deeper, planted, she moved 2 mm), the 65
   Hz dither, the ischial centring's stop on that CoP. On branch `sit-deep`
   (seat findings, the hitch on her hands): the two ischia, the rooms and the
   let-go leg on main's hands, the edge sit kept 3 of 3; the hitch as text
   moves her hips 0 - her let-go legs bear through feet on the floor, and the
   seat, told 0.3, is capped not held and unloads under her hands; the pan as
   a row of the step (weight 30) flung her, not kept. A low chair at her
   popliteal height (`chair_low`, hitch findings): landed 0.30 legs back, the
   deep sit 0.10 m further on planted feet; a scoot by the descent's lean and
   the kernel's slide unloaded the seat and stood her - the lean as far as
   the seat's cap, her legs' length following her hips, next. DOD:
   she sits a
   human's 0.31-0.47 legs back and rises on both soles at 2e-4 rad either
   way; she slides onto and off a seat over her knees, forward to its edge
   before her rise, and 180 mm deeper onto the bed; her walk, the bed and
   the errands as solved.
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
8. **Step 6, her map's rest**: her map in voxels in her estimate's frame
    (`machine.voxels`), its surfaces and structures boxes with their turn and
    an id look to look, her registry on them (`machine.registry`), what each
    affords her the text's (`machine.affordance`), her route and her errands
    on her things in both modes (sight findings, `tools/sim/eyes.py`): posed,
    every prop in reach told, 0.09 m off, the chair a seat with a back, the
    table a seat, the bed a bed. Left: her estimate rises 5-8 mm/s stepping
    in place (`inekf`, `estimate`) and the map smears a voxel in 12-20 s - a
    turn in place 185 things where 87 posed, her sit on her eyes' seat never
    staged: her chair's things grown meet the table's, `route.staged` returns
    its most inside them and she walks between free corners;
    a bed's dock the room's +x side (`route.docks`), a backless seat
    approached from a free face;
    the switch (white, no hue) its wall's; the three SDF bands at her knee,
    navel and crown, the kernel's funnels on her things, the mover's wake to
    the wayfinder. DOD: her sit and lie on her eyes' things, as on the
    stand-in's.
9. **Step 7, the lowering and the tuner**: `machine.lowering` folds a
    gait into a bank of the text's ABI (wbc_soa.h), its biquads seeded, a
    bank carried over her link's table. Left: her loop's channels through a
    bank's biquads and its fields, a bank taken at her barrier (`tap.due`)
    in her run of the room's schedule (`tools/sim/runtime.py`); its gaits
    are refused on her loop by name till item 1 - their insert's slew 850
    N m/s and jerk 12 000 rad/s^3 where hers says her drives' current loop.
    DOD: the room's schedule swapped in her run with no step in her
    torques; the tuner converged on it.
10. **The door charged walking**: the page's click mid-walk runs her at the
    closed door at 0.69 m/s; the panel sweeps 0 to 1.92 rad in 0.8 s and its
    edge wipes her - hinge left, the edge crosses her lane at omega r - out
    at 1.12 m/s askew, felled braking at the deck's rail (errands findings).
    From a stand the same errand walks out clean, 12.5 s. The push is the
    somatic shove's (`splice-somatic-object-interaction`, item 16's
    Schlesinger transport on the sliding patch), or her lane the hinge's
    side where omega r is small - the hinge the fixed point of the panel's
    move between looks. DOD: the click mid-walk out past the line, no fall,
    3 of 3.
11. **Test 1, door and sill, its rest**: the door ajar at 0.80 of her
    shoulders passed (`tools/sim/doorway.py`: her waist 28 deg at it, her
    arms 55 mm clear). Left: a 40 mm bar in the doorway, a table's corner
    inside. DOD: her chest yawed ~60 deg and her elbows tucked 600 ms before
    the door, her swing over the bar by the sub band; past the table her
    trailing elbow clear of its corner, 0 N m to her ankles.
12. **Test 2, a chair shoved**: a chair across her way to the stool, two legs
    on a rug, no fingers. DOD: her forearms or carpal heels on it at 18 N
    or more (`ulnar-carpal-shove-bilateral` read against her skin), its
    quadric in her cone and her capsules, lifted 25 mm where the rug holds it
    past 45 N, her shins clear of its legs, parked out of her corridor.
13. **Test 3, the stool and the pulled seat**: a stool 0.75-0.82 m; sat on
    and risen from; pulled back mid-descent. Pulled as the seat touches her
    (2.0 s of the chair scene) her sit's rows straighten her trunk past her
    heels and she falls. DOD: on, her forefoot, her thighs perched, her
    carpal heels braced, slid back till her ischia centre, her feet let go;
    off, slid forward till her forefoot catches before her ischia unload;
    pulled, the seat's weight 0, a capture step to xi = c + c'/omega_0, her
    gaze level, her torques inside their back-EMF bounds.
14. **Test 4, the bed**: from the stool to the bed, slid 180 mm deeper,
    rolled, high-Z, risen with a carpal heel on the table 650 mm off. DOD:
    the mattress's f0 within 180 ms of contact, its notch, no rebound; 0 A in
    her phases from w_dorsal 0.96; her knee and hip peaks 20 % under her
    unbraced rise's. Her side-lie (side_lie.lisp, `maneuver._pendulum`, refused
    by name): her latent has no roll of her base onto her trochanter with her
    legs off the floor. DOD: her back to the bed, perched on its edge, her
    trunk down onto her hip and her legs up along the bed, lying on her side,
    the bed bearing 0.45 of her or more, her neck under her walk's 110 deg/s.
15. **Test 5, the loft stair up and down**: five steps each way, a forearm
    skimming the wall. DOD: up, flat-foot strikes on concentric work; down,
    her stance knee eccentric, her pelvis dropped, her forefoot catching
    each step, shin and heel 32 mm or more clear of each nosing; V_bus
    under 60 V.
16. **The Hitchin bus** (`ortho_kernel/hitchin_isomonodromic_bus.lisp`,
    ops in `kernel.MECHANISMS`): the genus-0 parabolic Higgs field on her
    contact manifolds, poles her lever arms and PLL phases, residues her
    wrenches, the sum her centroidal wrench by the residue theorem. In the
    1 kHz SoA loop, in order of yield: the Schlesinger transport
    (`schlesinger`: d mu_i/dt = sum [mu_i, mu_j] d log(z_i - z_j), gain
    0.98, eps 0.025) moving sliding contacts' wrenches without inner
    fighting - the thigh scoot on the 0.75-0.82 m stool (item 13), the
    somatic shove -; the Gaudin Hamiltonians on the Hitchin base
    (`hitchin-base`, 8 nodes) as the autotuner's few orthogonal spectral
    invariants in place of its coupled parameter swamp; the twistor
    rotation (`twistor`: onset sigma 0.055, at most 1.0472 rad from I
    toward J) in place of SVD damping where a closed chain (feet and hands
    braced) nears rank loss, symplectic, no energy lost. Never built:
    infinite-dimensional gauge transforms, bundles of genus 2 or more,
    PDE solvers for the self-dual equations - the text's laws are the
    finite pieces above. DOD: the three in her loop and the C law, held
    bit for bit, the stool's scoot and the loft stair's brace measured on
    them.
17. **The page's chair errand falls a run in two** (introspect findings):
    said 'sit in the chair' at the page's start, she turned mid-walk at
    5.9 s, braked and fell at 6.1 in one run and sat at 16.97 in the next -
    her right hip 905 deg/s walking, 0.94 of its drive's rating. DOD: the
    page's chair prompt seated 3 of 3, the walk's hip within its rating.
18. **The tour's errands on fixed seconds** (introspect findings): the
    page's tour says rise_chair 8 s after sit_chair and rise_bed 9 s after
    lie_bed while their errands still walk - refused 0.5 m and 1.0 m off.
    DOD: the tour's chair and bed steps rise her from the seat she reached,
    3 of 3.
19. **An errand said lying on the floor**: the tour's door after its
    mattress step - she sat up, stood and fell at 47.78 s (gated); a
    rise_floor is refused by her legs (scenarios 18), an errand's walk is
    not. DOD: said lying, an errand refused by name, or walked after a rise
    that stands.
