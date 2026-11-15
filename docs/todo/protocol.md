# The somatic dual-domain kernel's protocol

Seven steps on the robot's loop and five tests in the lodge
(`tools/sim/house.py` runs the chair, the bed and the door): its loop
the real-time domain, a tuner apart the background one. The kernel's texts as
given: host/lisp/humanoid/kernel/, a module a file. The robot's figure, drives
and pack are bound from `machine.body.figure`, `machine.body.drives` and the
robot's build; the lodge's door, table, chair, stool, bed and loft stair (5
steps of 0.15 m along its right wall) unknown to the robot at t = 0; no fingers
grip. Each mechanism's numbers are its text's (`dsp.read(dsp.KERNEL)`); what is
done: [kernel](../findings/kernel.md).

1. **Step 2, the discrete phase margin on the robot's loop**:
   `machine.law.margin` reads its loops' return ratio offline
   (`tools/sim/margin.py`, kernel findings), nothing on the loop. Left:
   the monitor as the loop goes, its gait's own torque taken out of the
   probe's; the rollback of the law's K_d and the barrier's gamma it
   reads; the probe's amplitude through the gearboxes' play. DOD: the
   monitor on the loop reading what the tool reads, standing and walking,
   the walk as solved under it.
2. **Step 3's leak walking**: the chest's yaw asked walking leaks into the
   robot's centre of mass and ankles where its soles' bounds bind (kernel
   findings); standing it leaks under 1e-4 (test_kernel). DOD: under 1e-4
   of the chest's acceleration walking, as standing; the walk as solved.
3. **Step 4, reachability**: the sole's centre of pressure goes onto its
   ball over rho 0.86-0.98 (`machine.law.somatic`) and the legs hang let go
   on a seat over the popliteal height (`cascade.HANG`); the equinus
   window's ankle is not on the loop - its 0.58 rad plantarflex slew, the
   Achilles' series elastic. DOD: stepping down off a seat over its knees
   the robot's forefoot catches first; seated there its feet hang let go.
4. **Step 4, the Coulomb slide**: the regime is the step's (`somatic.CAPS`,
   `rounds.pushed`), but no slide moves the robot along a seat: seats over
   its popliteal height are mounted and left on its arms (`chair-mount`,
   `chair-dismount`), the text's slide onto and off them unplayed; the
   text's 65 Hz dither and its stop on the ischia's centring are not on
   the loop (seat, hitch findings). DOD: sat a human's 0.31-0.47 legs back
   and risen on both soles at 2e-4 rad either way; slid back onto a seat
   over its knees and forward to its edge before its rise; the walk, the
   bed and the errands as solved.
5. **Step 5, the body schema on the room**: `machine.law.schema` holds the
   robot's capsule pairs, not the room's boxes (kernel findings). Left:
   the text's null space whole (its balance bus for the trunk and
   shoulders, the limbs' synergies), the affordances' windows, the three
   slabs, the 4 payload slots spliced, in C. DOD: an arm swung at a table
   corner in its run stops short of it, 0 N m to its ankles, the jamb's
   yield kept.
6. **Step 6, the wayfinder's rest**: `machine.way.wayfinder` twists the robot's
   trunk through the gap 0.6 s ahead. Left: the robot's flow bent by the rows a
   shield holds against the room (item 5), a box held past 140 ms baked into its
   route's metric, a gap under its contorted width marked impassable, the crural
   foothold guard over swing 0.05-0.38. DOD: no oscillation before a doorway; a
   foot placed off a chair's crossbar.
7. **Step 6, the robot's map's rest**: its map in voxels in its estimate's
   frame (`machine.eyes.voxels`), its registry (`machine.eyes.registry`),
   its things' affordances (`machine.way.affordance`), its route and docks
   on them in both modes (sight findings). Left: the map placed by its
   estimate's drift (`inekf`); the switch told its wall's; the funnel chain
   to a thing's affordance (`solve-minimum-action-funnel-chain`); a mover's
   wake beyond the door's swell (`inject-gyroscopic-obstacle-curl`). DOD:
   the robot's sit and lie on its eyes' things, as on the stand-in's.
8. **Step 7, the lowering and the tuner**: `machine.texts.lowering` folds
   a gait into a bank of the text's ABI (wbc_soa.h), its biquads seeded,
   carried over its link's table; the loop reads none. Left: the loop's
   channels through a bank's biquads and fields, a bank taken at its
   barrier (`tap.due`) in its run of the room's schedule
   (`tools/sim/runtime.py`); the kernel's gaits refused on the loop by
   name till the insert's stages are on it (scenarios item 12). DOD: the
   room's schedule swapped in its run with no step in its torques; the
   tuner converged on it.
9. **The door charged walking**: the page's click on the door mid-walk
   runs the robot at the closed door and the swinging panel's edge wipes
   it (errands findings). DOD: the click mid-walk out past the line, no
   fall, 3 of 3.
10. **Test 1, door and sill, its rest**: the door ajar at 0.80 of the
    robot's shoulders passed (`tools/sim/doorway.py`, kernel findings).
    Left: a 40 mm bar in the doorway (the lodge's sill is 35 mm), a
    table's corner inside. DOD: its chest yawed ~60 deg and its elbows
    tucked 600 ms before the door, its swing over the bar by the sub band;
    past the table its trailing elbow clear of its corner, 0 N m to its
    ankles.
11. **Test 2, a chair shoved**: a chair across the robot's way to the stool, two
    legs on a rug, no fingers. DOD: the robot's forearms or carpal heels on it
    at 18 N or more (`ulnar-carpal-shove-bilateral` read against the robot's
    skin), its quadric in the robot's cone and its capsules, lifted 25
    mm where the rug holds it past 45 N, the robot's shins clear of its legs,
    parked out of the robot's corridor.
12. **Test 3, the stool and the pulled seat**: a stool 0.75-0.82 m (the
    lodge's is at the robot's popliteal height); sat on and risen from;
    pulled back mid-descent - pulled as the seat touches it, the robot
    falls (kernel findings). DOD: on, its forefoot, its thighs perched, its
    carpal heels braced, slid back till its ischia centre, its feet let go;
    off, slid forward till its forefoot catches before its ischia unload;
    pulled, the seat's weight 0, a capture step to xi = c + c'/omega_0, its
    gaze level, its torques inside their back-EMF bounds.
13. **Test 4, the bed**: from the stool to the bed, slid 180 mm deeper,
    rolled, high-Z, risen with a carpal heel on the table 650 mm off. DOD:
    the mattress's f0 within 180 ms of contact, its notch, no rebound; 0 A
    in the robot's phases from w_dorsal 0.96; its knee and hip peaks 20 %
    under its unbraced rise's.
14. **The side-lie** (side_lie.lisp, refused by name in `acts._pendulum`):
    the robot's latent has no roll of its base onto its trochanter with its
    legs off the floor. DOD: its back to the bed, perched on its edge, its
    trunk down onto its hip and its legs up along the bed, lying on its
    side, the bed bearing 0.45 of it or more, its neck under its walk's
    110 deg/s.
15. **Test 5, the loft stair up and down**: five steps each way, a forearm
    skimming the wall. DOD: up, flat-foot strikes on concentric work; down,
    the robot's stance knee eccentric, its pelvis dropped, its forefoot catching
    each step, shin and heel 32 mm or more clear of each nosing; V_bus
    under 60 V.
16. **The Hitchin bus** (`hitchin_isomonodromic_bus.lisp`, ops in
    `kernel.MECHANISMS`): none of its three finite pieces is on the loop -
    the Schlesinger transport (`schlesinger`) moving sliding contacts'
    wrenches without inner fighting (the stool's thigh scoot, the somatic
    shove), the Gaudin invariants on the Hitchin base (`hitchin-base`) as
    the autotuner's coordinates, the twistor rotation (`twistor`) in place
    of SVD damping where a closed chain nears rank loss; the text's
    infinite-dimensional rest never built. DOD: the three in the robot's
    loop and the C law, held bit for bit, the stool's scoot and the loft
    stair's brace measured on them.
17. **The tour's errands on fixed seconds** (`machine.way.tour`): its chair
    and bed steps say the rise 8 and 9 s after the errand while the robot
    still walks to the seat. DOD: the tour's chair and bed steps rise it
    from the seat it reached, 3 of 3.
18. **An errand said lying on the floor**: the tour's door after its
    mattress step stands the robot from the floor and it falls (introspect
    findings); a rise_floor is refused by its legs (scenarios item 14), an
    errand's walk is not. DOD: said lying, an errand refused by name, or
    walked after a rise that stands.
