# Findings: body

Build, buses, falls, get-up, drawing and clothes; the drives in
[drives](drives.md), the board's findings in [FINDINGS](../FINDINGS.md).

## Model

- MuJoCo, 55 kg, 26 drives, 1 kHz (2026-09-25): on the line the legs stand
  in a V, 8 cm out at the hip for 3 at the foot, needing 0.773 m of 0.770:
  the IK clamped, a foot hung 3 mm off the floor, the body tipped over it;
  the pelvis 12 mm lower. On one leg no stiffness held (x5 4.8 s, x20
  unstable); the CoM fed back through the pelvis's target did.
- Hair on two hinges under the crown, 0.06 kg at 1.8 Hz: 17 deg fore and aft
  and 6 aside, walking (2026-09-28).
- Jeans' legs softly stopped at 14 deg swung to 19, the shin 35 mm out
  through their sides, the sneaker 64. Stiffly stopped at 5, 8, 3
  (`physics.HEM_*`), a 74 mm hem on the vamp: the shin inside, the sneaker 5,
  the other foot 12 in where 16 (2026-09-28).
- Jeans as MuJoCo cloth (`flexcomp`, 3.14.0), the 1 ms step 0.31 ms: two
  grids of 400 vertices over the shanks 19.2 ms, 100 vertices 1.06, 100
  touching the body alone 0.47, 48 so 0.31. A coarse reference runs live, the
  denim not (2026-10-02).
- The whole body collides (`figure.BODY`).

## Buses

- Boards on their buses, simulated (`machine.buses`, 2026-09-27):
  + A bus a limb (the type's subsystems: the axis, each arm, each leg), a
    process each with its boards' PD loops, in lockstep with the world a
    step at a time over a shared block (q, qd, limit in; ctrl out; a byte on
    stdin a step); the host's frames real bytes on a TCP socket a bus
    (`socket://`, an emulated limb's port).
  + Modbus RTU (`machine.rtu`): a pass is one 0x10 broadcast of the bus's
    setpoints (9 bytes and an i32 mdeg a unit, SETPOINT_REG) and a 0x03 poll
    a board (8 bytes; reply 13: angle and rate, i32 mdeg and mdeg/s,
    STATE_REG); a frame lands its bytes after its stamp at 9 216 000 baud
    (USART2/UART5's rate in the .ioc; 115 200 is the debug VCP's alone),
    8N1; a reply the board's 30 us turn after the poll; nothing sent on a bus
    still busy a pass on.
  + A leg's seven boards hear a pass's setpoints 40 us on, the host their
    state 0.4 ms on: held 89.8 %, stir 3.55 mm, as with threads (89.8 %,
    3.53).
  + Lockstep of five processes: 19 us a step spinning on the block, 37
    spinning then a semaphore, 46 the semaphore alone. The walk's step 1 431
    us wall against the threads' 1 710 (GIL hand-offs); the buses' share
    ~350 us a pass (a sendall 21 us a bus, a pipe byte 10 a process, CRC and
    parsing ~115); 0 bad frames in 10 000 replies.
  + The setpoint rate a board carries on at is read between two frames, never
    from a hold: from the reset's hold to the first frame 37 us later it came
    to 150 rad/s and every drive slammed to its peak.
  + A world with no buses runs the drives itself (`tools/sim/getup_lab.py`).
- Five processes (the axis's 5 boards, each arm's 4, each leg's 7), 0 bad
  frames, 2026-09-28:
  + With the heat in each reply (21 bytes) a leg's wire is busy 430 us of a
    1 ms pass. The loop runs 0.63-0.7 of real time here, mostly the
    director's Python, the lockstep 8 %.
  + Each board keeps its heat (`machine.heat`: thermal.c's envelope on three
    lumped nodes, the 5230SL through a 1:64 cycloid, heat at 10 times the
    clock). At 0.85 strides/s the hips' laminate spent 0.95 at 60 s and
    derated to 0.5; at 0.75 and 0.65 it held 0.76 and 0.74. The director
    eases the pace from 0.7 spent: 0.79 over 110 s, never derated.
  + A knee warmed to 100 C (0.93, derated 0.73) walked on, 0.98 2.5 s later;
    in its SOA (on-resistance x50) it dropped its gates under 107 N m, armed
    again 54 ms later, its derate back to 1 in 2.8 s, walking on. Held 88.1 %
    (the SOA and hot-knee scenes 100 %).
  + A halt's settle fell at 15, 15.3, 15.6 and 16 s, and at 15 and 16 s at
    every commit back to the director's first (898e224): as it takes over,
    the CoM stands 91 mm outside the line between the feet; also with the
    weight shifted before the lift and the rear foot kept on its ball.

## Falls

- Fall detection (`machine.director`, 2026-09-27): the pelvis tipped past 12
  deg and tipping faster than 60 deg/s, or below 0.65 m, is past the walker's
  recovery; then a curl into the squat's joints over 0.4 s, the arms toward
  the fall, the body lying as it lands. Contacts: seat, back, chest, skull,
  arms, thighs (`figure.CONTACTS`).
  + Curled only at 35 deg over 1.5 s: the legs walked on, the torso through
    the floor.
  + Into the hole, triggered at 13.7 deg and 300 deg/s, 0.16 s before down at
    40: the hands took the floor 0.12 s before the head touched once; on the
    left side and seat from 7.4 s (0.7 s after).
  + Off the rug: sat down backwards onto the feet (1.5 kN each), rolled onto
    the back.
  + Walking stumbles tip 6-8 deg, under the trigger.
- The lace fall's head (2026-09-28): the lace held the trailing foot; a dive
  onto the hands, the legs straight behind, the arms gave; chin down 40 deg,
  the head met the floor at 0.85 m/s. Waist turned 0, 20, 35 deg: 2.02,
  1.41, 1.33 m/s; arms straight or on the forearms: the head down sooner.
  Head held up, the neck back 20, 40 or 60 (`falls.HEAD_UP_DEG`): chest and
  hips take the floor, the head never.
- Reach pointed 45 deg off the tip (the hole); turned at the waist past 20
  off the front (`falls.turn`): 5. In 24 falls the head touched in 12, 688 N
  at most; unturned 10, 1170 (2026-10-01).
- Pads where falls land, 5 mm of gel (`figure.PADS`,
  `tools/sim/landings.py`): stiff, it bounced, median landing 19.6 kN, bare
  6.0 (2026-10-01).
- Down after a sill the spine bent back 98-101 deg; stopped (`physics.STOPS`)
  31-37 (2026-10-01).
- Down (`machine.down`, 2026-10-02): the fall braked, the drives cut once
  still, a bus's boards checked one after another, 0.2 s each. Lying: 33.2
  W, 25 boards at 1.33 W, 105 W at the check's first five. Before: the page
  drew 136-163 W lying, the tuck holding the curl, and 157 W given up, stuck
  falling with the drives armed.

## Get-up

- In the joints alone (`tools/sim/getup_lab.py`): sat up from the back, it
  rolled onto its side (2026-09-27).
- After a fall the walk began with the pelvis still lowered 12 cm, 8-10 cm
  crouched: `Walker.reset` on landing (2026-09-28).
- Falling, the legs' and trunk's drives shorted (`falls.SHORT_FALLING`). Up
  on a woman's 40/25 N m arms, sat, folded over the knees, after the rug and
  the lace in 25 s. Risen facing where it lay, it fell 0.2-1 s into the walk:
  MuJoCo's pyramid cone grips along the world's axes; elliptic at impratio 10
  it walks 13 m every way (2026-09-30).
- Folded over the knees, the trunk went 42-45 mm into them, 33-37 squatting;
  colliding, the fold failed 3 of 3 until the knees opened 30 deg: 7 mm. Up,
  the walk kept its catch and speed from before the fall and held 6 s 2
  times in 11; reset, 10 (`Walker.reset`). The head's IMU picks the side
  rolled over. Walking back, a joint came 2.38 m behind the lens
  (`gynoid.Follow`) (2026-09-30).
- The get-up a plan the observer checks (`machine.planner`): on the heels
  from 18 of 22 kneels (`observer.OVER_FEET_M`), onto the feet ended
  crouched (`observer.EARLY`), the floor's boxes deep (`floor.DEEP_M`): 13 of
  13 falls walk again 20-25 s after, 12 padded (2026-10-01).
- From the modules (45cc518) none of 5 falls walked again, 5 of 5 before; no
  test walked up from the floor. The thighs as modules, r 58 -> 72 mm at the
  top, met the torso at hip -126 (-140 before): the crouch asked -138.7 and
  drove 100 N m into it, the hands 0.17 m ahead of the feet. Re-searched
  (`getup_search.py --table f`), the crouch's hip -126.5: 5 of 5 walking at
  21.0-23.2 s, each first try (2026-10-03).
- Re-searched hot (`getup_search`, 2026-10-04) against two faults: spasms and
  a fall restarting after a shove; the arms out on the way up. Starts: the
  five falls after a minute's walk with the drives' heat carried; success:
  walking again 3 s without a fall; the look's cost 10 a second with the
  hands past 620 mm from the chest or the torso past 70 deg from the sit on.
  + As it was: 0 of 5 (3 tries, hands out 18.8 s, bowed 9.3).
  + The onto-feet keyframes' best of 96 (8 x 12): 5 of 5 at 22.8-23.4 s, 1
    try, 4.5 and 5.3 s; the arms thrown up to 151 deg for the lift's 0.5 s,
    as before (142).
  + Arms held low (shoulders 30, elbows straight): the best of 96 stood up
    none of 5, hot or cold (1.4 and 5.9 s): the throw lifts off the heels,
    not the hip. Kept with the throw; arms down needs another way up (TODO
    12).

## Page

- The drives' callouts (HUMANOID page) dock at the viewport's edges, a leader
  to each pivot: beside the figure at 0.42 m they were three cells and
  crossed the stride (2026-09-27).
- The page threw at a zoom of 1.1^3: a callout placed at its joint's height
  at rest, the joint past the drawing's edge, was written past the last row
  (2026-09-28).
- Callouts 34 cells a joint -> 20 in the tty's frame, the name on a patch in
  its drive's heat colour (`ansi.thermal_rgb`) (2026-09-28).
- An obstacle drawn in its own ink: laid under the lifting foot it showed 0.1
  s before contact; on the page it is laid a stride on at the same phase,
  1.3 m ahead of the pelvis (2026-09-28).
- Playback (2026-09-28): the frames drawn at 15 a second showed the same
  state again 65 times in 235, the rest 0.036 s of simulated time apart on
  the mean with 0.022 of spread; the process runs 0.7 of real time in slices
  of 0.05 s. Played 0.2 s behind the newest state on a clock of its own, its
  pace eased over 0.5 s, the states between blended: the same state again
  once in 234, simulated time per wall second 0.22 of spread where 0.62. The
  drives' six-field readings had cost the loop's `flat` twice its time; a
  float taken as it is: 0.53 -> 0.64 of real time.

## Mechanics

- The hip's drives kept on the pelvis's side, its pitch on its axis
  (2026-10-02): on the thigh 0.12 m down with rods to the pelvis they would
  add 0.038 kg m^2 to the leg's 1.685 about the hip, its ~140 deg past a
  crank's dead point (the ankle's rod spans 95, 39-41 deg at worst), and
  ball-joint play where hysteresis shows. The femur and tibia hang from their
  drives' gearbox collars (`mechanism.HUNG`), no longer through the drums'
  centres, 40 mm into the hip's; the pelvis's boom 30 mm round, 170 N m on it
  147 MPa.
- The ankle's rod bent down the calf's inner back (`linkage.BENDS`), its ball
  on the heel's back 20 mm in: clear of every part over fit's 45 poses, 2 mm
  of its drive at full dorsiflexion; straight inside the leg it stood 51 mm
  out of the shell and 15 of the jeans, now 52 and 5 behind the calf.
  Rejected: arms at 33 and 34.5 mm with the hub kept (the walk from the
  squat fell at 13 s); the drive 15 mm ahead (two falls put the head down at
  1.6-1.9 m/s); the crank outside (the thigh's boards met it sat back, 15
  mm) (2026-10-02).
- The ankle a parallel pair (`linkage.PAIRS`, 2026-10-02/03):
  + Two M drives at 1:68, 80 and 150 mm under the knee, each a 12 mm carbon
    rod crossed from a crank on its drum's gearbox end at the knee's front,
    bent down past the drums to a horn on the heel's back 12 mm out or 16 in.
    The ankle's shell, 32-36 mm round, leaves any rod's line within 25 mm of
    the axis: on 48 mm arms the balls stood 26-29 mm out.
  + Pitch both together: lever 0.75-0.91, 160 N m, 652 deg/s. Roll one
    against the other: 0.60, 105 N m. Each board's current the pitch's share
    plus or minus the roll's, within its 25 A (`buses`, `physics.paired`).
  + Inside the shell and jeans standing, fit clean over its 45 poses;
    scoreboard 495, held 68.3 % (the L and its rod: 535, 64.5).
  + Skeleton colliding: 542, 62.3 %, never the floor: inside the shell,
    outside the skins' capsules, the toes' belt sank to it with the sole and
    felled the body; a knee's drum landed past its gel, 935 N against 102 bare.
    Colliding, shoved past saving into the crouch, the head met the floor at
    1.03-1.66 m/s in 3 of 64 falls, without at 0.66-0.81 in 2; one with
    nothing of the skeleton touching: chance.
- Every drive's output on its drum's axis from its gearbox's end
  (`tools/sim/fit.py`'s outputs, 2026-10-03). Before: the ankle's L along
  the shin turning its crank about the knee's axis; the hip yaw's gearbox
  facing up, away from the hip; the elbow's and wrist's belts turning about x
  off drums along the arm (across it the drums stood 3-26 mm out of the
  shell, so a bevel pair at each gearbox's end, `linkage.BEVELS`); the spine
  roll's M 90 mm off its joint, nothing between.
- The hip a gimbal of parts (`skeleton.gimbal`, 2026-10-02/03):
  + The yaw's M on the pelvis turns a fork (a steerer; a crown over the
    pitch's L; legs before it 20 mm in, behind it 20 out) to the roll's
    bearings 55 mm either side of the hip's centre; the roll's M rides the
    fork, its spur pair into the cradle round the L.
  + Each drive's stator on the stage before its joint, a body of its own
    (`figure.STAGES`), 4.6 kg off the pelvis, 35.95 kg kept: the walk from
    the squat fell at 5.8 s, the scoreboard 578 against 495, the rises worse;
    their masses ride the pelvis until the walk is retuned
    (`physics.STAGED`).
  + The boom 60 mm across and an arm up to each yaw drive: across, it lay on
    the L, their inner corners 16 mm higher rolled 25 deg, no crown between.
    fit clean over its 45 poses once the femur left its collar 34 mm out,
    not 28.5 (knees under the body it met the fork's front bearing, 1 mm).
- The trunk's roll drive (2026-10-03): the shell at the spine's pivot 70 mm
  ahead and behind, 102 to the sides.
  + On its own axis a roll M stood 47 mm out of it, 10 past the clothes
    ahead, 27 behind.
  + A rod from the pelvis to the roll's stage crosses the pitch: its ball on
    the pitch axis, rolled 10 deg and pitched 60, it read 10 of false roll;
    its slant took the roll's lever through zero near 58 of pitch.
  + A bevel differential across the pivot fitted (two M at x +-55 mm), but
    its bevels carry the drives' 87.5 N m each, 4.4 kN on a 20 mm radius:
    steel needs ~70 mm across, the drums then past the shell; sat back on the
    heels, hips -163, the femurs met its drums 3-6 mm wherever the knee's
    board left room.
  + The roll's M on the pitch stage beside the pivot met the hip's yaw drum.
  + Taken: the roll first (`figure.SEGMENTS`; a replica, not an exact copy):
    its M on the pelvis, 10 mm lower, a 1:1 spur pair into it; the pitch's L
    on its stage, 13 mm off the M; the hips' yaw drums rolled 35 deg; fit
    clean, the M 4 mm inside the skin; scoreboard 582, held 64.0 %.
- Drives held (`fit.held_by`, `skeleton.HELD`, 2026-10-03). Before: the
  hip's roll M, the ankles' and the toes' held by nothing; the arms' and the
  trunk's only by wires through them. Now: the roll M on two collars and
  struts into the fork's back leg; the yaw M's collar on the boom; the
  ankles' M on struts into the tibia; the toes' S on the foot's keel; the
  humerus and the forearm clamped to their drums' collars and on under the
  bevel to a clevis; the tibia's clevis round the ankle's cross. A bone is
  checked against each drum it does not clamp: the femur went 30 mm through
  the hip's yaw M at the sit back's asked -163 and meets it past -142; the
  hip reaches -130 at most, the torso stopping the thigh. Asked -130: 5 falls
  of 5 walking at 20.4-23.1 s. The gimbal rolls -35..+28; the roll onto the
  front asked 30, now 27: 5 of 5 at 20.4-23.3 s. The trunk's still wires.
- The trunk framed (`skeleton.TRUNK`, 2026-10-03):
  + The boom ran through the roll's M: its middle 28 mm down under it, the M
    on it; 40 mm back, a hip's roll M yawed 18 deg met it, 17 mm.
  + The pelvis's fork to the roll's bearings 66 mm before and behind the
    pivot; its front legs 55 mm out past the 1:1 spur pair and in under the
    bearing (a bar across met the pitch's bracket at 75 deg rolled 35, 11
    mm), its back one on the middle.
  + The pitch's bracket arched 52-85 mm over the roll's band to the waist's
    M, now 120 mm up the torso, its gearbox down (the mass model had it at
    the pivot, inside the pitch's L).
  + Over spine -30..85, roll +-35, waist +-50 the closest 2 mm; every drive
    held (`fit.held_by`).
- Sneaker, size 38, in shape and physics (`coaxial/graphics/sneaker.py`,
  2026-10-04):
  + 237 mm long (the heel 55 behind the ankle, the ball 117 ahead, the toes
    65), 94 wide at the ball, 76 tall at the heel, 15 mm of toe spring; the
    gum sole 28 mm at the heel, 18 at the ball; 0.25 kg (`build.HOLDS`).
  + The heel at 58 with the heel's spheres fell the first stride from the
    squat. Its sole's box 90 wide: two 12 mm spheres at the heel's corners to
    roll the strike doubled it, 1634 N against 1108, and the parry never
    caught (797 and 49.8 % against 604 and 73.7).
  + The forefoot's bend is the toes' spring, 25 N m/rad (a sneaker's 0.2-0.5
    N m a degree); grip and give as before (`mjcf`).
  + A bare quarter render reads as a low-top sneaker, the toe box rounded,
    the sole band seen. On it with the toes sprung the walk from the squat
    ran 8 s with catches at 7.3-7.9 s, the walker unretuned (it fell at 7.0
    on the old shoe at 40 N m/rad).
- The shell fitted to the skeleton as sized, the clothes over it
  (`tools/sim/fit.py`, 2026-10-04). Standing, past the shell: the hips' yaw
  inverters +6 -> -1 mm (the pelvis's rings at 0.07 and 0.11: 130 and 106 ->
  140 and 114 wide), the head's turn +4 -> -1 (the neck 5 mm fuller), the
  knee +1 -> -1 (the femur's and tibia's ends 2 mm fuller); left: the toes'
  stack +20 in the shoe, the elbow +2. The shell past the clothes, measured
  since (`tools/sim/seams.py`): the cheeks 19 mm out of the jeans between the
  seat and the thighs' legs -> the seat's crotch 20 mm lower, its bottom
  rings 110 and 95 wide, 130 back: within 1 mm, the rest 3-27 inside. A held
  drive's bevel pair crashed the page's mechanism view (no drum on the
  wrist): skipped, in test_render.
- Tubes cut from stock (`tools/sim/members.py --family`, 2026-10-03): of 13
  stock sizes (1 mm walls to 25 mm round, 2 from 30) the lightest family
  passing every member within 2.5 mm of its radius was six (12x1, 20x1,
  25x1, 30x2, 40x2, 50x2), 0.67 kg of tube against 0.68 in nine sizes; seven
  as built: 35x2 for the tibia's lower run (40 met the ankle's rod by 6 mm in
  the crouch), every tube in the shanks' high-modulus grade. Changes: the
  femur's lower run 50 -> 40 (flex 0.25 deg), the boom kept 50 (0.30 at 40),
  the neck 18 -> 20 (0.28 -> 0.20), the hip fork's arms 22 -> 25 (1.02 of
  their allowable at the hips' 203 N m peak at KV 110 -> 0.78), the humerus
  24 -> 25, the forearm 22 and 18 -> 25 and 20. Marginal: the tibia's lower
  run 0.31, the femur's 0.25, the roll's horn 0.26.
- Members sized for stiffness (2026-10-03): at 0.6 of their drives' peaks
  at most 0.25 deg each; roll-wrapped carbon E 70 GPa, G 20.

| Member | Stock tube | Room | Taken |
| --- | --- | --- | --- |
| femur | 60 x 56 | 50 | high-modulus 50 x 44, 0.14 deg; 36 at its top |
| tibia | 60 x 56 | 40 | 40 at its top, 30 past the rods, 20 its clevis |
| boom | 50 x 47 | 60 | 50: a hip's pitch twisted the 30 mm one 1.3 deg |
| hip fork | 22 x 20 | 24 | legs 22, bearing arms 18, crown a carbon plate |
| spine fork legs | 35 x 32 | 24 | high-modulus 24 front, 30 behind |
| trunk column, girdle | 50, 35 | 40, 30 | high-modulus 40, 30 |
| humerus below its drive | 35 x 32 | 20 | high-modulus 20, 0.37 deg |
| forearm below its drive | 20 x 18 | 14 | high-modulus 16, 0.14 deg |
| drums' struts | - | - | printed 20 for the rods' 3-4 kN |

- The arms' 12 mm tubes bent 7.7 deg under the elbow's 50 N m. Printed
  PAHT-CF takes a solid 30-80 mm round where carbon takes a 20-60 mm tube:
  printed parts short and stout. The boom steep to the yaw drives: at 45 deg
  the hips' L met it, 8 mm. The fork's crown a tube met the femur's collar, 4
  mm. The neck's bracket 10 mm, the head's 0.77 kg on it; 25 stood 15 mm out
  of the neck.
- Printed parts without overhangs: collars split rings printed axis up, tube
  sockets bore up, side holes a 45 deg roof; brackets plates printed flat,
  their loads in the layers' plane (PAHT-CF and PET-CF half as strong across
  them); seats 0.1 mm under, reamed.
- Members under their drives (`tools/sim/members.py`, 2026-10-03): each tube,
  plate, rod and strut against its drive's deliverable peak or 1.5x its
  clamp, the less; stress against the material, flex at 0.6 of it against
  0.25 deg.
  + As they stood: the pitch's bracket (12 mm) 8.1x its carbon's allowable,
    22.7 deg (the spine wound up 27 deg at its 142 N m: the trunk's visible
    bob); the neck's bracket 1.3x and 3.2 deg; the hip fork's 10 mm steerer
    3.7x in the yaw's torsion, its crown's diagonals 7.1x; the shank's lower
    run 1.2 deg; the arms 0.75-1.2x.
  + Sized: the bracket 36 mm high-modulus (40 took 9 mm of the waist's
    holder); the fork's legs 30 high-modulus, its back member 40; the neck 18
    at 47 mm out (20 took 6 mm of its collar); the steerer 24; the fork's legs
    30, its arms 22; the femur's top 40; the tibia's lower run 36 (40 took 5
    mm of the ankle's rods); the arms 24 and 22. 3 of 63 marginal: the
    tibia's lower run 0.31 deg, the neck 0.28, the roll's horn 0.26.
  + Joint wind-up at the clamps, structure and box: spine 1.1 deg, waist 1.6
    (its 44 mm box), hip and knee 1.1, hip roll 1.8, ankle 2.6 (the 12 mm
    rods' stretch over the horn).
  + Carbon tubes cut to length, epoxied into printed PAHT-CF holders (a
    socket one diameter deep at 15 MPa shear takes 500 N m on a 40 mm tube);
    each holder an extrusion along its print axis, no overhang in any print
    pose.
- Flex in the world (`physics.WOUND`, `drives.flex`, 2026-10-03): a board's
  encoder on the motor sees its joint wound up by its last torque over its
  gearbox's stiffness (`drives.BOX_K`, estimated 20/9/3 kN m/rad) and its
  structure's (`drives.WIND`). At the members as sized the walk from the
  squat fell at 6.4 s; the structure's wind-up alone 6.3, the gearboxes'
  alone 6.6, the ankles rigid 5.6; at half, 16 s walked. A loop on the motor
  takes no series flex: the joint's own angle sensor (WOUND 0, the default)
  or the wind-up fed forward on the board.
- The trunk's roll on a four-bar (`linkage.PLANAR`, 2026-10-03): its 1:1 spur
  pair's 100 mm wheels stood in the pitch bracket's sweep, and a stage at
  1:40 needs 1.5 after it. An 18 mm crank on the roll's M, a 24 mm horn on
  its stage, crossed, in the 12 mm between the M's face and the roll's
  bearing, the M 7 mm back: over +-35 deg its lever 1.33 at rest, 1.48-1.73
  toward the ends, its transmission 53 deg at worst, 2.9-3.7 kN in its rod;
  over spine -30..85 and roll +-35 the closest 1 mm.
