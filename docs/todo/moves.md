# The 32 base moves

A palette of 32 moves (`0x00`-`0x1F`), each a task frame, a selection, an
impedance, a feed-forward wrench and a guard, composed in sequence. Here it is
a catalogue of what she is to do, read against what she does: her one law
(`latent` -> `impedance.ask` -> `wrench.step`) realizes every move her texts
say (`defbehavior`, the suite's `defpatch`/`defmaneuver`, the kernel's
`defprimitive`/`defmodifier`); a guard is a phase's wait on her support
(`impedance.PROP_ON`) or a funnel's inlet window in manifold shares
(`define-postural-funnel-topology`), not a boolean with a fallback slot; a
move's numbers are her dimensions' or a text's as given. No second law, no
move table in C: the step runs on the host, the boards are drives (the WBC
device benchmarks the step on a model's stand).

| ID | Move | Hers | Left |
| --- | --- | --- | --- |
| 0x00 | idle, gravity alone | her rest; the bed's sleep `gate-inverter-high-z` (0 W) | a text with every mode's stiffness 0 floats her where she is held |
| 0x01 | freeze, damped | `latch-current-encoders`, `cascade-actuator-gates`; the STO chain is the stop | - |
| 0x02 | yield to a push | her shield (`schema`, the pairs' ECBF), the jamb's yield; `specialize-with-tactile-surprise-reflex` | the room's boxes in her shield (protocol 6) |
| 0x03 | hang along gravity | her arms held hanging (`acts._armed`, `W_STILL`); `specialize-with-deep-squat-suspension` | a load carried: `splice-carried-payload-envelope` on the loop |
| 0x04 | ballistic reach | `mode_arm_reach`, both arms a scalar; the texts' shapes (`ortho.SHAPES`) | a hand's task frame in her step (item 1) |
| 0x05 | guarded approach | `on-contact-threshold` at `PROP_ON`, the descent's soft landing | a hand's, on item 1 |
| 0x06 | tactile edge probe | `specialize-with-tactile-arm-sweep` (text); her skin's touches (`edge.TOUCH`, `touch`) | on the loop, on item 1 |
| 0x07 | soft land, settle | the handover's press (5.7 mm), `substrate-*`, the settle | a carried object's (`splice-manipulated-object-adjoint`), on item 1 |
| 0x08 | retract clear | `specialize-with-nullspace-limb-retraction` (the capsules' pairs, live) | a hand's retreat along its contact's normal, on item 1 |
| 0x09 | compliant slide | the Coulomb slide: `ask['slide']`, `(slide v)`, `somatic` (seats) | its driver (protocol 5); a hand's on item 1 |
| 0x0A | constrained arc | - (the lodge's door is ajar, Test 1; its quadric in her registry) | item 2 |
| 0x0B | prismatic glide | - | item 2 (no drawer in the lodge) |
| 0x0C | pivot, lever, pry | - | item 2 (the door has no handle) |
| 0x0D | rolling cylinder | - | item 2 |
| 0x0E | bimanual hold | `allocate-grasp-matrix-wrench` (8 N squeeze, 95 N crush), `splice-carried-payload-envelope` (texts) | on the loop: two hand frames and the internal squeeze, on item 1 |
| 0x0F-0x15 | spiral mate, wiggle insert, thread start, ratchet, snap, saw, strike | - (`couple-non-prehensile-shove` the nearest, a forearm's) | item 4 |
| 0x16 | power cage grip | her grippers (`figure`: a wrist and a gripper a side, 1 DoF) | item 3 |
| 0x17 | slip guard pinch | `engage-incipient-slip-reflex` (text: palmar shear piezos, 80-250 Hz) | item 4: no tactile sense on the board |
| 0x18 | scoop and cradle | `specialize-with-encumbered-payload-clearance`, `modulator-inertial-payload` (texts) | on the loop, on items 1 and 3 |
| 0x19 | gravity pivot regrasp | - | on item 3 |
| 0x1A | quiet stance | `stance_hold`, her knee 6 deg, the capture point's barrier (`barrier`, `enforce-dcm-barrier-zcbf`) | - |
| 0x1B | pendulum gait | her walk: the rows' rhythm, `xform-balance-biped-zmp`, `topology-pendular-walk`; 0.70 m/s, 408 J/m | - |
| 0x1C | capture step | `parry_shove`, xi = c + v / omega, `specialize-with-reactive-capture-step`, the stride hop's parry | - |
| 0x1D | brace and anchor | `borne.read` takes any pushed segment as a contact; `specialize-with-unilateral-wall-brace`, `-assistive-egress-bracing` (680 mm, 0.25), `recruit-brace-support` (texts) | a hand placed to brace, on item 1; Test 4's rise on a carpal heel |
| 0x1E | counterweight squat | `crouch`, `lie_floor`'s kneel; `specialize-with-deep-squat-suspension` (text) | a lift from the floor, on items 1 and 3 |
| 0x1F | saddle dock, charge | - (nearest: the bed's sleep at 0 W) | hardware out of these texts: 150 A through seat plates against a 63 V, 100 A board |

Of 32: 10 on her loop or as her phases' text, 12 as kernel text not yet on
the loop, 10 absent - the hands' and tools' and the saddle's.

1. **A hand's task frame in her step**: `wrench.step` has her upper body as
   posture rows alone; the kernel's grasp, shove, sweep and brace presuppose a
   hand's wrench row (its point, its frame, a cone where it presses, `borne`'s
   contact when it bears). DOD: a hand said to a point of the room gets
   there within her reach with her balance held and her walk as solved; the
   forearm's `couple-non-prehensile-shove` on it (Test 2's chair).
2. **The door's arc, a drawer's glide, a handle's lever**: on item 1, the
   hinge or axis from the thing's quadric in her registry, the arm's radial
   stiffness 0 about it, force along the tangent alone; a handle and a drawer
   in the lodge. DOD: the lodge's door swung from shut to 85 deg by her hand,
   0 N m to her ankles.
3. **A grip said**: her grippers as a mode of her text, closing to their
   drive's peak current (`bemf.PEAK`), holding; a thing to grip in the lodge.
   DOD: a handle lifted and set down (0x07) without slip told by its motion.
4. **Tools and slip**: 0x0F-0x15 and 0x17 want a tool frame at the hand and
   a force or tactile sense at the wrist the board has not (its AFE: phases,
   the DC link, an NTC; IMU and A1335 on the buses). Hardware first.
5. **On all fours** (asked; the kernel's `multimodal_*.lisp`): read whole;
   its clock, a morph's lags, the fifth limb, the truss and the equalizer in
   `machine.fours` ([support](../findings/support.md); test_fours); her loop
   runs none of it. Left: item 1 for a fore limb to swing; a limb's share
   from its clock (`MANIFOLDED` is a manifold's, `LET_GO` a leg's); the
   descent onto her hands (the floor scoot's); her centre 0.099 m back for
   the text's 0.68. DOD: laid on her toes and fists she holds 10 s, her arms
   0.32 of her or less; then crawls 1 m in the lateral sequence, never fewer
   than three bearing.
6. **On the floor** (asked; the kernel's `floor_*.lisp`,
   `ground_rolling_and_transitions.lisp`): read whole; her rest on a floor,
   the roll's ring and girdles and the trough's law in `machine.groundwork`
   ([support](../findings/support.md); test_groundwork); her loop runs none
   of it. Left: her shins, front and forearms as what bears her
   (`ortho.MANIFOLDS` has plantar, ischial, trochanter, dorsal, palmar); the
   origins on her body - the tall kneel at 0.60 of its axial column, the
   prone's arms in her sagittal plane, the tripod's fists level with her
   buttocks, not 63 mm under them; the roll's push-off heel planted; a way
   from her knees to her buttocks within her 25 deg hip turn. DOD: `compile-complete-groundwork-soa`
   played on her loop - knees, hands and knees, buttocks, back, rolled 180 deg
   onto her front and 360 deg on, up to standing - with no fall, each 1.5 s
   dwell held; the roll on the lodge bed too.
7. **On soft ground** (asked; the kernel's `deformable_*.lisp`): read whole;
   the work loop, its regimes, a patch's area, a power law's stiffness and a
   bed's phase and ring-down in `machine.substrate`
   ([support](../findings/support.md); test_substrate); her loop runs none
   of it. Left: item 6 first; a mattress, deep snow and a trampoline as
   simulated floors (her world has one rigid floor); a contact's surface
   speed for the work loop, its anchor sinking in her estimator; the
   identifier told plastic from damped by what stays sunk, not by energy;
   her fists past 14 kPa on all fours in snow. DOD:
   `compile-deformable-groundwork-suite-soa` played on her loop on the three
   floors, no fall, her estimator's height within 1 cm of her centre's
   through every sinking.
8. **Off her feet** (asked; the kernel's `terrestrial_*.lisp`, and
   `substrate_overlay.lisp`, the overlay their patches use that no text
   defined, bridged to the deformable baseline): read whole; a twist's
   coronal index and turnout, a sled's share and its anchors' hold in
   `machine.crawl` ([support](../findings/support.md); test_crawl); her loop
   runs none of it. Left: items 5 and 6 first; a sled as a sliding contact in
   her step (each of its contacts now stands or is free); a drag with over
   0.44-0.48 of her on its anchors; the supine bridge's origin on her hands
   and feet. DOD: the four segments of
   `compile-terrestrial-multimodal-suite-soa` played on her loop, 1 m each -
   the crab on her back and belly down, sideways; the scoot; her back up and
   down a 175 mm step - with no fall and her shoulders under their held
   torque on the step.
9. **At a seat's or a bed's edge** (asked; the kernel's `bedside_*.lisp`,
   fitted to her body): read whole; her skin's reach behind her, a rim's
   halt, a hung limb's swing, a roll's drift and the shrimp's escape in
   `machine.bedside` ([support](../findings/support.md); test_bedside); her
   loop runs none of it. Left: patches in her registry for her calves,
   shoulders and skull - the chairs' and the lodge bed's edges meet her
   calves, the escape plants her shoulders, the pillow takes her skull; her
   observer feels none of them; three strokes of her escape a quarter turn
   where the text plants once; a stool in the lodge. DOD:
   `compile-complete-stool-and-bed-lifecycle-soa` played on her loop in the
   lodge - the stool sat and left, the bed lain along from its side, rolled
   onto her side, front and back within its margins, risen to standing -
   with no fall, each dwell held.
10. **Onto and off a stool over her hips** (asked; the kernel's
    `overheight_*.lisp`, fitted to her body): read whole; her thighs' share
    crossing the edge, its capstan drag and a stool's centre of pressure in
    `machine.bedside` ([support](../findings/support.md); test_vault); her
    loop runs none of it. Left: item 9's stool; her hands bear on fists, so
    hung from them she leans back 40-76 deg - the texts' flat palms would
    hold her upright; her arms past what they hold for ever (shoulder 34.5,
    elbow 42.7 of 22.3 N m) within their peak. DOD:
    `compile-overheight-vault-and-dip-soa` played on her loop on a stool at
    her hips - vaulted on, 4 s seated, dipped off onto her forefeet - with
    no fall and the stool upright.
11. **Walk to run, a brake, a jump and a leap** (asked; the kernel's
    `ballistic_*.lisp`, fitted to her body): read whole; the regimes at the
    text's Froude numbers on her leg, a step's brake on a floor's friction and
    her hips' launch from her drives in `machine.ballistic`
    ([leap](../findings/leap.md); test_ballistic); her loop runs none of it.
    Left: the atlas's split-leap posture its header names; a launch with her
    hips and ankles beside her knees. DOD:
    `compile-ballistic-velocity-lifecycle-soa` played on her loop on a flat
    floor - walk, brisk walk, double-time, the leap, the run, the brake to a
    walk, the standing jump - with no fall.
