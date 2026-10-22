# Scenarios, by the general case

Each a test in MuJoCo: placed, commanded as a player would, judged by no
fall, its goal reached and its jerk (`tools/sim/scenarios.py`; those that hold
in `test_scenarios`, 20 of 31 at every wobble now:
[scenarios](../findings/scenarios.md)). In order: each a mechanism, the scenes it
serves its DOD; none an entry a case.

1. **Her support, whatever bears her** (`machine.borne`): a segment the skin
   feels pushed up is in her support region, grounds her, and is a contact of
   her whole-body step. A prop counts, and spans her support, by its share
   of her weight up to 0.15 of it, whole from there (by the share alone a
   brush of the bed's frame at the dock stepped her over its edge); her soles
   lifted by the seat's excess stay so, a human's seated feet down open
   (stunts.md). Met: the chair by its texts at 0 and 2e-4 rad, the seat
   bearing more of her than her soles for 2.6-5.1 s (at -2e-4 her perch
   slides off and she falls at 9.06 s, so at the base before the takes). The
   bed by its
   text is out of the held scenes: a 27 deg recline, the bed 0.39 of her to
   her soles' 0.61 - the step splits her weight evenly over three contacts
   where the legs should give it - and the page's lie wants 0.45. Open: the
   sit by the take collapses from its landing (stunts.md: her centre of mass
   released past the soles, the landing's sole swung up by the clearance, the
   knees folded to the told axial, the pelvis's pitch task given to the
   arms); a lie needs a trunk that does not roll (reclined 57 deg she rolled
   90 at 2e-4 rad) and a shuffle onto the bed (the human's hips go back 1.07
   legs); no rise from the bed stands her (bed_rise); the chair's sit by the
   take perches on its edge (hips 0.16 legs back where 0.48); the floor
   mattress, told to lie she crouches, her pelvis
   at 0.763 m after 12 s (4), and lain on the bare floor the referee calls a
   fall;
   `wbc_wrench.c` without contacts. DOD: chair, soft bed, floor mattress and
   floor, down and up, no fall called; reclined and risen from the errand's
   dock (aligned, her heels 0.05 m from the edge, her pelvis 0.14 m onto the
   mattress, the recline somersaults her: her soles unloaded, the bed's share
   0.74 to 0.31, pitch -84 deg; her height estimate 0.15 m up after 17 s of
   turning and backing), then the bed lengthwise, her soles on the mattress
   (test_errands).
2. **Weight handed over, never switched**: a sole's share of her support
   continuous in her height's reading, her support region and her step's
   bounds, where a flag now. `tools/sim/jerk.py` against her steady walk
   (head 597, hands 1398-1840 m/s^3): a start 1.4, strafes 2.9, turns 3.8, a
   spin 4.4, the chair 11.6, a hop down 11.1, the weave 17.9. Left: a sole let
   go at 60 N on its toes as the other takes 715 N (turns, the weave); the
   weave's hurried swing asking a hip 10.7 times its clamp, her balance's
   ask shrunk with it; a seat's touch flickering as she rises; a hop's
   landing, its shrink the neck's; a row's blend stepping her asks; her head
   and arms flung for her turns, seated and at a takeoff, where a seat's
   points give no moment (jerk.md); the switch at 0.15 of her weight gone
   (stunts.md): seated on the bed by the take her hands' jerk 17 000 to
   1 800; rising from the chair by the take 18 000 and a fall at 2.7 m/s; a
   stride's first passes, nothing pinned, the C step's two-sole solve 2.2e-2
   N m off the python's (judged by her accelerations and loads since).
   DOD: every segment's peak through stand, walk, turn, strafe, back, stop,
   sit, rise, hop no higher than in her steady walk; a test holds it.
3. **A landing on what is there**: a swinging sole that meets something
   raises her swings by its height, one that meets nothing reaches on down; a
   sliding one lowers the friction she asks. Met at 13 of 13 wobbles: the
   sill, the rug, the hole, asphalt; snow 11, a slide unread (grip.md). The floor
   ahead a plane of her standing soles' pitch (`modal.under`): up a ramp she
   lands where she aims, down at its top; past a crest or down a descent a
   swing hangs 6-27 cm over the floor. Measured and not kept (scenarios.md,
   blind on a flight): a sole standing by its load alone, a swing held at an
   edge till its toes clear it, the floor's rise read from her landings, a
   late swing lowering her - each took a held scene. Left: her centre of mass
   running on past a lone standing sole while the next swing is late, the
   common root on every descent; the first step down a flight, nothing told,
   a drop. DOD: stairs of 0.15 x 0.30 m up and down, a slope of 10 deg up,
   down and across; 3 times in 4 each.
4. **Her floor identified, a text over the general law**: what her
   standing soles meet - slip and sink under their load - read from her
   sensors against what her law assumes (friction 0.6 in her step, a rigid
   floor) and estimated as data; outside that envelope a surface's text
   (lisp) laid over the general law's: the friction her step asks, her
   rhythm, her step's length, her stiffness - no branch in the law. Skates:
   a sole's room a line along its blade, free along it, held across it. As
   it is: her grip felt as a sole slides (`machine.grip`) or told, her
   centre of mass, her going, her landings and her step's cones bounded by
   it outside her envelope, a situation's text capping her rhythm under its
   grip (`situations.lisp`, situation.md: seven read, one laying caps); the
   ice read 0.103 and the slick patch 0.065 a
   pass after a sole meets them, kept 30 s and handed to each landing sole:
   told or unaware she crosses both, and snow, at every wobble (grip.md).
   Left: a floor her soles never slide on - asphalt, snow - read from what
   her holding soles use alone; her insoles reading their normal load alone
   (scenarios.md, blind on a flight); skates. DOD: on skates she stands 10 s
   and goes 2 m by strides across her blades.
5. **Reachable landings ahead**: where each limb lands before her capture
   point is past it, read from her reach (`machine.reach`); none, her height
   lowered and a limb set to meet the fall, knees then hands, head off the
   floor. 80 N met: standing 32 of 32, walking 28 (forward-aside 5 of 8).
   200 N: standing 14 of 32 stand or fall soft, walking 21; the rest strike
   her head at 1.1-4.5 m/s. DOD: 200 N falls with the head under 1 m/s.
6. **One jump** (`machine.leap`, `terminal.views.humanoid_keys`): takeoff from
   the height and distance to land - a hop's landing said with it, its throw
   parameterized continuously by `Space` hold duration (`5 ms` to `250 ms` ->
   `charge` in `[0, 1]`). Met: the walking and running forward leaps (`5 ms`
   vs `250 ms`) against flight-time, dual-sole clearance and distance in
   `test_leap.py`; the standing jump's push from the drives (stunts.md:
   take-off 1.5-1.65 m/s, rise 0.49 legs of the human's 0.55 from the
   crouch, 0.015 m over her stand, two-footed off and down, standing at
   3.9 widths). Left: her soles clear 11 mm, her legs long in flight; she
   lands 0.32 m ahead in the bench's scene, in place in the page's; the
   legs 0.5 axial short of straight as she leaves (the apex over her
   stand); the surety's phase read kept for the seats (stunts.md); the
   chair's two texts reach no hop yet (scenarios.md);
   off and onto a 0.45 m seat her knees work at 0.93-1.0 of their peak, past the
   drives' 50 % margin (demand.md) - the seat's height or the knees' drives to
   decide. DOD: down from a chair, two-footed up onto one (0.45 m), in place;
   landed standing.
7. **The drives for it**: a gait under way, a swing led or her capture point
   past a lone standing sole grants a held drive 1.45x its clamp, past its
   parts' peak. Without it the turns hold and the turnabout better (13 of 13,
   7 granted); the weave falls at 4 of 13 (demand.md). DOD: each scenario's
   peak, rms, speed and heat under 2/3 of its parts.
8. **The takes as her moves** (`tools/sim/stunts.py`, `scenarios.STUNTS`):
   each of the 31 takes a scenario - its scene, her told as a player would,
   her rows measured as the take is (lengths in legs, times in sqrt(leg/g))
   and judged within a sole's length, her list, her arm's cone and one
   natural time of the take's, her head no lower than the human's and under
   1 m/s if she falls. A posture's take is told as its own rows (axial,
   sagittal and pitch read off the take, the rest her balance's, the trunk's
   pitch off her stand; stunts.md): 5 of 31 met - the slick patch, the ice,
   the kneel, the sit onto the chair and the rise from it -, the backflip
   refused for want of a text, 24 short, 214 widths off in all; the crouch
   takes, half a unit deeper with the straight leg their legs' zero, bottom
   out at the human's full squat (knee 140 deg) and sit down at 0.3 m/s; the
   long takes thinned to every second or third frame for the table's 512
   rows.
   Open by mechanism: her hands' jerk 7 700-20 900 m/s^3 where her walk's is
   1 400-1 800 through the sit, the rise, the dance and the bed - the
   whole-body step's upper joints at W_POSTURE 1 against W_COM 1000 make her
   arms the lever when her support flickers; weighed as her turn they stop,
   but the walk's pelvis roll goes with them and, the arms alone, the spine
   folds instead (item 2); the shoves and the
   step's edge put her head on the floor at 1.1-1.8 m/s where a human sits
   down or catches himself (item 5); the sit's hips 0.06-0.56 back run to
   run where 0.48, the bed's edge and lying (item 1); the chair's hops and
   the standing jump's landing (item 6); W at 0.93 m/s where a jog
   is 2.0 and a run 4.5, S at 0.19 where 1.3; the dance's bounce of 0.38
   legs felling her when streamed; a lean back of 13 deg standing met with
   3.7; the stairs (item 3). No spasm: each segment's jerk no higher than
   her steady walk's (item 2). The take's
   human and she by similarity: lengths by the legs' ratio, times and speeds
   by its root, masses and forces by its cube, normative for a woman of her
   size (`figure.MASS_KG` at 165 cm). One evaluation to iterate while a
   parameter of hers or of the loop is tuned (`stunts.evaluate`): how far
   each stunt is off its take in the measures' widths, the total, saved and
   diffed run to run; kinematic in her body's units, the drives' demand
   beside it, never in it, so a motor or a ratio changed alters what she
   reaches, not what a score or a parameter means. Where the drives fall
   short of a flip or a jump, a "performance" drivetrain config beside the
   one stack, the demand measured first (demand.md): laid - the U12 II KV120
   on box A for her spine, hips, knees and ankles
   (`machine/robots/gynoid_mesomorph_medium_performance.toml`) -, her
   standing jump rose 0.03 legs on it where 0.01: the hop's ask was the
   law's, 1.44 of the U12's amps at the hips; from the drives she rises
   0.43-0.51 legs of the human's 0.55 on her own stack (stunts.md). DOD: 31
   of 31 met at every wobble, each stunt's jerk within her walk's.
9. **Her place a glazed mountain lodge** (asked, pictures given): built and
   drawn (`machine.room`, scene 'lodge', [lodge](../findings/lodge.md)):
   the timber walls, the glass gable and its door, the snowy roof, the deck
   with its rail, the stone step and terrace, the loft stair; the wooden
   stair down the slope on the door's line. Open: the descent falls as item
   3's stairs do (`lodge_down` 11.2 s; stairs_down 3.4); the loft stair and
   the stone step walked by no scene; the pictures' snow on the ground, the
   valley's horizon and the gable's glass over the eaves undrawn. DOD:
   lodge_down and a climb of the loft stair held in test_scenarios; her
   sight telling its door and its glass; the ground snowed and the valley
   drawn beyond the rail.
10. **An errand from her side** (found, `tour.step_errand`): at (0.12, 0.28)
    facing 0.22 rad, the lodge's chair 1.47 m off and 56 deg to her left,
    told to sit she walks to (0.54, 0.83), where the walk's row (`stops_in`)
    passes 0.96 m off the dock's line, its tolerance 0.125; braking she goes
    on 0.75 m to its back and is down at 5.19 s. The balloons offer it there
    (F2; F alone the door ahead). DOD: the chair's, the bed's and the door's
    errands from every pose she sees them from, no fall.
