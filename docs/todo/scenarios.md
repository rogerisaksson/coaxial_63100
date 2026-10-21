# Scenarios, by the general case

Each a test in MuJoCo: placed, commanded as a player would, judged by no
fall, its goal reached and its jerk (`tools/sim/scenarios.py`; those that hold
in `test_scenarios`, 20 of 31 at every wobble now:
[scenarios](../findings/scenarios.md)). In order: each a mechanism, the scenes it
serves its DOD; none an entry a case.

1. **Her support, whatever bears her** (`machine.borne`): a segment the skin
   feels pushed up is in her support region, grounds her, and is a contact of
   her whole-body step. Met: the chair 2 of 3 wobbles (-2e-4 sits 0.66 s of
   1); the bed 11 - rising
   at -5e-4 the mattress's share flickers (2), at 2e-4 it bears half her
   weight 0.51 s of 1. Open: lying flat and lengthwise - told to lie she
   sits on the bed's edge and reclines 27 deg, her soles 44 mm up, the bed
   bearing 1.08 of her weight, and rises from it; her pelvis and trunk
   pitched apart; the floor mattress, told to lie she crouches, her pelvis
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
   points give no moment (jerk.md). DOD: every segment's peak through stand,
   walk, turn, strafe, back, stop, sit, rise, hop no higher than in her
   steady walk; a test holds it.
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
   `charge` in `[0, 1]`). Met: standing high two-footed jump (`5 ms` vs
   `250 ms`), walking forward leap and running forward leap (`5 ms` vs `250 ms`)
   verified against flight-time, dual-sole clearance and distance criteria in
   `test_leap.py`. Left: the chair's two texts reach no hop yet (scenarios.md);
   off and onto a 0.45 m seat her knees work at 0.93-1.0 of their peak, past the
   drives' 50 % margin (demand.md) - the seat's height or the knees' drives to
   decide. DOD: down from a chair, two-footed up onto one (0.45 m), in place;
   landed standing.
7. **The drives for it**: a gait under way, a swing led or her capture point
   past a lone standing sole grants a held drive 1.45x its clamp, past its
   parts' peak. Without it the turns hold and the turnabout better (13 of 13,
   7 granted); the weave falls at 4 of 13 (demand.md). DOD: each scenario's
   peak, rms, speed and heat under 2/3 of its parts.
