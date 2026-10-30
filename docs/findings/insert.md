# Findings: the anti-spasm insert

Her errands said from every spot (`tools/sim/house.py --said`, 112 runs a
verb; a spasm an arm past 87 deg/s, her neck past 37 - her walk's), traced pass
by pass, against the insert's five roots (`machine.law.supervisor`,
anti_spasm_supervisor.lisp):

- A regime switch at her brake's last landing (sit_chair from -0.1, -0.4, 0):
  the sole lands at 6.312 s and her step's strides go 1.00 -> 0.00 in one law
  pass; it bears 599.5 N, the left shoulder's torque 1.2 -> -54.6 N m, its
  clamp, the arm at -1 208 deg/s 100 ms on: the standing rounds trade what her
  soles cannot give onto it (W_POSTURE 1).
- A contact let go and back with the switch, reclining on the bed's edge
  (lie_bed from -0.1, 0.8, pi/2): the bed bears 0.60 of her on two points of
  ~90 N; a sole lets go while the other stands bearing 0.0 N, strides 0.23 ->
  1.00 in a pass, the bed's points [99.7, 82.1] -> [248.9] -> [73.9, 37.4] N;
  her neck 5.2 -> 50.5 N m (its WEP, 51.3), 1 282 deg/s in 60 ms.
- A limit cycle seated (-0.1, -0.4, 0): the seat's force flips each 6 ms -
  116, 33, 151, 33, 52, 37, 56 N -, her step's scale 0.44 to 0.93 and back,
  neck and head at +-51.3 N m pass by pass; the seat let go, her neck slams to
  -120.8 deg at 2 114 deg/s. Neck, head, shoulders and ankles have no stop in
  MuJoCo, their setpoints a room (`sprung._Q_LIM`: the neck -20 to 26 deg).
- Standing at a corner of her way: the left shoulder at its 58 N m clamp for
  0.3 s, the arm wheeled -8 -> -232 deg (its room ends at -65).
- No count to hunt, nothing ringing: her encoders are MuJoCo's exact angles
  (an A1335's 12 bits through 30:1, 5.1e-5 rad), her gearboxes rigid
  (`physics.BOXED` 0; boxed, a rotor on its wind-up rings at 14.8 Hz in the
  ankle to 41.4 in the shoulder). Her law's 3 ms passes step her drives' feed:
  her walk to 237 kN m/s a drive, the insert's 850 N m/s cuts it 280-fold.
- Her tank, a reaction mass's energy in its own inertia past a budget handing
  its drive to a damper that meets the clamp at the budget's rate (outside the
  tree first): budgeted by her lodge walk's 99.9th percentile (neck 101,
  shoulder 326 deg/s), the chair's 120 runs fell 39 -> 26 (24 saved, 12 new),
  its neck where both held 618 -> 178 deg/s at the median, 1 940 -> 203 at
  p90, 2 877 -> 334 at most, the arms at most 932 -> 394; the bed's 96 fell 5
  -> 7, its neck 1 259 -> 123, the arms 1 105 -> 395. Re-laid with the tank on
  that budget moved (the head 0.32 -> 0.05 J: strike shakes and the room's far
  wall made it). Across a joint's whole room (neck 169, shoulder 790 deg/s)
  the arms were never held: 50 chair runs fell 15 -> 13, the neck 742 -> 294.
  Her walk's own, its rate over a sole's settling: her head 4.5 deg/s, her walk
  holding it still.
- Outside the tree, two and three spots: a Schmitt latch on each borne segment
  (on at 15 N, off under 5, 45 ms dwell), its point held while latched, fell at
  the brake's spot, seated, at 24.0 s; her step's strides carried on a
  critically damped second order at 10 /s took the brake's arm swing at 6.43 s
  and left one at the corner's stand, 1 098 deg/s, the seated neck 1 966 ->
  1 354; her neck, head and shoulders held (`motion.control`'s w_up) past
  their rooms or their energy: 2 of 3 spots fell.
- The insert as her master bus (`wbc_bus.c`, her drives' 1 kHz pass), her walk
  on her loop, 51 s, each stage alone against none (0.70 m/s, 408-410 J/m):
  gate 411, low-pass 410, jerk 409, bridge 417, handover 409 J/m; all but the
  slew 0.72 m/s, 416 J/m. The tank alone drained to its 0.5 J floor in 2.6 s
  through one Q 0.5 band-pass biquad: her stride's 1.8 Hz leaked at -12 dB, her
  walk's own 4.5 W; a 6 Hz high-pass then a 35 Hz low-pass hold it. Its
  preload told to her drives' observer as given torque was a bias: 455 J/m.
- The slew at 850 N m/s felled her alone at 11.9-13.1 s, on her law's command
  or on the drive's feed; her law asks her hips 10 kN m/s at p99, 19 at most,
  and a let-go 114-117 N m in a pass; the ducked, narrowed walk fell at 118.6
  deg. Each drive's own instead: its peak x 2 pi x 2.5 kHz.
- Her damped least squares on the insert's raised cosine: her standing leg's
  sigma_min is 0.023 at the median, under 0.045 90 % of stance. On both legs
  0.61 m/s, 483 J/m, 2.23 off a woman's (as built 0.70, 408, 2.52), its last
  bits stable; on the swung leg alone 0.68-0.69, 416-420, 2.68-2.85; its
  larger with 0.02 there 0.68, 420, 2.95. On the swung leg it acts in its
  swing's first 20 ms alone: the trailing knee leaves at 13-16 deg (sigma_min
  0.035-0.044, lambda to 0.063), at landing 26 (0.07). With the bus she
  walked on stilts (2.87 off, knee at lift 18.5) till her heel's cap was a
  woman's at lift, 2.58 ([feet](feet.md)).
- The buffers before her stops: 0.70 m/s, 407-408 J/m, 2.51-2.52 off; the
  ducked walk bit for bit with and without them.
- Her strides handed over through three 85 ms lags each way, on ice her parry
  waited - a stride's rise 0.31 -> 1.0 handed 0.95 -> 0.64 - she tilted 9-27
  deg and fell at 4.54 s (slip, every wobble); at a third of it each, 4.56. A
  rise at once, a fall lagged: slip, chair, sprint, rug, snow, stutter, moves
  and hole stand, her walk 0.70 m/s, 407 J/m, 2.61 off.
- The jerk at 12 000 rad/s^3 on her drives' observers' ask alone: they pushed
  against the rest, her arm yielded 0.24 rad at a jamb where 0.55; her
  drives' torque following it, her law's 3 ms asks ramped out: on stilts, 465
  J/m, 2.80 off, her leaps and viewport errands red. Each drive's own instead:
  its slew over its rotor's inertia.
- The bridge's 1.2 N m preload as a bias in the command's sign - one
  reversible drive a joint, no antagonist to pull against - with its window's
  65 % slew cut sat her on one sole in the chair (144 and 0 N, hips 13.3 and
  -12.3 N m) and she fell rising at 10.69 s; either alone she rose. Gone: the
  window's cut alone.
- The soft clips' knee at zero (1.5 lim (u - u^3/3)) took 5 % of a hop's push at
  its drive's own jerk: her leap's charge lengthened it 0.10 and 0.08 m where
  0.15, a jamb yielded her arm 0.24 rad. Their knee 6 dB under the ceiling:
  0.18 and 0.24 m, 0.55 rad.
- The bridge's window on every change inside it held her lumbar's torque near
  zero at 35 %: the viewport's chair fell at 4.73 s on every nudge, on the
  lumbar alone at 5.71. On a reversal alone - the gap a transmission crosses -
  she sat.
- The tank tripped at its floor too: a hop's push drained its 4 J in 30 ms, every
  drive rolled back and she rose 669 mm off one sole. Tripped by the detector
  alone, as the text has it - parasitic oscillation in its band - she jumped
  as before.
- Her knees' stop's damper on the bus's gated, low-passed rate braked a jump's
  knee lock late: she rose 410 mm (gate) and stepped on 0.84 m after landing,
  0.41 (low-pass); on the encoder's rate itself, every stage on, 331 mm and
  -0.06 m.
- Each alone - the D low-pass, the reversal's bridge, the handover with its
  rise at once - felled her walking a right turn (replay walk_turn_right_at_0.69,
  8.4-8.9 s at every nudge; the base stands to 13.5 s): with the handover her
  law stood her at 7.58 s while her step still strode (0.96-0.88), both soles
  let go, she landed on one at 360 N and tilted 17 deg. Out of the bus, open.
- The insert's comparators on what bears her off her soles, gating which
  borne segments her step plans on (on 0.18 of her weight, off 0.06, 45 ms
  dwell): the chair's pelvis bounced off the seat - its share 0.32, 0, 0.47,
  0, 0.42, 0 -, each landing held out of her step up to 45 ms while it bore
  0.3-0.47 of her, and her rise fell at 9.26 s; every manifold latched, she
  rose. A debounce on what the step plans on hides a force: hysteresis is a
  mode's, her stance's.
- The buffers at their width alone stop an approach of 1.9 rad/s at most
  (0.06 rad, 140 /s^2, zeta 1): in the standing jump's crouch her soles'
  centres of pressure sat on their heels and her step threw her head for her
  trunk's lean (her neck 55-73 rad/s^2, its posture's row the cheapest), her
  neck met its buffer at 13 rad/s, ran 0.36 rad past its stop and she landed
  0.35 m on. Engaged as far again as the spring takes to stop the approach
  (v / (e w), 0.4 rad at 13 rad/s): 0.044 rad past.
- Her swung knee's torque steps - -36 to +39 N m at its lift, -40 to 0 at
  its rise's end, the clearance's cubic stepping its sole 37 m/s^2 - count
  three hot half cycles and trip its tank: at a 0.6 rad heel's cap with the
  knee's lock buffered, 28 trips in 12 bed errands and, wobbled 3 poses x 8
  headings, 13 docked, 7 fell. At a woman's heel (36.2 deg) the lock
  buffered, 12 of 12; the lock left to the drive pass's knee spring alone, 9
  of 11 shoves from behind, weaves and chairs fell. The clearance on a
  quintic, C2: 2.80 off a woman's, 441 J/m; on a quartic, C2 at its top:
  3.22, 455 J/m (as built 2.52, 409); neither kept.
- After the run leap her law asks her down 4-6 m/s^2 and on 2-6 m/s^2 on one
  sole on its ball (5.86-6.26 s); her step plans that sole 243 -> 0 N as
  asked. Unbuffered, her knee hits its lock at 7.8 rad/s and she lands on
  both (the base the same); its lock's buffer reached ahead, she sank to
  0.47 m and fell, 4 starts in 5.
- Her support off her soles on the insert's comparator (latched at 0.18 of
  her weight, let go 45 ms under 0.06, its box held) where its share ramped
  to 0.15: reclining on the bed's edge it had gone 0.66, 0, 0.17, 0.71 in a
  second and she slid onto the mattress; test_errands 19 of 19 (17), the
  viewport's chair and bed rise, scenarios 66 of 67 (snow, as on main). Its
  slot first named SUPPORT, modal's ZMP mode of that name, 1.0, laid it on
  her right sole's stance: slip fell at every wobble.
- The comparators on her soles, outside the tree, as the loads her stance's
  modes read (on 0.18 of her weight, held through dropouts under 0.06 for 45
  ms, 18 ms blanking): her stance's toggles within 45 ms, bed 27 -> 7, jump
  40 -> 2; but a sole held standing while it unloads is pushed on as it
  leaves. Let go as the sole rises: every leap's flight 0.05-0.11 s where
  0.18-0.22, 3 errands red. Let go as her centre of mass rises past 0.1 m/s:
  the walking leaps and the jump hold, the run leap at no charge flies 0.06
  s, snow falls at every wobble (1 of 3 as built). Let go 18 ms after: 3 of
  4 leaps 0.045-0.072 s. Not kept: her step plans on her stance, and a
  contact held past its unloading is a force planned where none is.
- Her drives damp a step of their rates once the D channel settles: a pass
  after it 0.3 % of the damping and the preload's sign, +2.5 W; at the gate's
  attack and the low-pass's 2 % time (26 ms), -6.64 W where -6.55 bare.
