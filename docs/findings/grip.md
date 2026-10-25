# Findings: grip

Her floor's friction as her soles feel it (`machine/grip.py`), told or
estimated, and her law bounded by it; tests/test_floor.py holds it, `ice_told`
and the patches in [scenarios](scenarios.md) walk it.

- The ice, the slick patch and the snow were boxes 1, 1 and 4 mm over the
  floor's slab; a loaded sole sinks 1 to 3 mm (1.0 mm standing) and stood
  through them on the slab's friction, 1.0: on the ice her sliding soles used
  0.07 to 0.22 where the ice's is 0.02. Raised 5, 5 and 40 mm the ice fell at
  3.0-3.8 s and the slick patch (0.06) at 2.8-3.9; snow (0.35) and asphalt held.
  Ice is now a rubber sole's, 0.1; 0.02 is a skate blade's.
- A bearing sole that slides uses the floor's friction, its push's share across
  her IMU's up (`machine.grip`). Its slide is what the draw of her IMU-carried
  speed to her bearing soles has added up to while they bore (`estimate`'s
  slip): walking a level floor 0.071 m/s at most, turning 0.127, on ice 1.36 at
  the 90th percentile; a sole slides past 0.15. Read across a sole's own up, one
  pitched 0.2 rad at its heel's strike read its push 0.2 across; read as what
  her step asked against what a sole used, the drives' lag at a walk's start
  read 0.05 on a floor of 1.0; read as her soles' speed apart in double support,
  rolling soles read 0.05 m/s and more in 94 of 216 passes.
- Read in two passes a slide's share took the slick patch to 13 of 13 wobbles,
  but past 0.15 m/s her estimator reads a turnabout's pivot, a shove's skid, a
  weave's and a trip over the sill as slides, and the bench lost the turnabout,
  the sill and 5 walking shoves. A slide counts with the floor's friction
  against it (45 deg) and its share held within 5 % over 4 passes: on the ice
  0.099-0.101 while its load fell 348 -> 194 N, read 9 ms after it set out; the
  shove's skid read 0.174 -> 0.137, the sill's edge 0.122 -> 0.096.
- On it: her centre of mass asked along the floor no harder than her standing
  soles' grip times g; her going no faster than sqrt(2 STRIDE_M grip g), 0.77
  m/s on ice; a sole landing no further from her than grip times her height,
  0.09 m on ice; each sole's cone its grip (`wrench`, `wbc_cone.c`); a surface's
  text capping her rhythm. Her law takes the grip only outside her envelope -
  under the highest surface's grip, 0.25 -; a sole landing starts from what she
  knows of the floor: kept from the slick patch, her reach was 5 cm on a floor
  of 1.0 for a second and she fell there.
- A slippery floor is a situation of her text (`machine/situations.lisp`,
  `(defsituation slippery (grip below 0.25) ..)`, [situation](situation.md)):
  under its grip her rhythm's amplitudes no more than its, whatever she does,
  blended in over the fifth under 0.25, `ice_walk`'s shape. What she is told of
  the floor - by a player or her eyes, the `grip` command - is her grip's prior.
- Her level walk the same to the digit; the bench 60 of 90 scenes, 80 N standing
  32 and walking 29 as before her grip. Told the ice's 0.1 she crosses it
  standing at 13 of 13 wobbles (`ice_told`); unaware ice 1 of 13, down at
  5.5-6.7 s; the slick patch 6 of 13: after the first slide, not before it, is
  what she does not yet meet.
- Unaware, her grip reads the ice 0.104 three passes after a sole meets it, then
  relaxes toward her prior and each landing restarts there: every step slides
  anew, and her speed climbs 0.4 -> 1.0 m/s. A landing sole started from the
  other's grip, she knows the ice throughout and still falls - ice 0 of 13, the
  slick patch 5: met at 0.69 m/s, her bounds on ice give landings of 9 cm and
  0.98 m/s^2 along, too little to slow in a step or two; told, she meets it
  slow. Not kept.
- Snow holds 11 of 13 wobbles - 13 before its patch was made real, 12 after.
  At 3e-4 rad her standing sole's load fell 379 -> 40 N and her pelvis went
  0.23 -> 2.78 m/s ahead in 0.1 s, her grip read 0.6 throughout. At +2e-4
  (red on main) it is the stop on the snow after the walk, not its far lip:
  braking from 7.3 s her pelvis sinks at 0.6-1.8 m/s, both soles come off
  twice (7.52 s; 7.96-8.02 s, a hop to 0.88 m), the standing sole strikes at
  380-536 N, the other lifts 0.2 m and she is down at 8.6 s; her soles'
  lowest points and the floor felt agree within 2 cm throughout. Her grip
  reads the snow, 0.32-0.34 from 6.5 s, and her step asks 0.6 to the end:
  `outside` takes a felt grip only under a situation's bound, and the snow
  unaware is none (nothing told, the envelope's top 0, nothing outside it).
  Unsaid, the envelope's top her law's own MU would hand the step the 0.34 -
  and every slide she reads on a floor of 1.0 (a pivot's, a shove's, a
  weave's: 0.27-0.55) with it; the whole scenario suite decides.
- Unaware ice, traced: her left sole read 0.103 one pass after it met the ice
  (2.31 s) and the slick patch 0.065 (0.06); the grip relaxed to her prior
  over 3 s and was back inside her envelope (0.25) within a second, each
  landing sole started at the prior, she drove on to 1.06 m/s, slid 0.3-0.4
  m/s, flew at 5.9 s and fell at 6.6-6.7 s. Relaxed over 30 s and a landing
  sole starting from the floor the other stands on: 0.6 -> 0.2 m/s by 4 s,
  slips under 0.04, across the ice standing; ice, the slick patch and snow 9
  of 9 wobbles, the turns, the turnabout, the weave, the spin, the sill, the
  rug and the hole as before; her flat walk 51 s, 0.70 m/s, 408 J/m, 2.55
  off the band. Ice and the slick patch join `test_scenarios`.
- A sliding sole's share is the most the floor holds, a holding sole's the
  least: what is not friction - the patch's 5 mm edge pushing a toe, the
  sole's own inertia - only adds to a push across. On the slick patch (0.06)
  its far edge read 0.104-0.113 and her grip rose to 0.074; she split, the
  sole standing on the patch sliding back as her centre of mass passed it 19
  cm in 0.54 s, at 0 and 2e-4 rad. A slide now only lowers her grip, and a
  slide read over its plateau is taken whole, no time to it (`SLIDE_S` 0.006
  s gone; 0.004 stood the patch too): the patch 3 of 3, ice, ice told, snow,
  asphalt and the kept replays at every wobble, her walk to the digit.
