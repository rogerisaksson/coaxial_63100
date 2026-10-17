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
- A surface is text (`machine/surfaces.lisp`, `(defsurface NAME (grip G) ..)`):
  under its grip her rhythm's amplitudes no more than its, whatever she does,
  blended in over the fifth under G; one so far, slippery under 0.25 with
  `ice_walk`'s shape. What she is told of the floor - by a player or her eyes,
  the `grip` command - is her grip's prior.
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
