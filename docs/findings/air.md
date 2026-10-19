# Findings: air

The air every craft flies through (`machine.weather`); the quad's frame in it:
[quad](quad.md).

`machine.weather`: `AIR`, `blown`.

- One kind of weather at a time: calm, constant, gusty, changing, turbulent,
  20 s each or one kept (W), eased in on an S over 8 s. A wind of 0.5-4 m/s
  at 5 m, sheared along the floor; 1 - cos gusts of 2.4-4 m/s over it in 4-8
  s, eddies of 0.1-1.2 m/s rms of two stages each, the frame's and each
  disc's - these and its rise none at the floor, whole from 3 m. The frame's
  drag is against it; a disc's thrust rises with air rising and over the
  floor, 1 / (1 - (R / 4d)^2), 4/3 within half a radius.
- The air grown and subsided: what it changes by at a frame, 0.09-1.5 m/s^2
  rms by kind, a turbulent kind's 4.6 at the most, where on gusts in 1.5-3
  s, eddies of one stage - white in what they changed by - and a kind eased
  on from its first 1.2-14, 44 at the most: kicks.
- The law is told none of it and learns the wind from its drag in 0.12 s:
  a hover at 5 m in 3.9 m/s stood 0.68 m off its spot unlearnt, never
  `held`; 0.03 m learnt; in gusts to 6.8 m/s 0.23 m at most.
- On its instruments (`machine.senses`: the attitude 0.2 deg off, RTK fixes
  at 20 Hz 60 ms behind, 2 cm and 3 cm/s) a hover at 1.5 m holds its spot
  within 5-6 cm rms, 8-10 cm at most, in every air, calm too - the tilt on
  gravity through the filter, not the wind; on the truth 0-2 cm, 7 cm in
  gusts. The signal lost 12 s, the star camera holding the attitude: 0.53 m.
  Dryden's scales - eddies over 15 and 2 m of the wind, risen from 1 m -
  took the ornithopter's dive reversal from 1.37 to 1.52 of its cyclic a
  pass ([aircraft](aircraft.md)): not kept.
- Its push learnt and not the wind: a lap in a constant wind passed a gate
  0.65 m off where 0.37 in still air (the push turns with the frame's own
  speed through a bend); the wind itself: 0.36-0.44.
- A bend planned on what the wind leaves of its grip: gusty 0.78 -> 0.50
  m, turbulent 0.94 -> 0.58, a lap 1 s slower.
- On the page's four boards the routine's fall went 3.5 m downwind and was
  back on its spot by its stop; laps 29.0 and 25.7 s, a gate 0.23 m off at
  most, boards to 0.76 of their envelopes, none throttling.
- The pointer shows the wind's direction as the view has it, its size and
  kind (`terminal/views/quad/wind.py`).
