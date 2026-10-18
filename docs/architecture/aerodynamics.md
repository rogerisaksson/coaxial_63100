# Architecture: Aerodynamics

What flies, on one flight stack: intent as text, the kinematics in a
vacuum, figures as rows and routines as cards, the grounds; the airframes in
MuJoCo, each on its 63100 drives. The quad and the core in C it flies on:
[quad](quad.md). Findings: [aircraft](../findings/aircraft.md),
[envelope](../findings/envelope.md).

| Module | Is |
| --- | --- |
| aerolisp | flight intent as text - `(stream_inject ..)`: latent targets, a stiffness and a damping a mode, an oscillator, task constraints - compiled to aero/'s payloads, a stream laid over the one before; a vehicle's blocks by `defblock` and `use` |
| aerokin | the quad's kinematics in a vacuum, no branch: a row of `aerobatics`' setpoints and the frame as it flies to aero/'s payload - a collective, the rotors' axis's tilt in the heading, the rates its turn gives |
| aerobatics | a figure a row of setpoints (`aerokin`'s), a routine a card of them |
| grounds | the gates in the air and what stands about them |
| airfield | the aerobat's runway, its gate course laid for a plane, what stands about them |
| course | the gates among trees, houses and cars, a lap a line flown as rows |
| helicopter | the 3D helicopter in MuJoCo: collective and cyclic swashplate, tail rotor, stunts |
| ornithopter | the Dune-style four-wing flapping ornithopter in MuJoCo: articulated wings, dive fold, stunts |
| aerobat | the 3D RC aerobatic monoplane in MuJoCo: prop-hang, high-deflection surfaces, stunts |
| wingborne | a plane's wing and tail in the air, its run on its wheels and its law wing-borne |

- The texts: `aerodynamics.lisp`, `dynamics.lisp` and `kinematics.lisp`
  shared; `quadcopter.lisp`, `helicopter.lisp`, `ornithopter.lisp` and
  `aerobat.lisp` a craft each.
- Drawn by `coaxial/graphics/<craft>.py`; flown on `terminal/views/<craft>/`
  and `show_<craft>.py` ([views](views.md)).
- `test_helicopter.py`: the helicopter and the ornithopter in MuJoCo, the
  three's boards and views; `test_aerobat.py`: the aerobat's routine on its
  airfield and its gate course in wind.
