# Architecture: Quad

The quad on four coaxial boards: its frame in MuJoCo, the core in C it
flies on, its action space, its flights as text, its race. The shared
flight stack: [aerodynamics](aerodynamics.md); drawn: [views](views.md).
Findings: [quad](../findings/quad.md), [streams](../findings/streams.md).

| Where | Is |
| --- | --- |
| `machine/quad` | a quad on four coaxial boards in MuJoCo: its frame and on what its rotors turn |
| `machine/weather` | the air every craft flies through: a kind of weather at a time, its wind, gusts and eddies |
| `machine/senses` | what a craft's law reads of its frame: the board's IMU, an RTK receiver on the 10 Mbit RS485 bus, a star camera where the signal is lost |
| `machine/flightpack` | a craft's flight pack: its cells' volts by the charge left, its bus under its drives |
| `aero/` | a frame flown from a compiled intent (machine.aerolisp's payload): its blend, oscillator and observers, the lift inside its constraints (aero.c), where the rotors' axis is asked and the torque there (aero_turn.c), the thrusts shared by an active set (aero_share.c); a choice a mask (aero_bits.h), no branch; host-tested; on the image behind device 14 (cmd_aero.c, `coaxial/devices/aero`) |
| `machine/pull` | the quad's action space, no branch: a rotor's top on its clamp and on its link's volts, the four's on their pack's charge, its reach, its stage's watts - from motor, propeller and pack, its tables laid once |
| `machine/intents` | the quad's flights as text, and each stream's seconds |
| `host/tools/sim/` | quad_race (the quad's laps through fixed flights, a search over its plan and its line); streams (a text's streams flown on aero/, ideal rotors; a hover asked from upsets) |
| `host/tools/cores/aero.py` | aero/ through ctypes, the core the firmware builds |

## Suites

- `test_aero_core.py` holds aero/ to its form - no branch in its sources, no
  flag set from its data in this host's code -, its share to the best of
  every set of rotors held at an end, its impedance to the closed form in a
  vacuum, a wind told from a push (`aero_kit.py` its rig).
- `test_quad_streams.py`: the text's payloads and the power loop flown, a
  hover in the tour's air and from upsets, the page's routine and the course
  from their cards' rows.
- `test_quad_pull.py` holds the action space (`machine.pull`) to the board's
  law on the stand-in: a rotor's top, its reach, its stage's watts, the four
  on their pack.
- `test_quad.py` its routine and payloads on aero/; `test_quad_course.py`
  its course on ideal rotors; `test_views_quad.py` and
  `test_views_quad_flight.py` the QUAD page on four stand-in boards;
  `test_views_inset.py` the quad's camera on its gimbal in the corner.
