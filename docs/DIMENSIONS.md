# Dimensions (written by host/tools/sim/dimensions.py, not by hand)

The robot's axes by drive type: 21 driven on 3 types, 6 without a drive.
The candidate of record, laid in `machine/body/drives.py` and written by
`tools/sim/dimensions.py --write`; the rationale: docs/findings/stacks.md.

## Types

| type | drives | motor | gearbox | inverter | stack mm | kg | peak N m | holds N m | deg/s | rotor kg m2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| U8 | 11 | 87 x 27 mm, KV 100 | 64 mm, 1:30 | 100 mm, 100 A | 100 x 64 | 0.73 | 108 | 42 | 960 | 0.177 |
| MN6007/B45/coaxial_63100 | 4 | 67 x 26 mm, KV 160 | 64 mm, 1:45 | 100 mm, 100 A | 100 x 63 | 0.63 | 77 | 33 | 1024 | 0.142 |
| MN6007/B/coaxial_63100 | 6 | 67 x 26 mm, KV 160 | 64 mm, 1:30 | 100 mm, 100 A | 100 x 63 | 0.63 | 51 | 22 | 1536 | 0.063 |

A stack its inverter's disc, its outrunner and its gearbox on one axis; kg
with the inverter. Peak at the gearbox's output at the inverter's amps;
holds, indefinitely in still air (`drives.COOLING` 1.0); deg/s unloaded
at the pack's lowest, 48 V; the rotor as its joint feels it through the
box.

## Axes

### Type U8

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| hip roll | 2 | on the hip yaw | a spur pair 1:1.67 | 50.1 | 180 | 128 | 575 |
| spine roll | 1 | on the pelvis | a four-bar | 40.0 | 144 | 76 | 720 |
| spine | 1 | on the spine roll, its inverter on the torso | - | 30.0 | 108 | 108 | 960 |
| waist | 1 | on the torso | - | 30.0 | 108 | 76 | 960 |
| hip yaw | 2 | on the pelvis | - | 30.0 | 108 | 47 | 960 |
| hip | 2 | on the hip roll | - | 30.0 | 108 | 108 | 960 |
| knee | 2 | on its axis | - | 30.0 | 108 | 108 | 960 |

### Type MN6007/B45/coaxial_63100

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ankle | 2 | on the shank | a rod, a pair with the ankle roll | 45.1 | 154 | 109 | 1022 |
| ankle roll | 2 | on the shank | a rod, a pair with the ankle | 21.1 | 72 | 72 | 2181 |

### Type MN6007/B/coaxial_63100

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| neck | 1 | on its axis, its inverter on the torso | - | 30.0 | 51 | 16 | 1536 |
| head | 1 | on its axis, its inverter on the torso | - | 30.0 | 51 | 5 | 1536 |
| shoulder | 2 | on the torso, its inverter on the torso | - | 30.0 | 51 | 36 | 1536 |
| elbow | 2 | on the upper arm | - | 30.0 | 51 | 23 | 1536 |

### No drive

| axis | joints | held by |
| --- | --- | --- |
| toes | 2 | a spring, 10 N m/rad about 0 deg |
| gripper | 2 | a stop, at 80 deg |
| wrist | 2 | a stop, at 0 deg |
