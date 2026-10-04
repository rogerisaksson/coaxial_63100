# Dimensions

Her axes by their drive's type: 21 driven on 2 types, 6 without a drive. The
candidate of record, laid in `machine/drives.py` and written by
`tools/sim/dimensions.py --write`; why it is this one, docs/findings/drives.md.

## Types

| type | drives | motor | gearbox | inverter | stack mm | kg | peak N m | holds N m | deg/s | rotor kg m2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| A | 7 | 68 x 30 mm, KV 110 | 84 mm, 1:30 | 100 mm, 100 A | 100 x 86 | 1.20 | 203 | 132 | 1056 | 0.158 |
| B | 14 | 60 x 16 mm, KV 90 | 64 mm, 1:30 | 70 mm, 50 A | 70 x 66 | 0.55 | 124 | 52 | 864 | 0.062 |

A stack its inverter's disc, its outrunner and its gearbox on one axis; kg
with the inverter. Peak at the gearbox's output at the inverter's amps;
holds, for ever in the oil (`drives.COOLING` 0.3, assumed); deg/s unloaded
at the pack's lowest, 48 V; the rotor as its joint feels it through the
box.

## Axes

### Type A

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ankle | 2 | on the shank, its inverter on the thigh | a rod, a pair with the ankle roll | 27.3 | 370 | 140 | 1160 |
| spine | 1 | on the spine roll, its inverter on the torso | - | 30.0 | 203 | 142 | 1056 |
| hip | 2 | on the hip roll | - | 30.0 | 203 | 171 | 1056 |
| knee | 2 | on its axis, its inverter on the thigh | - | 30.0 | 203 | 171 | 1056 |

### Type B

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| hip roll | 2 | on the hip yaw | a spur pair 1:1.67 | 50.1 | 207 | 129 | 517 |
| spine roll | 1 | on the pelvis | a four-bar | 40.0 | 165 | 77 | 648 |
| ankle roll | 2 | on the shank | a rod, a pair with the ankle | 17.9 | 148 | 100 | 1450 |
| waist | 1 | on the torso | - | 30.0 | 124 | 77 | 864 |
| neck | 1 | on its axis | - | 30.0 | 124 | 15 | 864 |
| head | 1 | on its axis | - | 30.0 | 124 | 8 | 864 |
| shoulder | 2 | on the torso | - | 30.0 | 124 | 40 | 864 |
| elbow | 2 | on its axis, its inverter on the upper arm | - | 30.0 | 124 | 25 | 864 |
| hip yaw | 2 | on the pelvis | - | 30.0 | 124 | 52 | 864 |

### No drive

| axis | joints | held by |
| --- | --- | --- |
| toes | 2 | a spring, 10 N m/rad about 0 deg |
| gripper | 2 | a stop, at 80 deg |
| wrist | 2 | a stop, at 0 deg |
