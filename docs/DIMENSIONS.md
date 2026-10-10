# Dimensions

Her axes by their drive's type: 21 driven on 2 types, 6 without a drive. The
candidate of record, laid in `machine/drives.py` and written by
`tools/sim/dimensions.py --write`; why it is this one, docs/findings/stacks.md.

## Types

| type | drives | motor | gearbox | inverter | stack mm | kg | peak N m | holds N m | deg/s | rotor kg m2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| U8 | 15 | 87 x 27 mm, KV 100 | 64 mm, 1:30 | 100 mm, 100 A | 100 x 64 | 0.73 | 108 | 40 | 960 | 0.177 |
| MN6007 | 6 | 67 x 26 mm, KV 160 | 64 mm, 1:30 | 100 mm, 100 A | 100 x 63 | 0.63 | 51 | 19 | 1536 | 0.063 |

A stack its inverter's disc, its outrunner and its gearbox on one axis; kg
with the inverter. Peak at the gearbox's output at the inverter's amps;
holds, for ever in still air (`drives.COOLING` 1.0); deg/s unloaded
at the pack's lowest, 48 V; the rotor as its joint feels it through the
box.

## Axes

### Type U8

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ankle | 2 | on the shank, its inverter on the thigh | a rod, a pair with the ankle roll | 27.3 | 197 | 140 | 1055 |
| hip roll | 2 | on the hip yaw | a spur pair 1:1.67 | 50.1 | 180 | 129 | 575 |
| spine roll | 1 | on the pelvis | a four-bar | 40.0 | 144 | 77 | 720 |
| ankle roll | 2 | on the shank | a rod, a pair with the ankle | 17.9 | 129 | 100 | 1611 |
| spine | 1 | on the spine roll, its inverter on the torso | - | 30.0 | 108 | 108 | 960 |
| waist | 1 | on the torso | - | 30.0 | 108 | 77 | 960 |
| hip yaw | 2 | on the pelvis | - | 30.0 | 108 | 52 | 960 |
| hip | 2 | on the hip roll | - | 30.0 | 108 | 108 | 960 |
| knee | 2 | on its axis, its inverter on the thigh | - | 30.0 | 108 | 108 | 960 |

### Type MN6007

| axis | drives | sits | through | ratio | peak N m | clamp N m | deg/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| neck | 1 | on its axis, its inverter on the torso | - | 30.0 | 51 | 15 | 1536 |
| head | 1 | on its axis, its inverter on the torso | - | 30.0 | 51 | 8 | 1536 |
| shoulder | 2 | on the torso | - | 30.0 | 51 | 40 | 1536 |
| elbow | 2 | on the upper arm | - | 30.0 | 51 | 25 | 1536 |

### No drive

| axis | joints | held by |
| --- | --- | --- |
| toes | 2 | a spring, 10 N m/rad about 0 deg |
| gripper | 2 | a stop, at 80 deg |
| wrist | 2 | a stop, at 0 deg |
