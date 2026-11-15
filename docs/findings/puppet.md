# Findings: the puppet

A take made a move of the robot's own: the take as G-code, played as a puppet
in a scene, bound to a stage of the robot's text. The takes as scenarios:
[stunts](stunts.md).

## From a take to the robot's text: the puppet

A take becomes a move of the robot's own in three steps, each a command:

    python tools/sim/gcode.py "lying/Lying Down from Sitting"
    python tools/sim/puppet.py take lying_down_from_sitting --trunk 55 --slow 1.5 --gcode danced_lie
    :joint-program :danced-lie                      ; on a stage of the robot's text

The first writes the take as humanoid G-code, the general rig's and its own
(`gcode/<take>.gcode`, `machine.texts.gcode`). The second plays the robot's
program in a situation the take was not recorded in - placed in a scene, told an
act, the program on the boards' servos from the pass its text's program ends -
as a regime: phases ended by what it senses (loads by body, places along the
furniture's axis, the trunk's elevation) with regulators over the program's own
targets (`tools/sim/puppet.py`). The third binds what the robot was told back,
written as G-code, to a stage of its text (`maneuver._program`,
`machine.process.joints`). What the puppet found on the bed: the heel stroke
(poses solved from the robot's figure's lengths: the heels planted 0.22 m from
the hips, the pelvis bridged 0.10, the heels pushed to 0.45, 0.14-0.16 m a
stroke supine), the lazy crab (the trunk held 55-60 deg by the anchored hips
while the fists bear behind, a leg at a time over the edge, the heels set by
their felt load, a push sized to the mark, the lie with the heels lifted: flat
within 8 deg, no fall, +-0.15 m of the mark) where every open-loop crab fell,
and the take itself danced: recorded on the bed's side it rolls the robot off
the side at its foot end raw, slides the robot off the end held at full pace,
and held at 55 deg at two thirds pace lays the robot's flat on the bed at the
pull and lie's own landing. In the robot's law the lean needs `:trunk-hold 4.0`
on its stage (`maneuver._held`); the leg over the edge onto the furniture has no
act yet (`:limb-lift` raises the hanging foot 0.19 m, under the top).
