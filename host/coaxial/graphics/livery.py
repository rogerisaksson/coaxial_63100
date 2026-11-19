"""The robot's livery: its paints - black carbon tubes, gunmetal rims, foam pads, anodised drives.

The robot bare of its clothes is held to the pictures of robots beside the tree
(`tools/sim/alike.py`): struts and plates in black, grey and white, a gynoid's actuators
magenta. Half of what is seen of them is dark, a quarter grey, a fifth light, an eighth
magenta in one of the two, and no other hue.

    from coaxial.graphics.livery import TUBE, FRAME, PAD, FOAM, VOID, STEEL, BOLT, BOARD, SERVO, GEAR
"""
from coaxial.graphics.lit import paint

#: A tube's and a bone's black carbon under a clear coat (`lit.TWILL` with `lit.HARD`: solid,
#: its tows told a cell each, a glint on it) and a guard's rim's gunmetal - a cell wide, in
#: carbon's tows a rim was a dashed line -; a pad's foam, light on its face and grey under
#: it; steel - a strut, a rod, a crank, a gimbal, a clamp -, dark: light guards on a dark
#: frame, as the pictured robots' plates on their struts - in a mid grey its frame and its
#: guards' shaded sides were one mass, and in its own near white a third of the robot was
#: light; a board's black laminate: in its green, and a quick-release in its violet, an
#: eighth of the robot was in hues no pictured robot wears; the carbon near neutral - seen
#: through the guards' open honeycomb, its blue cast was a ninth of the robot in blue.
TUBE, FRAME = paint((64, 66, 68), twill=True, hard=True), paint((112, 116, 126), hard=True)
PAD, FOAM = paint((236, 240, 246), hard=True), paint((128, 134, 146), hard=True)
STEEL, BOARD = paint((72, 76, 86), hard=True), paint((44, 46, 54), hard=True)
#: A bolt's and a rivet's head, bright: a dot on a drive's face, in a rim's corner.
BOLT = paint((226, 230, 236), hard=True)
#: A pad's backing: black, no dot of it drawn (`lit.braille`) - a groove between two cells is dark.
VOID = paint((0, 0, 0))
#: A drive: its motor's can and its board anodised magenta, its gearbox a darker one - a gynoid
#: of struts' actuators'. In their frames' blues the drives were grey among grey, and with a
#: gearbox in steel half a servo was.
SERVO, GEAR = paint((255, 48, 200), hard=True), paint((196, 36, 152), hard=True)
