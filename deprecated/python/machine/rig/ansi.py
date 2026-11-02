"""Moved from host/machine/rig/ansi.py: BLUE, HALF_UP - nothing in the tree named them."""


#: The cold end of a three-band scale - under normal, where green is
#: normal and red is past it. Brighter than the thermal ramp's blues,
#: which are a field's floor and not a band to read at a glance.
BLUE = 33


#: The other half, for a cell whose LOWER row is off the picture. Without
#: it that case was drawn as a background-coloured space - a whole cell
#: painted where half was meant - so the bottom edge of a round board
#: resolved a cell coarser than the top and read as blocky.
HALF_UP = '▀'
