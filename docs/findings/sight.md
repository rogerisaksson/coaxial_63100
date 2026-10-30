# Findings: her eyes

A stereo pair on her head (`machine/eyes/sight.py`, `machine/eyes/stereo.py`,
`physics.SIGHT`), rendered from the world by MuJoCo, the disparity by block
matching on the card (`wgpu` WGSL compute / `cupy`) or numpy, the depth from
it, the points into a map she keeps of the room (`machine/world/room.py`), its things
told by shape and colour, a way planned round them, and the page's
picture of her view (`coaxial/graphics/inset.py`); test_sight.py holds it.
Nothing of hers reads the world's geometry.

## The eyes

| | |
| --- | --- |
| cameras | two on the head, 0.1 m apart, 60 deg high, 320 x 240, looking 15 deg down |
| a look | 3.6 ms on the card (both renders 2.0 ms, 3-stage WGSL/CuPy block matching 1.60 ms; 95.6 ms on CPU, 99.85 % kept agreement, max 0.0173 px) |
| kept | 50 % of the pixels: textured (a window's spread over 1.5), unique (the next best 1.15 times the best, off its own neighbours) and picked back by the right eye within a pixel |
| depth | within 1.4 % of the renderer's at the median, 3.3 % at the ninth decile |

- A checker texture repeats: block matching matched a square to the next and
  the map filled with phantoms (1 747 cells); so did speckles laid 12 a
  face, 4.5 px apart at 3.4 m. A cube's speckles on one scale in space
  match once.
- The model's `statistic extent` was 10 km from the floor's parked patches:
  the cameras clip nearer than a hundredth of it and saw nothing; set to 8 m.
- The model has no light: four on the ceiling for her eyes.
- Looking level, the view's lower edge 30 deg down, a table's top was out
  of it nearer than 1.4 m and a chair's seat nearer than 1.9.
- A table's top looked down on from 2 m spreads 2.2 grey levels a window:
  at a spread of 4.0 a quarter of it was kept, at 1.5 0.70, 99 % of those
  within 1.5 px of the renderer's.

## The map

0.1 m cells; a cell is a thing from 2 looks with 3 points in it between 0.08
and 2.3 m high, 0.7 to 3.5 m from the eye, hit more often than seen through
under its top. What she has not seen is open. After a look round standing
(25 looks):

| | |
| --- | --- |
| occupied cells | 725, 697 of them within 0.15 m of a wall or a thing |
| the chair | told where it stands, 0.6 m square, 1.02 m high |
| the table | told within 0.07 m, 1.3 x 0.7 m, 0.79 m high |
| the lamp | told where it stands, 1.77 m high |
| the bed | told within 0.05 m (0.00 m moved to (-0.9, 1.2)), 1.0 x 2.1 m, 0.49 m high |
| the walls | four walls mapped out to 3.5 m |
| lumps | each box from its cells alone, no kind a case |

- A ray freed every cell it crossed: over the table to the wall behind it
  each freed the table's, and the table was never told (87 cells in all). A
  ray frees a cell only passing under what stood highest in it.
- A lump's kind is its footprint and height first (a wall over 1.8 m, a lamp
  thin, a bed long and low, a table waist-high and over 0.65 m long, a chair
  short), its colour where those do not tell; the colour is the nearest
  kind's chromaticity, so the light on it does not matter. Lumps join
  within one look: a door's leaf is apart from the wall it stands in.
- Her way: A* over the cells grown by her 0.3 m radius, pruned to its
  corners, through what she has not seen as through the open, laid again
  every 2 s where what she has seen since bars it. Held to known-free
  cells, a target unseen had no way and every first word began with a look
  round, 14 s on 23 steps.

## The picture on the page

In the instruments' frame (`coaxial.graphics.inset`: rounded, its title in
the top edge, what she has told in the bottom one), W takes it away. A hull
is its ink along its edge and lightly over what it covers: filled at 0.45 the
wall's hull tinted the whole picture. A kind's label a plate at its hull's top
left, moved a row off another's: `thing` under `lamp` read `thlamp`. The four
rows of black over the walls are the room's: it has no ceiling. In a viewport
12 rows high the picture threw an IndexError once she had looked: nothing is
drawn where it has no room.

What bears her is what she has identified: a scan dropped every box within
0.05 m of her across, so the chair centred under her seat was out of her
sight and its rise could not stand on it. A thing with a verb within 1.2 m is
in her sight whichever way she faces (`scanner._in_cone`).

Her registry fed by her eyes (`sight.told`, `sprung.LOOKS` under
`physics.SIGHT`): a look round posed standing, the 65 things her map told made
96 slots - the chair 13, the lamp 10, walls 41 - where the stand-in's boxes
made 33 of 53. The chair, table and bed are each told within 0.10, 0.05 and
0.05 m of where they stand; the chair's 13 slots lie 0.10-2.15 m from it -
her map's own 'chair' lumps by their colour, not one chair told 13 times -
and the lamp's 9 to 5.4 m. The registry matched each axis over a thing's own
half size, a grown lump's centre shift its growth's half: 85 slots, the
chair's 13 still, and the bed's slot 0.38 m off where 0.05 (a growing lump
filtered in at the cell's precision averages its partial centres) - undone.
The slots are her map's lumps as told (`sight.classify`), not the gate:
`Map.lumps` joins cells four ways by their colour alone, so the chair and
the loft's stair beside it are one 1.6 x 1.2 m lump told 'table' by its 0.99 m,
and a table leg's single cell is a 'chair' by its 0.76 (65 things of 53
props, 22 of them 'thing' cells 0.1-0.5 m high along the walls). Joined by
their height within a cell too, the walls split into thin tall fragments
told 'lamp': 137 things, 211 slots - a cell's highest point is no surface
height for a standing thing. A lump's kind is a table (`sight.KINDS`).

## Her map in voxels

Her looks into voxels (`machine/eyes/voxels.py`, 0.1 m, the floor's 0.08 to
2.3 m) in her estimate's frame - her eyes' pose from her head as her
invariant EKF places it, not the renderer's -; a voxel a thing from 2
looks with 3 points in it, hit more often than seen through, the rays
stopping at the first voxel she holds occupied (grazing a wall she has seen
freed its voxels and the wall came in holes). Its kind the palette colour
whose body colour its chromaticity lies nearest with the light's white added
(the dichromatic line to grey): a lit timber wall read stone or switch
otherwise, 99 structures where 37; walls whole at 0.001-0.020 off their
colour. A top is an occupied voxel seen through above with nothing in the
layer above round it (a hole in a wall's face is none); a surface the
4-connected tops of one level, a structure the 26-connected rest joined
apart over and under a surface within its footprint (a back on its seat,
the legs under it: joined whole they were one 137-voxel thing with the
loft's edge), each with its own kind by its pooled colour, a part of
another colour its own thing past the three-sigma test on the structure's
pixels (the door in the glass gable, a chair's back against a wall; a
highlight none); a structure resting on nothing - above the floor's layer
under no surface - a stray. A thing's box: its footprint's principal axes
its turn, its extents along them a cell wide, its height its levels'; its
id the thing's it shares most voxels with at the look before; whether the
space over it is seen clear with it (`clear`: a seat or a bed is such a
surface, a structure's top unknown). Her registry (`machine.eyes.registry`)
matches a look's thing by that id before the Wasserstein gate and takes its
box as the look has it (a growing bed top filtered in by halves had split
into slots short of her buses' 1.8); its frustum test on a box's corners
(the nearest point alone left a table top's near edge under her view
and the table unseen); its boxes carry their slot and `clear`. What a thing
affords her is the text's (`machine/way/affordance.py`,
`evaluate-situational-affordances`): a seat a surface topping 0.28-0.92 m
with 0.075 m^2 clear above it in the pelvic and thoracic slabs, a bed a
surface under 0.75 m and 1.35 m long clear in the thoracic slab, a chair a
seat with a back - one of its kind reaching her lumbar's 0.248 m over the
seat (`figure`), its bottom no more than a cell over the seat's top (held
within a cell of it, a chair painted whole had none: its back and rear legs
one structure off the floor), no longer than its side a cell over; a wall
behind a bench is longer. Her things are her registry's boxes in both modes (`g['things']`:
without SIGHT the props, a look showing the whole room; with it what her
frustum has shown), her route planned round them (`route.blocked`, grown by
their blind spots' swell) and the furniture a verb goes to found among them
by what it affords (`furniture.GOES`), the errand's thing held by its slot
through its stunt (`machine.way.errand`, `maneuver.said`), its stage laid anew
where what she has seen since blocks it (`errand._cornered`), a look round
first where nothing of the kind is in sight (phase `look`).

Painted with MuJoCo's furniture (`world.props()`, each box's voxels filled
in its kind's colour, test_sight) her map at 0.05 m tells the chairs, the
table and the bed 0.000-0.035 m off, their floor hulls within 0.050-0.071
m (a voxel's diagonal 0.071), the bed moved 0.9 m and turned 0.5 rad as
well; at 0.1 neither chair (a back 0.9 m wide on a 0.7 m seat, the low
chair's none). Her eyes fill none finer than 0.1: at 0.05 a look round told
553-791 things to 85 (HITS 1-3; 490-559 at 640 x 480), her registry's 256
slots all live, the bed off her buses. A map's cost, test_sight's looks
(a voxel's kind over every voxel, empty ones too, took 0.78 s of 1.30 at
0.05):

| Voxel | Memory | `add` | `things` | Things |
| --- | --- | --- | --- | --- |
| 0.1 m | 2.7 MB | 78 ms | 28 ms | 96 |
| 0.05 m | 21 MB | 86 ms | 174 ms | 553 |
| 0.025 m | 168 MB | 171 ms | 6.2 s | 5 507 |

Her fake camera (`machine.eyes.camera`): her left eye's colour and the parallax from
the renderer's depth, her map in a process of its own, its things a look
later. A look round at 0.1 m: 64-67 ms a look to the stereo's 86-88, 43
things to 86, centres 0.06 m off the props at the median to 0.09, 25 of 29
props - the gable 3.41 m off lies at FAR_M, the stereo's depth error brought
it in -, the chair a seat with a back, the bed on her buses, her way to the
chair clear. On her loop 12.8 ms a look where she runs: 1.09 x flat out with
it, 1.08 without. The lodge's walls joined at its corners were one structure,
its box 1.8-3.1 m off each wall: a structure is told in pieces, halved across
its long axis till no box claims floor further than a voxel from its cells
(blocked cells 2 892 -> 2 242, the props' grown 2 371); a surface whole.
Her camera runs as `python -m machine.eyes.camera`: her page's worker is daemonic
and multiprocessing gives such no children. On her loop turning in place her
head's estimate (`inekf.head`) rose 5 mm/s - 0.058 m at 12 s, 0.153 at 38 -
and her map smeared: the chair told with no back, the bed in three; laid from
the world's head she found both. Her page shows her eyes alone: its panel her
camera's picture (`physics.CAMERA`, her registry the stand-in's still) and what
her eyes' view holds (`scanner.seen`, 37.6 deg either side, FAR_M), F's offers
what it has shown - held to 75 deg and 1.2 m wherever they named things no look
had shown. Her worker names her things (`sprung.named`): a scan 0.12 ms on her
59, 12 ms named on the page each frame. A bed is one of its kind she lies on: by
lying alone the deck, the terrace's steps and the bench were beds.

A look round posed standing (`tools/sim/eyes.py`, 25 looks, 70 ms each
here): every one of the 29 props her eyes could take in told, its centre
0.09 m off at the median, its sizes 0.11, the lodge's axis-aligned turns 0
deg (the bed moved and turned 0.5 rad told within 0.15 rad, test_sight); 87
things, 75 slots her buses take - the chair's seat a seat with a back, the
table a seat, the bed a bed, the loft's edge a seat, the lamp a lamp, the
door a door -, 46 things no prop's (one-voxel pieces at the walls,
3 looks of 25 each). Kinds right for every prop kind but the switch (white:
no hue, its wall's). On her loop a turn in place (24 looks, 12 s): 185
things - her head's estimate rises 8 mm/s (her law's) and 5 mm/s (the
EKF's) stepping in place, the map smearing a voxel in 12-20 s, surfaces
doubled -; told to sit she finds the chair's seat (the seat with a back
nearest her) and walks toward it, but her stage is never free: her chair's
things grown 0.4 m meet the table's, `route.staged` finds no free run along
the dock's normal and returns its most, inside them, `_cornered` lays the
same, and she walks between the nearest free corners 22 s (`stall_probe`).
Told with nothing of the kind among her things she looks first
(`sprung.looked`): without SIGHT the stand-in sits as before.
