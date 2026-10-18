# Findings: her eyes

A stereo pair on her head (`machine/sight.py`, `physics.SIGHT`), rendered from
the world by MuJoCo, the disparity by block matching in numpy, the depth from
it, the points into a map she keeps of the room (`machine/room.py`), its things
told by shape and colour, a way planned round them, and the page's
picture of her view (`coaxial/graphics/inset.py`); test_sight.py holds it.
Nothing of hers reads the world's geometry.

## The eyes

| | |
| --- | --- |
| cameras | two on the head, 0.1 m apart, 60 deg high, 320 x 240, looking 15 deg down |
| a look | 95-120 ms: both renders 2 ms, block matching 48 disparities over 9 x 9 windows the rest |
| kept | 51 % of the pixels: textured (a window's spread over 1.5), unique (the next best 1.15 times the best, off its own neighbours) and picked back by the right eye within a pixel |
| depth | within 1.3 % of the renderer's at the median, 2.9 % at the ninth decile |

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
