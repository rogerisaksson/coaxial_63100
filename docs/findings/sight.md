# Findings: her eyes

A stereo pair on her head (`machine/sight.py`, `physics.SIGHT`), rendered from
the world by MuJoCo, the disparity by block matching in numpy, the depth from
it, the points into a map she keeps of the room ([room](room.md)), its things
told by shape and colour, her errands' ways planned round them, and the page's
picture of her view (`coaxial/graphics/inset.py`); test_sight.py holds it.
Nothing of hers reads the world's geometry.

## The eyes

| | |
| --- | --- |
| cameras | two on the head, 0.1 m apart, 60 deg high, 320 x 240 |
| a look | 95-120 ms: both renders 2 ms, block matching 48 disparities over 9 x 9 windows the rest |
| kept | 8 % of the pixels (textured, unique); the room's things speckled for it |
| depth | within 1.7 % of the renderer's at the median, 7.9 % at the ninth decile, 1.0 % at 1-2 m |

- A checker texture repeats: block matching matched a square to the next and
  the map filled with phantoms (1 747 cells); speckles of a darker shade on a
  flat colour match once.
- The model's `statistic extent` was 10 km from the floor's parked patches:
  the cameras clip nearer than a hundredth of it and saw nothing; set to 8 m.
- The model has no light: four on the ceiling for her eyes.

## The map

0.1 m cells; a cell is a thing from 3 points between 0.08 and 2.3 m high,
within 3.5 m of the eye, hit more often than seen through; free from 2 looks
through it; unknown is not free. After a look round standing (0.5 rad/s, a look
every 0.5 s, 29 looks):

| | |
| --- | --- |
| occupied cells | 299, 277 of them within 0.15 m of a wall or a thing, 3 beyond 0.4 m |
| the chair | told within 0.16 m, 1.03 m high |
| the table | told within 0.05 m, 0.82 m high |
| the door | its leaf seen closed in the doorway, told by its colour; passable to the planner |
| lumps | 25, 11 of them over 0.04 m2 |

- A lump's kind is its footprint and height first (a wall over 1.8 m, a lamp
  thin, a bed long and low, a table waist-high and long, a chair short), its
  colour where those do not tell; the colour is the nearest kind's
  chromaticity, so the light on it does not matter.
- Her way: A* over the cells grown by her 0.3 m radius, through known-free
  cells alone, pruned to its corners; a goal unseen is no goal (planned toward
  the target beyond the door she was sent to the nearest cell she knew free
  and walked into the wall beside the door).

## The errands on her eyes

Asked a word on a fresh map she looks round first (12.6 s standing), plans to
the approach point, and plans again every 2 s walking; the last stretch is
straight. Out through the door: from her stand she crosses the
door line 0.30 m from the nearer post; E pressed as she walks, 0.41 m (before
her eyes, the walk's restart fell short of the door); aligned within 3.4 deg
before the last stretch (6 deg crossed it 0.19 m from the post and she fell).

## The picture on the page

In the instruments' frame (`coaxial.graphics.inset`: rounded, its title in
the top edge, what she has told in the bottom one), W takes it away. A hull
is its ink along its edge and lightly over what it covers: filled at 0.45 the
wall's hull tinted the whole picture. A kind's label a plate at its hull's top
left, moved a row off another's: `thing` under `lamp` read `thlamp`. The four
rows of black over the walls are the room's: it has no ceiling. In a viewport
12 rows high the picture threw an IndexError once she had looked: nothing is
drawn where it has no room.
