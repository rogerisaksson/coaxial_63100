# Findings: the lodge

The robot's place: the glazed mountain lodge round the robot
(`machine.world.room`), its scene (`floor.scenario`, 'lodge'), what the page
draws of it and the robot's eyes tell; the scenes on its stairs in
[scenarios](scenarios.md).

- The lodge (`machine.world.room`, scene 'lodge'; asked, from pictures of dark
  timber cabins on snow): timber walls, the gable glazed between mullions with
  the door in it, two roof slabs under snow nothing collides with, the deck past
  the gable with a glass rail and a gap on the door's line for the wooden stair
  down the slope (`floor.scenario`: stairs_down there, snow and a spruce below,
  the plane 0.75 m down), a stone terrace a step up at its left end, a loft
  stair along the right wall; the robot is laid on the door's line
  (`floor.lay`). 132 geoms, MuJoCo's step 96 -> 106 us a pass; the robot's loop
  533 us, 1.88 of real time. On the page's PNG the gable, the roof and the rail
  read, the glass by its mullions alone; the robot's eyes tell the roof a thing.
  The descent (`lodge_down`) fell at 11.2 s at the stair's foot as stairs_down
  does at 3.4 (item 3): defined, not held.
