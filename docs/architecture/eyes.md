# Architecture: Eyes and way

The robot's eyes and the map it keeps of the room, and its way among its
things: what it sees, what it takes each thing for, where it goes and how it
docks. The rest of the robot round its loop: [robotics](robotics.md).
Findings: [sight](../findings/sight.md), [errands](../findings/errands.md),
[spots](../findings/spots.md).

## The eyes

| Module | Is |
| --- | --- |
| eyes/sight | the robot's eyes: a stereo pair on the robot's head, the disparity by semi-global matching, the map the robot keeps, its things, the robot's way round them |
| eyes/gimbal | the robot's eyes' gimbals as built and as calibration told them: each camera's turn and place; `gaze` the servos turning them, `squint` the right eye's turn told from its pictures |
| eyes/gaze | the robot's eyes' gaze: three servos turn its eyes by pushrods, across, down and against each other |
| eyes/squint | the robot's right eye's turn past its calibration's model of it, told look by look from its pictures |
| eyes/epipolar | the robot's eyes' pictures laid on its epipolar planes however its eyes turn: a row a plane through both |
| eyes/stereo | the stereo disparity by semi-global matching on the card (cupy) or the CPU, integers alike, over DISPARITIES |
| eyes/camera | the robot's fake camera: the robot's eye's frames shot where the world runs, the renderer's parallax; the robot's map kept in a process of its own, its things a look later |
| eyes/vision | the robot's looks beside its law: a look's registry pass on a process of its own, joined before it is read |
| eyes/instances | the robot's looks' objects: a segmenter's masks on the card, each boxed from its stereo's points under it |
| eyes/classes | what the robot's eyes take a thing for, the text's semantic classifier: a look that tells it alone its class - a door, a bed, a lamp, a switch -, a material's thing by its box against the robot's figure and the text's slabs and kernels - by its top a floor, a stair, a step, a platform, a sofa, a chair, a table, a bench, a stool, a cabinet, a shelf; by its faces a window and the posts framing it, a railing, a ceiling, a wall, a post -, a chair's back, a leg or a face under a top the top's; a door's or a bed's look where its box bears it out; a building its shell - walls, windows and doors joined or in one plane - with an opening in it, a cabin of one storey else a house, outside it a veranda or a terrace by it and the ground; many to one, what is of one thing one - a window's panes and posts, a stair's treads, a wall either side of its openings -, two chairs side by side two; the robot's registry's slots by it, the robot's panel a thing once, and the robot's picture: the robot's map's things, the floor and the ground under it and what lies past its reach lumped by look, each thing outlined once, a building seen from outside outlined |
| eyes/stacked | a look's things as arrays, restacked where they changed; their classes kept while their values hold |
| eyes/registry | the robot's map of the room's things: the kernel's 256 slots filled by the robot's looks (the robot's eyes' or their stand-in), matched in the Wasserstein metric over a thing's own size, filtered in, weighed by the text's log-odds in and out of the robot's eyes' frustum, blind spots kept 45 s and swollen, movables diffused, a mover tracked at its velocity; the robot's buses' boxes the slots at its salience |
| eyes/voxels | the robot's map of the room in voxels: its surfaces and structures told as boxes, each with its turn |
| eyes/ground | the robot's floor as its looks lay it: a top-down mosaic of its grey and height, each look laid onto it |
| eyes/grids | the robot's grids' cells grown and joined, its points' hulls: of numpy alone, nothing of its own read |

## The way

| Module | Is |
| --- | --- |
| way/affordance | what the room's things afford the robot (affordance_funnel_calculus.lisp): a seat, a bed, a chair's back |
| way/furniture | what the robot's eyes tell the robot of a thing a verb goes to: its top, its near edge, its centre |
| way/obstacles | each floor point's cost among the robot's things, whatever they are (smooth_interactive_wayfinder.lisp) |
| way/route | the robot's way round its things to a dock: A* over their cells' costs (`obstacles`), corners on the widest arc the free cells and its legs give, walked at its turn times the radius, braked into it; docks `dock`'s |
| way/wayfinder | the kernel's wayfinder's gauge: the robot's trunk twisted through the gap its route meets 0.6 s on among what its high band meets |
| way/dock | where the robot docks at a thing: the face, its place along it, where it stands to turn onto it |
| way/peek | the robot's docks foreseen: the faces its wayfinder reaches on each thing, before it is told to go |
| way/planner | the robot's wayfinder apart: its dock and way planned in a process of their own, as its camera's map is |
| way/errand | the robot's errands: told a verb, the robot walks to its thing among its things, docks and plays the stunt |
| way/words | the robot's errand's words: the thing it goes to, what its planner is asked, what each phase tells it |
| way/steer | the errand's words to the robot: a word told once, a turn in place toward its way while off it |
| way/cocoon | the robot's braking cocoon on its keys: it stops short of a thing as a let-go key stops it |
| way/tour | the scenes one after another, commanded as an agent commands them |

## Suites

| Suite | Holds |
| --- | --- |
| `test_sight.py` | a look round, posed standing, into the robot's map, its occupied cells against the room's walls and things, the chair, the table and the bed told where they stand by what they afford the robot, the bed's turn with it; a way to the chair planned clear; the page's picture of the robot's view; its way round the lodge's furniture walked through its corners |
| `test_gaze.py` | the robot's eyes turned and placed as built and as its calibration told them; the stereo's depth held to the renderer's own looking ahead and turned across and down; the squint told from its pictures look by look; a way's places scanned in turn, a place fixed wherever it stands; its walk to its dock led by its way |
| `test_camera.py` | a look round on the robot's fake camera's frames, the furniture told where it stands; the robot's map apart telling what the robot's map here does; the robot's loop under SIGHT on it; what the robot's eyes take things for: a house's things by their boxes and what is of one thing, the lodge as a house, a cabin from outside, the lodge from its deck, each thing once, the robot's registry's slots keeping each one's class, the robot's picture past the robot's map |
| `test_registry.py` | the robot's map's look in C beside its numpy twin: each thing its own slot, the loop where two meet one, a full registry's least salient taken, things moving, unseen, pruned; the lodge's own looks walking out of its door, every slot's every field and class bit for bit; a look's time |
| `test_wayfinder.py` | the robot's wayfinder and its map: a gap's width across its way, what its looks told it |
| `test_docks.py` | the robot's docks foreseen on its page (`machine.way.peek`): the lodge bed's long side and its foot |
| `test_errands.py` | errands: a stop settles, F heard, a new scene walked |
