# Findings: pages

The terminal's pages and what they draw. The host's other findings:
[host](host.md); the board's: [FINDINGS](../FINDINGS.md).

- A `Feed` slept its period after the read: a 20 ms read at 50 ms fed 14
  readings a second to a 20 fps page. Start to start now.
- HUMANOID rendered under `--keys` (a key a frame, `tools/render/page.py`):
  O turns the view, X puts her at the other side, Shift+W runs her at 0.90
  m/s, W+A walks her an arc the chase follows, C has her down with `[F]
  #201 stand` offered, the chair scene offers `[F] sit chair` at 0.8 m. Her
  process ran at 0.2-0.3 of real time on four cores, 8-10 frames a second.
  A frame at 15 a second showed the same state again 65 times in 235, the
  rest 0.036 s of her time apart with 0.022 of spread, her process at 0.7 of
  real time in slices of 0.05 s (`terminal/views/playback.py`).
- HUMANOID's balloons (`gym.offered`, `coaxial.graphics.balloons`): from her
  lay in the lodge facing +z the door 3.6 m at 12 deg, the bed 2.5 m at 45,
  the chair 1.7 m at 62 - F1 go out, F2 lie down, F3 sit -, at 150 x 44 the
  bed behind the docked eyes and its balloon at their edge. Numbered nearest
  first, F alone sent her from (0.12, 0.28) to the chair 56 deg aside, not
  the door ahead, and she fell at 5.19 s (scenarios item 10): numbered by
  her turn to them, F alone keeps what she faces. Lying on the bed her one
  offer read F where three had read F1-F3, and backing to it the door at her
  sight's edge came and went four times in 1.5 s: one alone is F1 too, a
  thing once seen kept till her scene or her epoch is laid anew.
- The crafts' pages flew on a `Feed` thread in the page's process and drew
  the newest of its 20 ms steps: on a 200 x 56 truecolor terminal 13.4-18.7
  frames a second, gaps p95 80-115 ms, the longest 484-639, the flight's
  Python sharing the GIL with a frame's 3D (16-33 ms) and rich's update
  (21-28 ms). Run apart as hers is (`machine.process.apart`, `quad/flying.py`) and
  played back no further than 0.15 s behind what it said, a frame 0.1 s of
  its time at the most: 18.7-19.3 frames a second, p95 66-77 ms, the longest
  75-89, its time on at the wall's (1.00). HUMANOID on the same loop: 17.2.
- Played back at a frame's start, the flight reached the terminal 35-80 ms
  later, rich's update 24 ms and 51 at p95 - a collection of the whole heap
  in it (28 at p95 with none): on screen the helicopter flew 0.67-1.48 of
  its speed frame to frame (p5-p95), "it jerks". Each frame now leaves on a
  beat (`screen.Beat`: the last one's write and the 90th percentile of the
  work since), the heap before the page frozen, the flight shown at that
  instant (`screen.drawn_at`): 0.98-1.02, the gap frame to frame 0.3 ms and
  1.5 at p95, a frame 61 ms where 55.
- A helicopter frame was 52 ms: its 3D 23 (the rotors' rings 7 of it, 192
  lines of a few numpy calls each), rich 20 (the CRT 18: every cell of the
  screen a list, twice, for the few dozen it puts), compose 4. The chrome
  keeps rich's segments and splits them where it puts (`chrome._Row`, the
  same cells on eight pages at seven instants), a ring's chords one call
  (`callouts.lines`, `line`'s dots): 35 ms, frames to 30 a second
  (`FPS_CAP`; a board read 20, `FEED_HZ`) - the helicopter 28.7, the quad
  27.8 where 16.4 and 13.8 -, each one a synchronized update (DEC 2026).
  On its course lap at 12.6 m/s the craft on screen is off smooth motion
  0.04 dots (0.38 at p95), its path in the world 9 mm at p95; the chase
  holds it 15 dots off the middle, 43 at p95.
- HUMANOID drew a frame every 68 ms, her motion on screen 0.59-1.07 of its
  speed: her playback eased to her process's ratio and caught up after it.
  Held to what the page shows (`apart.LEAD_S`) and played back steady
  (`Playback.steady`, as the flights are): 0.92-1.03, her wear window's
  passes a slowing, not a burst. Her frame was 57 ms: her room's edges 413
  `line` calls (13 ms; `props.lines` an ink a call), the braille's runs a
  scan of every cell, rich 20. Her 3D drawn a frame ahead in a process of its
  own (`terminal.ui.ahead`, the flights' too), the viewport two passes
  flatter (`Marquee` centres, `Chrome` takes the rate): a frame every 36.6 ms
  (27.3 a second), what is left rich's layout of her instruments (12 ms);
  the flights 33.8 (29.6), on the cap.
- HUMANOID's R crashed the page at once with no traceback
  kept; headless it took R and saved 97 rows then, and R starting, three
  states and R saving is test_views_humanoid.py's. The
  suspect stands: a page started before `buses.FIELDS` changed (play and
  flex then; delta, delta_d and ahead) reads a block of
  another layout - a page is restarted after a pull.
- THERMAL OBSERVER said `AFE off` with AFE_ON high: no reading yet, the first
  sample is 30 s after opening. It says which now.
- THERMAL OBSERVER and the side columns, a review's 29 findings against the
  refit:
  + `--switch` armed a real board with the break bypassed; now AFE_ON off,
    the latch cleared, the break in circuit, AFE_ON restored as found.
  + The map's centre patch was stripped, its middle 9.6 K off.
  + A hud counted one row and the column never paged: the
    page's 78 lines cut at 42; SESSION's THERMAL 17 of its 20 nodes.
  + TUBES 37 cells in 36, the NTC's tube an ellipsis; ten rows on three pages
    wider than their boxes. The boxes: `terminal/views/thermal/boxes.py`.
  + The stand-in's budget never had seconds to its limit; its load cycle put
    45 A through the hot swap.
  + The trip's cap was asked equal to 1e-6 of a margin trimmed in 1e-3.
  + SENSE's `err` read +9 K on a model 0.1 K out.
  + The real board's `--switch` on the bench at 23.9 V: armed with the break
    in circuit, three legs at 50 % for 5 s (the observer's drivers 31 -> 41 C
    blind, the thermistor 0.9 K up after), disarmed, AFE_ON off as found.
- QUAD's pass on four stand-ins was 5.1 ms of 20, 16.2-17.0 frames a second
  on a lap: a slice's envelope weighed the identification's doubt a node, 42
  times (0.6 ms a pass), and the frame's forces were crossed a rotor a step in
  the world's frame, 40 crosses (1.0 ms). One margin a walk of the nodes and
  the four's torque once a pass about the frame's own axes (the frame the
  same to 1e-13 over 8 s of tumbling): 3.4 ms, 17.9-18.8 frames, 19.2 with
  nothing flown.
- QUAD's two plots inked and marked alike: a plot's left curve and its scale
  one ink, its right another, the bus a mark a row as the height (orange and
  red had been the bus's and the temperature's).
- The board's raster on a GTX 1080 Ti (wgpu, Vulkan): 392x224 100 ms -> 2.1
  ms; the menu's turntable 64 -> 13 ms at 52x18. BOARD ATTITUDE 66 -> 64 ms
  a frame at 200x60: its ground (32 ms) and paint (15) are the frame now.
- BOARD ATTITUDE, 150x44 on native://: 10.4 -> 29.6 fps turning (ANGLE 29.7).
  Its frame was Python cell by cell - the face's paint 30 ms, the floor 16,
  the shadow map 4 a new bin; on arrays now, bit for bit (test_paint,
  test_ground, 157 frames A/B), drawn in a process of its own a frame ahead
  (`ahead`, `easel`), the page's own work 21 ms, rich 14-18 of it. Its feed
  asked a frame: 19 -> 27 new poses a second of 29.6 frames, the rest a read
  past a frame (25 ms beside the page, 14 alone). Its start: one preload file
  for the machine's trees left each other's pages cold (decimates 1.5 s,
  outline 1.0), a file a stamp now; the drawing's process 1.9 s cold, 1.5 of
  it wgpu's C types cffi parses in every process; a daemon, it decimates a
  cold preload itself (a pool stopped the bench's page). On the bench from the
  front page 2.3 -> 0.9 s: the board's open 0.49 (the AFE's settle 0.3), its
  parts 0.10 (the map 0.25), the rotation vector 0.12, the first frame 0.15,
  drawn here on the warm card till the process is up, 2.1 s on, the exposure
  and vote carried; by name, the card cold, 2.5 s. Its drawing kept from the
  terminal's start (`AHEAD`), a named page's before the loader's imports:
  by name 1.9 s, from the front page 0.9 s its own process's first frame.
  The crew's decimates had
  dropped the bore's keep: a missing preload drew the two holes.
- The front page's model was drawn at inner height - 2 inside a 1-column
  padding: a blank row top and bottom, a column each wall. Now the box's
  full inside (checked at 90x28, 120x36, 200x60).
- The front page's stand shows the quad after the gynoid, 9 s alone in a
  hover and once over its nose: 10 ms a frame on the card, 69 on the CPU,
  where the stand keeps to the board (`terminal/stand.py`); QUAD's row had no
  kana.
- Rotor observer at a 200x60 terminal (can 95 dots, tuned at 21): the ring
  stroke grew to 3.0 dots half-width, pulled teeth floated loose. Stroke
  capped at 1.0 (0.8 broke into dots), the undriven tooth length drawn as
  track, kept out of any cell an area holds.
- One DC bus connector's screw hole drew as a box, then vanished into the
  connector (724f950 absorbs nested blocks): the circle test centred on the
  centroid and 14 unevenly spaced points read dev 0.047 (limit 0.03).
  Least-squares centre: a drum, like the other four; no other primitive moved.
- BOARD ATTITUDE face down drew the top's parts on the underside. The
  outline's grace adds the cell's depth span, and a tilted face spans
  0.036-0.08 a cell against a 0.032 slab: 533 of 535 drawn dots sat behind
  it. An edge on the slab's far face gets the fixed grace only; the face art
  (the top's layout) is not read from behind (rasters face down,
  up, tilted).
- A shaded cell keeps 0.4 of its dots at the least (`lit.braille`, 0.40 + 0.60
  of its light against the blue noise): on 0.04 + 0.96 of its light to the
  1.5 the crafts' mid-grey shells and the gynoid's clothes read as clouds of
  points beside the glTF and FBX models they were judged against. Her jeans
  (70, 98, 150), 0.062-0.068 m a thigh where 0.072-0.088, 0.052 m a hem where
  0.074; her shoulders' rings 0.118 m at 0.344 m where 0.14.
- The helicopter after the T-REX 700 beside the repo: its canopy a 0.57 m
  teardrop over the frames, red over black; a loft's fourth number drops a
  ring, and the lime canopy had sat under the frames, unseen from the side.
  The ornithopter's cabin 0.18 x 0.15 m, its wings their 0.085 m chord where
  0.032 was drawn.
- The tty's face (VS Code 1.141's xterm 6.1 in Edge 154, WebGL,
  125 %): a cell is the face's W wide, its braille and boxes drawn to fill
  it.

| Face | Cell, device px | Note |
| --- | --- | --- |
| Consolas 14 | 9 x 22, 2.44 | 205 x 43 cells |
| Eurostile 14 | 17 x 19 | text strewn |
| Eurostile 12 px, lines 1.3, letters -3 | 11 x 22, 2:1 | read as Extended |
| Eurostile, letters -5 (VS Code's least), lines 1.2 | 9 x 20 | 205 x 47 cells |

  Latin glyphs past their cell overlap it, never rescaled. Too wide and
  uneven either way: back on VS Code's own. DTC's Eurostile Extended reaches
  Chromium as Times New Roman: a cmap subtable truncated.

- The terminal's keys from one table (`terminal/ui/bindings.py`): HUMANOID's
  bound in five places, each page its own way; where they met, the arrows
  lifted her view and paged the column, Esc in her prompt left the page, F's
  choice outlived the keys after it, the rotor's keys took lower case alone,
  SESSION's + - were unbound. At 150 x 44 the 13 pages open with the column
  a rail - its arrow breathing with the screen's sweep, a key's ink to (28,
  191, 191) and back over 5 s -, the front page's panels up (no Tab);
  HUMANOID's pairs ran 194 cells, the terminal's at the bar's right end now,
  the page's cut before them. Esc over HUMANOID showed a small second front
  page; now the front page, the band its page's name alone. Folded, BOARD
  ATTITUDE drew the open column's 106, chat, render a 24 column; now
  `taskbar.room`, 141.
