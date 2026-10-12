# Findings: the envelope in flight

The quad's boards' thermal envelopes under its flights: a climb, a hover, what
a board may carry (`thermal_run.c`, `terminal.views.quad.flight`); what its
rotors pull (`machine.pull`). The quad's own: [quad](quad.md); the board's:
[FINDINGS](../FINDINGS.md).

## A climb

Full tilt from a hover on four stand-in boards, still air, the heat's clock
ten times the flight's:

| The four's pull | Their amps, the legs at 0.9 | Throttled | Tilt at the most | Apex, m |
| --- | --- | --- | --- | --- |
| a share by rule on the envelopes' room, read 10 times a second | 33, 8, 32, 18 | each on its own, to 0.10-0.42 of its clamp | 83 deg | 21.6 |
| the same, each rotor's own cap in the allocation | 44, 33, 44, 44 | each on its own | 33 deg | 29.1 |
| each board its hold, its legs' mean squares counted | 50.8-51.2, 43.7-43.9 at its end | none | 9 deg | 37.3 |

- A leg's node is 0.12 J/K behind 28 K/W: 57 A had the legs at 0.90 of
  their envelope 0.2 s on. Each board throttled on its own within one read
  and the frame went over.
- A board says what it may carry (thermal op 4, `hold_amps`, MINOR 32): the
  most current with every node its clamp reaches keeping 0.5 s to its
  throttle's point - five of its slices, a fifth of its way a slice - and
  holds its clamp to it. From a hover's 87 C all of the clamp's 90 A; asked
  its top, 60 A 0.16 s on and 66 to hold, 52 at 0.24 s, then 48 falling to
  44 as the patches warm, the legs at 0.89-0.90, nothing throttled. On the
  throttle's own 2 s it had 47 A of 60 from the hover.
- The stand-in gave its heat one sample of the phases a slice: at 1.4 kHz a
  leg stood 30 K over its neighbour by where the samples fell (133-166 C),
  and the board it was on throttled first. Counted a period as the board's
  sync counts them (`world_drive_window`): 157, 152, 156 C.
- The flight's share is the least of the four's holds, of its clamp, read a
  step; a rotor's cap what its board's current reaches from the speed it
  has. Counted from its speed and its loop's slew, a rotor behind its ask
  had its next cut to that and the laps were 0.3 s the longer.
- A rotor's speed for a current without its own friction (0.49 N m of 2.04
  at 612 rad/s): asked 659 where it held 612, the tilt had no rotor to
  answer with and stood 19 deg off late in the climb.

## The race on it

- The boards keep 88-100 % of their pull at the least where 34-60 %: SOA
  0.76-0.86. The plan at 0.399 of that pull came into gate 7 1.1 m high off
  the dive in two winds of three - the ideal rotors' did in one before -,
  its discs 100-109 deg over where 121-131 in still air.
- `course.PULL` gridded alone: 0.34 and 0.36 whole, 32.41 and 29.54; 0.37
  29.21, 34 flights of 34 whole in eleven winds; 0.38 one struck. Laps
  14.2-14.4 and 14.5-15.2 s where 13.8-14.3 and 14.3-15.3, a tip 0.16-0.17 m
  inside its margin, the ideal rotors' whole in every wind.

## A hover

- A board in a hover (62.7 V, 8.1 A, 2 014 rpm) makes 9.4 W: its legs'
  dry switching 6.45 (2.15 W a leg at 63 V and 50 kHz), their conduction
  1.0, the shunts 0.34, the rest 1.55. Dry at 63 V it makes 8.0: the hover's
  amps are 1.4 W of it.
- The wash takes the laminate under the legs from 91 C in still air to 60,
  its middle from 52 to 36; the legs stand 30 K over their patches in
  either, 87-90 C: 0.44 of their envelope before any pull is asked.
- At 25 kHz the bench's dry rise halves (17.7 and 9.2 K at 60.8 V): TODO.

## What the rotors pull

`machine.pull`: a rotor's top from its motor, its propeller and its link,
against the law on a stand-in (`test_quad_pull.py`):

| Link, clamp | The law, rad/s on A | The space, rad/s |
| --- | --- | --- |
| 63 V, 60 A | 703.3 on 60.0 | 703.3, the clamp's end |
| 63 V, 90 A | 721.2 on 62.8 | 721.2, the link's |
| 57.3 V, 60 A | 687.8 on 57.7 | 687.8 |
| 48 V, 90 A | 626.3 on 48.8 | 626.3 |

- The four on one pack end on its link: 690.6 rad/s full, 58.1 A each, 131 A
  from the pack, its bus at 57.7 V; 644.3 at its reserve. Four boards came
  to 682.7 where the space has 682.5, 0.939 of the charge left. WEP's clamp
  adds no top there: its 90 A are reach, 5 507 rad/s^2 at 500 rad/s where
  2 609.
- A rotor's speed gains what its current leaves over its propeller: 3 596
  rad/s^2 at 401 rad/s where the space has 3 607; on a 44 V link, 57 A of
  the clamp left, 2 070 where 2 106, the estimate's frame a degree behind.
- Stepped to 90 A from its idle a drive trips on overcurrent; asked up over
  0.5 s it does not.
- The law's own v . i is under its stage's watts - the copper's and the
  shaft's: 1 % at a hover, 9.5 at 500 rad/s, 21 at 680. Its duties wait a
  period and its vector meets the rotor that much turned, 10.8 degrees at
  9 400 rad/s electrical. Over two laps 17.9 kJ where the currents took
  43.8 and the propellers alone 33.9: the pack was drawn 0.41 of what its
  stages took. Counted from the currents (`pull.watts`) a course's flight
  takes 20 % of its charge where 8, 67 A at the most, the bus to 57.5 V;
  the routine 12 %, full tilt 129 A and 56.3 V.
- A lap turns its rotors 517 rad/s at the most, their currents at the clamp
  in their spools: it is flown on their reach, not their top. Its plan's
  pull is the clamp's, 703.3 rad/s on any charge, and a row's reach the
  link's. Planned on the pack's top it slowed as the pack emptied: 14.6 and
  15.1 s, 15.2 and 15.5 from 60 %.
- The race on it, 12 flights: 28.94 where 29.09, a gate's frame 0.05-0.18 m
  inside its margin. In fourteen winds 34 of 34 whole, the boards' laps
  14.3 and 14.5-14.6 s where 14.5-14.8, their pull 80-85 % at the least
  where 73-84.
