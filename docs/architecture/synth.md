# Synth: her loop as a synthesizer

The latent loop is a synthesizer feeding a control system: a sequencer lays
notes, an oscillator turns at the tempo, envelopes blend each voice to its
note, gates open a limb's swing, sidechains move a voice's depth or the tempo
from what she senses; the control system - the impedance, the whole-body
step, the drives - renders what comes out. Each term names one thing in the
code, no term a second thing. The parts themselves: [planners](planners.md).

## Signal chain

| stage | the loop's | where |
| --- | --- | --- |
| sequencer | rows laid at their times, one a tick | `latent.table`, `latent.step` |
| note | a row: its voices' targets, its attack, its track | `ortho.ROW`, `latent.VALS`, `SAID` |
| clip | a take's beats as notes, legato | `takes.beats`, `takes.wire` |
| patch | a text: its phases as notes, its oscillator, its springs | the lisp, `ortho.compiled` |
| program change | a text said in place of what was to come | `worker.heard_say`, `se2.said` |
| oscillator | one angle at the tempo, a voice's depth and phase at it | `latent.THETA`, `HZ`, `GAIN`, `AMP`, `PHASE` |
| envelope | a voice's blend from where it is to its note | `latent.FROM`, `TO`, `T0`, `BLEND`, `CONTIG` |
| generator out | each voice's reference, rate and acceleration, its spring and damper, the phases | `latent.OUT`: `REF`, `RATE`, `ACC`, `KS`, `DS`, `PSI`, `SWAY`, `TURNS` |
| gate | each limb's swing window and its share done | `stride.beat`: `on`, `off`, `u` |
| sidechain | a parameter moved by a sensed signal, weighted 0 to 1 | the table below |
| modulation | the patch's knobs moved by named amounts | `gait.MODIFIERS`, `gait.modified` |
| control system | the ask on her body, the step, the drives | `impedance.ask`, `wrench.step`, `edge` |

## Terms

| term | her loop's | the number |
| --- | --- | --- |
| voice | a mode of her latent vector | 11, `ortho.MODES` |
| pitch | a voice's target | `Z`, the mode's unit (`modal.AXIAL_M` 0.115 m, `PITCH_RAD` 0.5) |
| velocity, accent | a voice's depth on the oscillator | `AMP` |
| tempo | the oscillator's frequency, a stride a turn | `HZ`: the walk 0.9, a jog take 1.27, a run 1.65 |
| beat | a footfall, the oscillator half a turn on | `stride.beat`'s `phi` through `PHI_W` |
| bar | a stride, the oscillator a turn | 2 pi of `THETA` |
| clock | the oscillator's angle, turned | `THETA`, `TURNS` rad/s |
| attack | a note's blend time | `blend_duration`: the walk-on 2000 ms, a beat its gap |
| sustain | the note held till the next | `TO` |
| decay | the gait let down to the stand | `gait_brake` 1800 ms, `sprung.BRAKE` 500 ms |
| release | the row back to rest | `ortho.REST`, `REST_MS` 200 |
| legato, glide | contiguous notes drawn as one curve | `ortho.CURVE`, `latent.CONTIG` (Catmull-Rom) |
| rest | a stand, the oscillator halted | `cpg_override (halt_oscillators)`, `GAIN` 0 |
| gate length | the swing's share of a turn | `stride.SWING` 0.4, `PHI_W` |
| swing (the feel) | the sway a quarter turn ahead of the drive | `phase_offset` 108 deg |
| hold | a note waiting on the soles or the floor | `waits.held_rows`, `risen_rows`, `landed_rows` |
| quantize | a landing drawn to a tread's grid | `stride.footholds` |
| transpose | a floor's rise read into her height | `stride.floor_mean`, `impedance._landings` |
| flam | two notes in one tick: they add | `latent.step`, `hit @ VALS`: Hz 1.78 = 0.86 + 0.92 |
| track | a row's sequence id | 200-205 the texts, 300 a take, 301 a modified row |
| preset | a gait take as amounts on the patch | `takes.amounts`, `takes.pattern` |
| mod wheel | a word of her movement language as an amount | roll +0.86 a catwalk (`tools.sim.normal`) |
| polyphony | a note says some voices, leaves the rest | `SAID`, NaN unsaid |

## Sidechains

| from | to | where |
| --- | --- | --- |
| the twist told: speed, strafe, yaw | the drive's and arms' depth, the tempo, the lateral's target | `se2._modulate_cpg` |
| a shove felt, the twist blocked | the sway and the drive ducked to the brake's, the stand's targets | `touch.step` |
| her capture point past the soles | the swing's rate up | `stride.hurried` |
| her soles' grip, told or felt | the tempo capped, the landings' reach | `situation`, `grip` |
| the floor told | the swing's clearance, its tread, her height's rise | `impedance._landings` |
| her need to step | the stride on at any drive | `stride.beat` |
| a seat bearing her | the notes that raise her held | `waits.risen_rows` |
| nothing bearing her | the notes that lower her held | `waits.landed_rows` |
| her rhythm died down | a due note restarts the clock | `waits.woken_rows` |

## No voice yet

- Flight: no note lifts both soles; a jump is the leap's throw (`leap`), not
  the oscillator's. A jog or a run on the oscillator is her walk at the
  take's tempo and drive, 1.3-2.0 m/s ([sequencer](../findings/sequencer.md)).
- Both arms overhead, both heels up: no voice (each is one limb against the
  other).
- A hold longer than two swings' time lets the note through
  (`waits.held_rows`): the swing waited for is not coming.
