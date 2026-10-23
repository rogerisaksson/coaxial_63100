# Synth: her loop as a synthesizer

The latent loop is a synthesizer feeding a control system. Read as a producer
reads a patch: a master clock on a grid, transients that gate, LFOs on the
voices, envelopes, sidechains ducking one voice from another, a filter stack
on the top, a DC offset driving the Q. Each term names one quantity in the
code with its number; what her walk's patch does not have is said so. The
parts themselves: [planners](planners.md).

## The patch as built

Her walk (`sprung.WALK`: stand, sway, step, walk; `ortho.py --walk --look`:
0.70 m/s, 409 J/m, 2.55 off a woman's):

| stage | setting | her loop's |
| --- | --- | --- |
| master clock | 108 BPM a step (0.9 Hz a stride), free-running, no PLL: the clock turns at `HZ`, the body follows; re-locked only by a half-turn flip when her sway leads the other way (`se2.twisted`, `THETA` + pi) and by a due note waking a dead rhythm (`waits.woken_rows`) | `latent.THETA`, `HZ`, `TURNS` |
| grid, groove | hard-quantized: a landing on its beat's share (`u`), the second limb exactly half a turn behind (`LIMBS`), groove 0, humanize 0 - the one deviation the capture point's sidechain hurrying a swing | `stride.beat`, `stride.hurried` |
| gate | a limb's swing 40 % of the turn, stance 60 (a woman's 60-62): `SWING` 0.4, `PHI_W`; opened by the heel transient, closed by the let-go | `stride.beat`: `on`, `off`, `u`; `leap.standing` |
| transient | the heel strike: 1.37 of her weight, 0 ms attack, a 15 mm fall, the gate's trigger at 15 N (`TOUCH_N`); the sole turned flat at 400 /s^2 (`FLAT_K`), its heel let 0.6 rad up at 6 rad/s at most (`HEEL_RAD`, `HEEL_RAD_S`) | `stride.heels`, `looks.FORM` |
| LFO | one sine per voice on the clock: the lateral 0.31 (of her stance half width) and the pelvis list 0.4 at +108 deg to the drive (a 1/2 note at the stride), the drive 0.6, the lift 1.0, the rock 0.6, the turn 0.4, the arms 0.8 in phase | `AMP`, `PHASE`, `GAIN`; `gait_base.lisp` |
| mid/side | the feet 0.13 legs apart (a woman's 0.02-0.22), never mono-summed: each landing is laid its own side of the other by `APART_M` 0.1 m, a limiter the width voice cannot pass (roll +0.86's width -0.48 moved nothing); the hips narrow stereo: pelvis roll 5.0 deg at the floor of a woman's 5-15 | `stride.meets`, `WIDTH`, `LIST` |
| sidechain, duck | the heel transient ducks her height 15 mm as it lands and the knee loads to 1.37; the release is the height spring's, critically damped (`HEIGHT_KD` 100 /s^2 and 20 /s, 122.8 and 20.9 on her hips by `HIPS_BY`: zeta 0.94-1.0) - no overshoot, no fling to the other side; the weight crosses on the lateral LFO alone | `wbc.HEIGHT_KD`, `impedance.ask` |
| low-pass, top | a spring and a damper a voice, 12 dB/oct: the trunk's turns and the arms at 100 /s^2 and 20 /s (`TURN_KD`, `POSTURE_KD`, zeta 1.0) behind the note's blend (6 dB/oct, linear); the whole-body step weighs the upper joints 1 against the centre's 1 000 (`W_POSTURE`, `W_COM`): the head nods 2 deg, the elbow 2.8 (a woman's 5-45: over-filtered), the arm 28 | `modal.rest`, `wrench.W_*` |
| counter-phase | the arms 0.8 in phase with the drive (each arm against its own leg: the counterweight), the pelvis turn 0.4 in phase | `phase_offset` 0 deg |
| DC offset | the trunk's pitch 0 (0.01 on the walk-on), the centre over the soles' middle 0 along, the crouch 0.065 (`WALK_ON`) | `Z[PITCH]`, `Z[SAGITTAL]`, `Z[AXIAL]` |
| Q | the sway's spring on the capture point (`REST_K` 2.0: a spring K omega, a damper omega + K): zeta 1.03 at omega 3.3 /s, Q 0.49 - dead flat, no ring, nothing near self-oscillation | `poise.into`, `modal.rest` |

## The catwalk patch

A runway take as a patch, and what the loop does with it
([sequencer](../findings/sequencer.md)):

| the patch | the take | hers so modified |
| --- | --- | --- |
| hard-quantize the calcaneal transients to the grid, groove 0 | 102 BPM (0.85 Hz) | 108 BPM; pace -0.05 says 102, her stance held it |
| mono-sum the tarsal sub to the dead-centre sagittal line | feet 0.08 legs apart | not available: `APART_M` limits the mid channel at 0.1 m, feet stay 0.13 |
| pan the pelvic-obliquity LFO wide, sidechain-ducked by each heel | roll 23 deg, swinging 0.45 | roll +0.86 (list 1.26): 18.8 deg, named a catwalk; the duck the height spring's, no overdriven release |
| 24 dB/oct low-pass on the thoracic and scapular jitter | arms 82 deg, swinging | her top is 12 + 6 dB/oct; the arms' swing +1.13 (1.93) fell her with the stride +0.40 |
| bias the L4-L5 lordotic DC offset to the edge of self-oscillation | trunk lean -1.4 deg, crouch 0.26 | lean -0.16 is sayable (`Z[PITCH]`); the Q is `REST_K`'s, dead flat: no knob rings her |

## Terms

| term | her loop's | number |
| --- | --- | --- |
| voice | a mode of her latent vector | 11, `ortho.MODES` |
| note | a row: its voices' targets, its attack, its track | `ortho.ROW`, `latent.VALS`, `SAID` |
| clip | a take's beats as notes, legato | `takes.beats`, `takes.wire` |
| patch, preset | a text; a gait take as amounts on the walk | the lisp; `takes.amounts` |
| modulation, mod wheel | the patch's knobs moved by named amounts; a word of her movement language as one | `gait.MODIFIERS`, `tools.sim.normal` |
| patch file | a dry state, a modifier, a morph, a schedule in bars, read as written | `machine.dsp`, machine/runway.lisp |
| suite | the ortho-DSP modules: a manifold, transducers, topologies, modulators, limiters, a sequencer; their patches gaits voiced on her walk by their changes off the nominal, which plays her walk | `dsp.read`, `voicing.voiced`, `VOICES`, machine/ortho_dsp/ |
| system identification | a take's clock on its strikes, its coronal mid and side, its sidechain, its filter bank, emitted as a modifier against her walk | `tools/sim/identify.py` |
| autotune | a root's knobs each moved by its measured sensitivity on six physics residuals, a step within 0.15 of each knob's scale, her form held; of her texts' 173 named knobs 100 reach her | `machine.tune`, `tools/sim/tune.py` |
| sidechain | each touchdown latches a duck of her height: depth, attack, release | `ortho.DUCK`, `stride.ducked` |
| sync trigger | the morph begun in the double support past the lateral voice's zero crossing, the asked ZMP in the soles' boxes | `patch.crossing`, `window`, `locked` |
| program change | a text said in place of what was to come | `worker.heard_say`, `se2.said` |
| envelope | a voice's blend from where it is to its note: attack the blend, sustain the note held, release the row back to rest (`REST_MS` 200), decay the brake (`gait_brake` 1800 ms, `sprung.BRAKE` 500) | `latent.FROM`, `TO`, `BLEND` |
| legato, glide | contiguous notes drawn as one curve | `ortho.CURVE`, `latent.CONTIG` |
| flam | two notes in one tick: they add (Hz 1.78 = 0.86 + 0.92) | `latent.step` |
| rest | a stand, the oscillator halted | `cpg_override (halt_oscillators)`, `GAIN` 0 |
| hold | a note waiting on the soles or the floor | `waits.held_rows`, `risen_rows`, `landed_rows` |
| quantize | a landing drawn to a tread's grid | `stride.footholds` |
| transpose | a floor's rise read into her height | `stride.floor_mean`, `impedance._landings` |
| generator out | each voice's reference, rate and acceleration, spring, damper, the phases, the clock's rate | `latent.OUT` |
| control system | the ask on her body, the whole-body step, the drives | `impedance.ask`, `wrench.step`, `edge` |

## Sidechains

| from | to | where |
| --- | --- | --- |
| the twist told: speed, strafe, yaw | the drive's and arms' depth, the tempo, the lateral's target | `se2._modulate_cpg` |
| a shove felt, the twist blocked | the sway and the drive ducked to the brake's, the stand's targets | `touch.step` |
| her capture point past the soles | the swing's rate up | `stride.hurried` |
| her soles' grip, told or felt | the tempo capped, the landings' reach | `situation`, `grip` |
| the floor told | the swing's clearance, its tread, her height's rise | `impedance._landings` |
| her need to step | the stride on at any drive | `stride.beat` |
| a seat bearing her; nothing bearing her | the notes that raise her held; those that lower her held | `waits.risen_rows`, `landed_rows` |
| her rhythm died down | a due note restarts the clock | `waits.woken_rows` |

## The master insert

The anti-spasm supervisor (`anti_spasm_supervisor.lisp`) is the last stage of
every patch, gait or maneuver, before her rows are laid. Each of its params is
named on her loop (`machine.supervisor`); a gait saying another is refused:

| stage | as written | on her loop |
| --- | --- | --- |
| velocity noise gate | 0.0035 rad/s, 4 / 25 ms, smooth-step | open: under her encoders' step, 0.05 rad/s at 1 kHz |
| D-term low-pass | Butterworth 2nd order, 42 Hz, causal | open ([scenarios](../todo/scenarios.md) 14) |
| jerk | each drive's current loop, cubic soft knee | each drive's asked acceleration and its torque's share of what is cut: its slew over its rotor's inertia, 5.75-16.1 M rad/s^3 |
| torque slew | each drive's current loop | its peak x 2 pi x 2.5 kHz, the loop's sampling bound: 806-3 089 kN m/s |
| damped least squares | sigma_min 0.045, lambda_max 0.18, raised cosine | open: lambda 0.02 at every sigma (`wrench.DAMP`) |
| boundary buffers | 0.06 rad, 140 /s^2, zeta 1.0; knee, ankle, shoulder, neck | rows of her step before her stops: the knee's straight lock and high stop, the neck's and head's stacks' travel; her ankles and shoulders have none (`wbc.Body.buffers`) |
| Schmitt comparators | 0.18 / 0.06 of her weight, 45 ms dwell, 18 ms blanking | open: a sole stands from 15 N, a segment bears from 5 N |
| bumpless handover | C2, 85 ms, critically damped | open |
| zero-crossing bridge | 0.85 N m window, 0.35 kept, no preload; hips, knees, ankles, lumbar | open; no preload: one reversible drive a joint, no antagonist |
| energy tank | 12 J, 6-35 Hz, 3 reversals past 4.5 W; 3.2 x damping, -9 dB, 450 ms | each drive's rate and torque in the window, its half cycles' energy; on a parasitic oscillation, its own damping injected and its gain rolled back |

## No voice yet

- Flight: no note lifts both soles; a jump is the leap's throw (`leap`), a jog
  or a run on the oscillator her walk at the take's tempo and drive, 1.3-2.0
  m/s. A gate past 50 % (both legs swinging) is not a knob: `SWING` is a
  constant, not a note's.
- Mono-summed feet: her width asked narrower lowers the landings' floor
  (APART_M) and `stride.guarded` keeps the swing off as it passes, but she
  holds 81-95 mm at most (findings/sequencer.md). A ringing Q: `REST_K` and
  `HEIGHT_KD` are constants of the law, not the patch's.
- Both arms overhead, both heels up: no voice (each is one limb against the
  other).
