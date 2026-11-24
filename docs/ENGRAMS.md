# Motor Engrams, Checkpoints & Stunt Similarity

Motor engrams (`engrams`) are hot-swappable, closed-loop sensorimotor programs
(compiled C `wbc_sensed_t` tables, `ortho.ROW` latent curves, CPG patterns, or
beat programs) loaded and swapped live without cold restarts or pose jumps.

## Core Modules

- `host/machine/process/engrams.py`: `load`, `swap` (C0 handover), `unload`,
  `unload_all`, `load_specs`, `stunt_bundle`.
- `host/machine/process/engram_catalog.py`: prepared `CATALOG`, `SUITES`, and
  `STUNT_BUNDLES` mapped to human mocap takes in `stunts/`.
- `host/machine/process/checkpoint.py`: `capture`, `restore`, `advance_until`
  for bit-exact MuJoCo (`mj_copyData`) and controller snapshot branching.
- `host/tools/sim/engram_mc.py`: parallel Monte Carlo combinatorics from
  pre-dock or pre-maneuver checkpoints (`engram_scenes.py` evaluators).
- `host/tools/sim/stunt_similarity.py`: 0..100 anatomical fidelity scoring and
  bottleneck diagnosis against mocap references (`host/tests/takes/stunts.json`).

## When to Use

- Use `engrams` and `checkpoint` whenever exploring chair/bed docking variants,
  posture transitions, or gait/turn shaping without tuning global constants.
- Capture a checkpoint right before docking or task transition, then restore in
  place between trials so every engram bundle branches from identical physics.

## Parallel Workflows

1. **One stunt with a bundle of engrams in parallel** (worker pool evaluates
   each candidate or `;`-separated slot Cartesian product against one scene):

   ```powershell
   python -X utf8 host/tools/sim/engram_mc.py crouching_down `
     --engrams crouch_down_mocap,crouch_down_calm,crouch_down_brisk --jobs 16
   python -X utf8 host/tools/sim/engram_mc.py chair_dock `
     --engrams "chair_mount_stock,chair_mount_brisk;chair_dismount_stock,chair_dismount_calm"
   ```

2. **Multiple stunts in parallel, each hot-swapping its own engram bundle**:

   ```powershell
   python -X utf8 host/tools/sim/engram_mc.py chair_dock bed_dock crouch_dock turn_dock `
     --samples 3 --jobs 24
   python -X utf8 host/tools/sim/engram_mc.py --all-stunts --samples 1 --jobs 24
   ```

3. **Anatomical & kinematic stunt similarity analysis** (0..100 score, `far_sig`,
   `far_all`, `scale_min`, hand jerk, bottlenecks, and `--against` A/B deltas):

   ```powershell
   python -X utf8 host/tools/sim/stunt_similarity.py --jobs 24 --save baseline.json
   python -X utf8 host/tools/sim/stunt_similarity.py --bundle best --against baseline.json --jobs 24
   python -X utf8 host/tools/sim/stunt_similarity.py running_forward --spec cand.json
   ```
