# Motor Engrams, Checkpoints & Stunt Similarity

Motor engrams (`engrams`) are hot-swappable, closed-loop sensorimotor programs
(compiled C `wbc_sensed_t` tables, `ortho.ROW` latent curves, CPG patterns, or
beat programs) loaded and swapped live without cold restarts or pose jumps.

## Core Modules

- `host/machine/process/engrams.py`: `load`, `swap` (C0 handover), `unload`,
  `unload_all`, `load_specs`, `stunt_bundle`.
- `host/machine/process/hotswap.py`: unified compilation, dry-run validation,
  live hot-swap, and MuJoCo checkpoint/pose reload for LLM-generated C11 cores,
  traced Python/NumPy `motion.HOOKS` cores, and JSON/Lisp engrams.
- `host/machine/process/engram_catalog.py`: prepared `CATALOG`, `SUITES`, and
  `STUNT_BUNDLES` mapped to human mocap takes in `stunts/`.
- `host/machine/process/checkpoint.py`: `capture`, `restore`, `advance_until`
  for bit-exact MuJoCo (`mj_copyData`) and controller snapshot branching.
- `host/tools/sim/hotswap.py`: CLI for offline compilation, live TCP hot-swap
  and checkpoint reload (`--attach PORT`), and parallel candidate iteration.
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

4. **LLM-generated cores & engrams: compile, live hot-swap, and MuJoCo reload**
   (`hotswap.py` compiles C11 SIMD kernels, Python/NumPy hooks via `bodied`, and
   JSON/Lisp engrams, dry-runs candidates for shape and finiteness before live
   swap, and reloads the robot to saved states or explicit `(where, yaw, qpos)`
   poses in MuJoCo):

   ```powershell
   python -X utf8 host/tools/sim/hotswap.py --compile candidate.json
   python -X utf8 host/tools/sim/hotswap.py --attach 8765 --save pre_dock
   python -X utf8 host/tools/sim/hotswap.py --attach 8765 --swap candidate.json
   python -X utf8 host/tools/sim/hotswap.py --attach 8765 --reload pre_dock --where 0.2,-0.4 --yaw-deg 15
   python -X utf8 host/tools/sim/hotswap.py --iterate candidates.json --scene chair_behind --say sit_chair --jobs 16
   ```
