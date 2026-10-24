# The dual-domain kernel's protocol

Six steps on her loop and four tests in the lodge (`tools/sim/house.py`):
her loop the real-time domain, a tuner apart the background one. Its texts
as given: host/machine/ortho_kernel/. The robot's figure, drives and the
room are bound from `machine.figure`, `machine.drives` and `machine.room`,
the furniture unknown to her at t = 0. Each mechanism's numbers are its
text's (`dsp.read(dsp.KERNEL)`).

1. **Step 1, the SoA bridge in C**: `rcu`'s block laid by a C header -
   64-byte lines, two banks, the bank index atomic, a ring a producer -,
   the block locked in memory. DOD: the C and `rcu` agree on every offset
   (a test); her loop's C writes a ring and takes a bank; her walk bit for
   bit linked and unlinked.
2. **Step 2, a momentum observer**: the external wrench on each of her
   patches - plantar, ischial, trochanter, dorsal, palmar - from her
   encoders, her IMU and her drives' torques over M(q), at the text's
   35 Hz. The insert's Butterworth D-channel and backlash bridge are
   scenarios item 14. DOD: each patch's normal force against MuJoCo's
   through walk, sit, lie and rise, its error measured and in findings.
3. **Step 3, her step's budget and its null space**: the pass's cost
   against 350 us; a posture ask's leak into her soles' wrenches and her
   ankles' torques. DOD: both measured; the chest's yaw ask leaking nothing
   her balance reads.
4. **Step 4, weights by force, a barrier on her capture point**: each
   manifold's weight sat(<f, n> / m g) of the observer, slewed up 4.5 /s,
   down 45 /s; the capture point's barrier h' + gamma h >= 0 in her step;
   the blind admittance field (a pelvis seeking a seat until one bears
   her); her DC link's voltage and the shunt below its ceiling. DOD: her
   rows' told weights replaced by measured ones, her walk, sit and rise as
   green; V_bus under 60 V through every scene.
5. **Step 5, her map**: a right-invariant EKF on SE_2(3) (IMU, contacts by
   their weights), a registry of 256 dual quadrics from her eyes, three SDF
   bands cut at her knee, navel and crown, the kernel's affordances and
   funnels compiled into the bank her loop does not hold. DOD: the EKF's
   drift against `estimate`'s measured; the lodge's furniture in the
   registry (kind, centre, size errors); a funnel chain to the bed planned.
6. **Step 6, the autotuner closed**: `tools/sim/runtime.py` on the room's
   schedule, a window a stride or a one-shot, its table swapped at the
   zero crossing C2. DOD: converged on it; the swap's jerk within her walk's.
7. **Test 1, door and sill**: the door ajar at 0.80 of her shoulders, a low
   bar in the doorway. DOD: her swing over the bar by the sub band, her
   chest yawed through by the high band, her step rate kept, the yaw's
   leak to her ankles measured.
8. **Test 2, a chair across the bed's dock**: on a rug and on boards. DOD:
   the dock's funnel blocked, the chair moved outside her corridor and its
   slot updated; a lift refused by name (her hands grip nothing).
9. **Test 3, the chair pulled away**: as her 180 deg turn ends, her pelvis
   over the seat. DOD: the ischial weight 0 throughout, a squat or a
   capture step back, her gaze level, V_bus under its ceiling.
10. **Test 4, the bed**: lain in, high-Z, risen to the table's chair. DOD:
    the mattress's f0 identified within 180 ms, its notch, no rebound;
    0 A in her phases from w_dorsal 0.96; the rise braced by a hand on the
    bed's edge or the table, her knees' peak lower than unbraced; the bed's
    and the chair's height and stiffness written to the registry.
