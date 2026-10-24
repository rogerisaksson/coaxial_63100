# The dual-domain kernel's protocol

Six steps on her loop and four tests in the lodge (`tools/sim/house.py`):
her loop the real-time domain, a tuner apart the background one. Its texts
as given: host/machine/ortho_kernel/. The robot's figure, drives and the
room are bound from `machine.figure`, `machine.drives` and `machine.room`,
the furniture unknown to her at t = 0. Each mechanism's numbers are its
text's (`dsp.read(dsp.KERNEL)`).

1. **Step 2, the insert's inner stages**: its Butterworth D-channel at 42 Hz
   and its backlash bridge, each alone felled her turning walk (scenarios
   item 14). DOD: both on her loop at their text's numbers, every suite as
   green.
2. **Step 3's leak walking**: her chest's yaw asked walking moves her
   centre of mass 0.12-0.15 m/s^2 and her ankles 3.4-4.6 N m where her
   soles' bounds bind ([kernel](../findings/kernel.md)). DOD: under 1e-4 of
   the chest's acceleration walking, as standing; her walk as solved.
3. **Step 5, her map**: a right-invariant EKF on SE_2(3) (IMU, contacts by
   their weights), a registry of 256 dual quadrics from her eyes, three SDF
   bands cut at her knee, navel and crown, the kernel's affordances and
   funnels compiled into the bank her loop does not hold. DOD: the EKF's
   drift against `estimate`'s measured; the lodge's furniture in the
   registry (kind, centre, size errors); a funnel chain to the bed planned.
4. **Step 6, the autotuner closed**: `tools/sim/runtime.py` on the room's
   schedule, a window a stride or a one-shot, its table swapped at the
   zero crossing C2. DOD: converged on it; the swap's jerk within her walk's.
5. **Test 1, door and sill**: the door ajar at 0.80 of her shoulders, a low
   bar in the doorway. DOD: her swing over the bar by the sub band, her
   chest yawed through by the high band, her step rate kept, the yaw's
   leak to her ankles measured.
6. **Test 2, a chair across the bed's dock**: on a rug and on boards. DOD:
   the dock's funnel blocked, the chair moved outside her corridor and its
   slot updated; a lift refused by name (her hands grip nothing).
7. **Test 3, the chair pulled away**: as her 180 deg turn ends, her pelvis
   over the seat. DOD: the ischial weight 0 throughout, a squat or a
   capture step back, her gaze level, V_bus under its ceiling.
8. **Test 4, the bed**: lain in, high-Z, risen to the table's chair. DOD:
   the mattress's f0 identified within 180 ms, its notch, no rebound;
   0 A in her phases from w_dorsal 0.96; the rise braced by a hand on the
   bed's edge or the table, her knees' peak lower than unbraced; the bed's
   and the chair's height and stiffness written to the registry.
