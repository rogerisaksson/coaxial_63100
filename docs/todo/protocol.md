# The somatic dual-domain kernel's protocol

Seven steps on her loop and five tests in the lodge (`tools/sim/house.py`):
her loop the real-time domain, a tuner apart the background one. Its texts
as given: host/machine/ortho_kernel/ (the somatic ones `somatic_*.lisp`). Her
figure, drives and pack are bound from `machine.figure`, `machine.drives` and
her build; the lodge's door, table, chair, bed and loft stair (5 steps of
0.15 m along its right wall) unknown to her at t = 0; no fingers grip. Each
mechanism's numbers are its text's (`dsp.read(dsp.KERNEL)`); what is done:
[kernel](../findings/kernel.md).

1. **Step 1, the somatic manifolds**: her observer's points and her rows'
   manifolds take the forefoot (`metatarsal-equinus-bilateral`), the
   thighs' backs (`posterior-femoral-bilateral`) and the forearms' and
   carpal heels' push (`ulnar-carpal-shove-bilateral`). DOD: each read
   against her skin in a scene that loads it.
2. **Step 2, the insert's inner stages**: its Butterworth D-channel at 42 Hz
   and its backlash bridge, each alone felled her turning walk (scenarios
   item 14). DOD: both on her loop at their text's numbers, every suite as
   green.
3. **Step 3's leak walking**: her chest's yaw asked walking moves her
   centre of mass 0.12-0.15 m/s^2 and her ankles 3.4-4.6 N m where her
   soles' bounds bind. DOD: under 1e-4 of the chest's acceleration walking,
   as standing; her walk as solved.
4. **Step 4, reachability**: rho = |hip - contact| / her leg's reach each
   pass; from 0.86 to 0.98 her sole's cone onto her forefoot under her
   ankle's plantarflexion, past 0.99 her soles' weight to 0 and her legs let
   go into damped pendulums. DOD: on a seat over her knees her feet hang
   let go; stepping down her forefoot catches.
5. **Step 5, the Coulomb slide**: a thigh's and the seat's patch sliding at
   mu_k F_z, F_z under 0.55 of her weight, the text's 65 Hz dither. DOD: she
   slides onto and off a seat over her knees, and 180 mm deeper onto the
   bed.
6. **Step 6, stairs and the shove**: the step's energy, +m g dz ascending,
   -m g |dz| descending (stance knee eccentric, the pelvis dropped 0.08 rad,
   the swing ankle 0.36 rad down); her forearms or carpal heels pushing a
   chair at 18 N or more, lifted 25 mm past 45 N of friction. DOD: Test 5;
   Test 2.
7. **Step 7, her map and the tuner**: a right-invariant EKF on SE_2(3)
   (`machine.inekf`: IMU, her soles' contacts by their weights), a registry
   of 256 dual quadrics from her eyes, three SDF bands at her knee, navel
   and crown, the kernel's affordances and funnels compiled into the bank
   her loop does not hold; `tools/sim/runtime.py` on the room's schedule, its
   table swapped at the zero crossing C2. DOD: the EKF's drift against
   `estimate`'s measured; the lodge's furniture in the registry; the tuner
   converged on the room's schedule.
8. **Test 1, door and sill**: the door ajar at 0.80 of her shoulders, a 40
   mm bar in the doorway. DOD: her swing over the bar by the sub band, her
   chest yawed ~60 deg through by the high band, her step rate kept, the
   yaw's leak to her ankles measured.
9. **Test 2, a chair shoved**: a chair across her way to the stool or the
   bed. DOD: her forearms or carpal heels on it at 18 N or more, it shoved
   out of her corridor, lifted 25 mm where the rug holds it past 45 N, its
   slot updated.
10. **Test 3, the stool and the pulled chair**: a stool 100-180 mm over her
    knees; sat on and risen from; a chair pulled away mid-pivot. Pulled as
    the seat touches her (2.0 s of the chair scene) her sit's rows
    straighten her trunk past her heels and she falls. DOD: on, her
    forefoot, her thighs perched, her carpal heels braced, slid back till
    her ischia centre, her feet let go; off, slid forward till her forefoot
    catches her capture point; pulled, the seat's weight 0, a squat or a
    capture step back, her gaze level, V_bus under its ceiling.
11. **Test 4, the bed**: from the stool to the bed, slid 180 mm deeper,
    rolled, high-Z, risen. DOD: the mattress's f0 within 180 ms of contact,
    its notch, no rebound; 0 A in her phases from w_dorsal 0.96; risen with
    her legs over the edge and her carpal heels pushing, her lumbar and hip
    peaks 25 % under her unbraced rise's.
12. **Test 5, the loft stair up and down**: five steps each way, a forearm
    skimming the wall. DOD: up, flat-foot strikes on concentric hip and knee
    work; down, the forefoot catching each step 34 mm or more clear of its
    nosing; V_bus under its ceiling.
