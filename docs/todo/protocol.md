# The somatic dual-domain kernel's protocol

Seven steps on her loop and five tests in the lodge (`tools/sim/house.py`):
her loop the real-time domain, a tuner apart the background one. Its texts
as given: host/machine/ortho_kernel/, a module a file (the two master
texts' 26). Her figure, drives and pack are bound from `machine.figure`,
`machine.drives` and her build; the lodge's door, table, chair, stool, bed
and loft stair (5 steps of 0.15 m along its right wall) unknown to her at
t = 0; no fingers grip. Each mechanism's numbers are its text's
(`dsp.read(dsp.KERNEL)`); what is done: [kernel](../findings/kernel.md).

1. **Step 2, the back-EMF envelope**: tau_max, tau_min each pass from her
   DC link's volts / sqrt(3) and each joint's speed, her law's torque box
   asymmetric (wrench.py, wbc_wrench.c); her boards already limit their
   current so. DOD: her law never asks past it; a capture step's torques
   inside it.
2. **Step 2, the insert's inner stages**: its Butterworth D-channel at 42 Hz
   and its backlash bridge, each alone felled her turning walk (scenarios
   item 14). DOD: both on her loop at their text's numbers, every suite as
   green.
3. **Step 2, the discrete phase margin**: the text's lock-in at 15, 22, 30
   and 38 Hz on her loop's return ratio (her observer's 35 Hz lag, her
   barrier at 14 /s and her field's 43 ms on a 3 ms law pass, against her
   gearboxes' resonances, `physics.BOXED` 1: 14.8 Hz at the ankle to 41 at
   the shoulder); under 38 deg K_d and gamma derated to their floors. DOD:
   each's gain and phase measured, none pumping energy into them.
4. **Step 3's leak walking**: her chest's yaw asked walking moves her
   centre of mass 0.12-0.15 m/s^2 and her ankles 3.4-4.6 N m where her
   soles' bounds bind. DOD: under 1e-4 of the chest's acceleration walking,
   as standing; her walk as solved.
5. **Step 4, reachability**: rho = |hip - contact| / her leg's 890 mm each
   pass; from 0.86 to 0.98 her sole's cone onto her forefoot under her
   ankle's plantarflexion, past 0.99 her soles' weight to 0 and her legs let
   go into damped pendulums. DOD: on a seat over her knees her feet hang
   let go; stepping down her forefoot catches.
6. **Step 4, the Coulomb slide**: a thigh's and the seat's patch sliding at
   mu_k F_z, F_z under 0.55 of her weight, the text's 65 Hz dither, inside
   her capture point's barrier. DOD: she slides onto and off a seat over her
   knees, and 180 mm deeper onto the bed.
7. **Step 5, the body schema**: the text's 18 capsules on her segments and 4
   payload slots, their distances to each other (13 pairs), the room's
   quadrics and the three slabs by a log-sum-exp at 25, each swollen by its
   braking distance at her drives' back-EMF-bounded torque; the ECBF
   d'' + k_d d' + k_p d >= 0 (11 Hz, 1.05) in her dynamically consistent
   null space first, her balance bus past its saturation; open only to a
   scheduled patch. DOD: her walk's pairs' clearances measured, none under
   its floor; an arm swung at a table corner stops short of it, 0 N m to her
   ankles.
8. **Step 6, the wayfinder's two ways**: her ECBF's multipliers deflect her
   geodesic flow by rank-1 Sherman-Morrison, held past 140 ms into the
   metric; the streamline 600 ms ahead pre-shapes her null-space posture
   where the gap lies between her contorted and nominal widths; her shins'
   and knees' clearance pulled back through her leg's Jacobian over swing
   0.05-0.38. DOD: no oscillation before a doorway; a foot placed off a
   chair's crossbar.
9. **Step 6, her map**: her right-invariant EKF (`machine.inekf`) and a
    registry of 256 dual quadrics from her eyes, three SDF bands at her
    knee, navel and crown, the kernel's affordances and funnels. DOD: the
    lodge's furniture in the registry; its blind spots remembered 45 s.
10. **Step 7, the lowering and the tuner**: `machine.lowering` folds a
    gait into a bank of the text's ABI (wbc_soa.h), its biquads seeded, a
    bank carried over her link's table. Left: her loop's channels through a
    bank's biquads and its fields, a bank taken at her barrier (`tap.due`)
    in her run of the room's schedule (`tools/sim/runtime.py`). DOD: the
    room's schedule swapped in her run with no step in her torques; the
    tuner converged on it.
11. **Test 1, door and sill**: the door ajar at 0.80 of her shoulders, a 40
    mm bar in the doorway, a table's corner inside. DOD: her chest yawed
    ~60 deg and her elbows tucked 600 ms before it, her swing over the bar
    by the sub band; past the table her trailing elbow clear of its corner,
    0 N m to her ankles.
12. **Test 2, a chair shoved**: a chair across her way to the stool, two legs
    on a rug, no fingers. DOD: her forearms or carpal heels on it at 18 N
    or more (`ulnar-carpal-shove-bilateral` read against her skin), its
    quadric in her cone and her capsules, lifted 25 mm where the rug holds it
    past 45 N, her shins clear of its legs, parked out of her corridor.
13. **Test 3, the stool and the pulled seat**: a stool 0.75-0.82 m; sat on
    and risen from; pulled back mid-descent. Pulled as the seat touches her
    (2.0 s of the chair scene) her sit's rows straighten her trunk past her
    heels and she falls. DOD: on, her forefoot, her thighs perched, her
    carpal heels braced, slid back till her ischia centre, her feet let go;
    off, slid forward till her forefoot catches before her ischia unload;
    pulled, the seat's weight 0, a capture step to xi = c + c'/omega_0, her
    gaze level, her torques inside their back-EMF bounds.
14. **Test 4, the bed**: from the stool to the bed, slid 180 mm deeper,
    rolled, high-Z, risen with a carpal heel on the table 650 mm off. DOD:
    the mattress's f0 within 180 ms of contact, its notch, no rebound; 0 A in
    her phases from w_dorsal 0.96; her knee and hip peaks 20 % under her
    unbraced rise's.
15. **Test 5, the loft stair up and down**: five steps each way, a forearm
    skimming the wall. DOD: up, flat-foot strikes on concentric work; down,
    her stance knee eccentric, her pelvis dropped, her forefoot catching
    each step, shin and heel 32 mm or more clear of each nosing; V_bus
    under 60 V.
