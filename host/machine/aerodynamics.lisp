; General aerodynamic building blocks for Coaxial 63100 rotorcraft (`aerodynamics.lisp`).
;
; Any multi-rotor or single-main-rotor helicopter can be puzzled together by composing
; `airframe`, `rotors`, `main_rotor`, `tail_rotor`, and `drive_load` blocks via `(use ...)`.

(defblock airframe standard_airframe
  (air_density 1.2)
  (parasite_cda 0.15))

(defblock rotors quad_x_16x14
  (arm_length 0.30m)
  (disc_height 0.06m)
  (disc_radius 0.2032m)
  (disc_half_height 0.008m)
  (k_thrust 0.00010995)
  (k_drag 0.000004136)
  (inflow_j0 1.05)
  (blade_pitch 0.3556m))

; A 550's 1.25 m rotor on 2 kg, one size with the others; its cyclic as quick on the inertia
; of its size as the 1.0 m rotor's was.
(defblock main_rotor swashplate_550_3d
  (disc_height 0.22m)
  (radius 0.625m)
  (collective_limit 14.0deg)
  (cyclic_limit 10.0deg)
  (k_thrust 0.0004398)
  (k_cyclic 0.000025)
  (k_drag 0.000004136)
  (profile_drag_share 0.35)
  (induced_drag_share 1.65)
  (cyclic_drag_share 0.50)
  (inflow_j0 1.05)
  (blade_pitch 0.3556m)
  (inflow_floor 0.35))

(defblock tail_rotor tail_110_antitorque
  (boom_arm 0.85m)
  (radius 0.11m)
  (pitch_limit 16.0deg)
  (k_thrust 0.000075))

(defblock drive_load swashplate_tail_load
  (main_base 0.42)
  (main_collective 1.65)
  (main_cyclic 0.55)
  (main_ceiling 1.25)
  (tail_base 0.25)
  (tail_pitch 1.25)
  (tail_ceiling 1.00))

(defblock flapping_wings dune_4wing_flapper
  (wing_span 0.56m)
  (wing_chord 0.085m)
  (shoulder_x 0.068m)
  (fore_z 0.085m)
  (aft_z -0.055m)
  (shoulder_y 0.048m)
  (stroke_limit 38.0deg)
  (pitch_limit 24.0deg)
  (sweep_fold_limit 52.0deg)
  (k_thrust 0.0004460)
  (k_lift 0.16)
  (k_pitch 0.0000172)
  (k_roll 0.0000180)
  (k_yaw 0.0000300)
  (k_drag 0.000004220)
  (inflow_floor 0.35)
  (gear_ratio 6.0)
  (beat_pulse 0.30))

(defblock drive_load ornithopter_wing_load
  (use swashplate_tail_load)
  (main_base 0.44)
  (tail_base 0.40)
  (tail_pitch 1.35)
  (tail_ceiling 1.20))

; The fuselage's 0.09 x 0.16 m front at 0.8, the gear's legs and wheels, m^2; the wing's own
; drag is cd0's. At 0.03 the airframe was 0.12 of its wing in drag, three times a 3D plane's.
(defblock airframe monoplane_airframe
  (use standard_airframe)
  (parasite_cda 0.016))

; 51 g/dm^2 on 2 kg, a 55-inch 3D plane's, one size with the others (64 inches at 39 before): on
; 1.30 x 0.26 m (59 g/dm^2) it stalled at 9.4 m/s and turned on 32 m at 14 m/s. Its surfaces as
; quick on the inertia of its size. Its propeller a 1.4 m 3D model's 17x8, fixed, on its speed:
; static thrust and power coefficients 0.08 and 0.04, k = C rho D^4 / (2 pi)^2 and C rho D^5 /
; (2 pi)^3. The quad's 16x14 grown to 20 in, or four of them's thrust on a pitch it had, 7.5 g.
(defblock aero_surfaces unlimited_3d_monoplane
  (wing_span 1.40m)
  (wing_chord 0.28m)
  (wing_area 0.392)
  (prop_radius 0.2159m)
  (prop_z 0.356m)
  (tail_arm 0.484m)
  (stab_area 0.046)
  (fin_area 0.035)
  (k_thrust 0.0000863)
  (k_drag 0.000002965)
  (blade_pitch 0.2032m)
  (inflow_j0 1.05)
  (prop_inertia 0.0004)
  (cl_alpha 4.2)
  (cd0 0.035)
  (cd_induced 0.085)
  (elevator_limit 45.0deg)
  (aileron_limit 38.0deg)
  (rudder_limit 48.0deg)
  (k_pitch 0.00010393)
  (k_roll 0.00007559)
  (k_yaw 0.00011810)
  (propwash_share 0.85)
  (inflow_floor 0.35))

; The propeller's torque over its k_drag w^2: main_base unloaded, main_collective more by its
; thrust's share of the static, main_cyclic by its surfaces' squares, main_ceiling at the most;
; the contra stage's by the tail's - both boards on the propeller's own rotor.
(defblock drive_load aerobat_prop_load
  (use swashplate_tail_load)
  (main_base 0.34)
  (main_collective 0.66)
  (main_cyclic 0.72)
  (main_ceiling 2.5)
  (tail_base 0.32)
  (tail_pitch 1.15)
  (tail_ceiling 1.15))

(defaerodynamics shared
  (use standard_airframe))
