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

(defblock main_rotor swashplate_500_3d
  (disc_height 0.175m)
  (radius 0.50m)
  (collective_limit 14.0deg)
  (cyclic_limit 10.0deg)
  (k_thrust 0.0004398)
  (k_cyclic 0.000016)
  (k_drag 0.000004136)
  (profile_drag_share 0.35)
  (induced_drag_share 1.65)
  (cyclic_drag_share 0.50)
  (inflow_j0 1.05)
  (blade_pitch 0.3556m)
  (inflow_floor 0.35))

(defblock tail_rotor tail_150_antitorque
  (boom_arm 0.68m)
  (radius 0.15m)
  (pitch_limit 16.0deg)
  (k_thrust 0.000060))

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

(defblock airframe monoplane_airframe
  (use standard_airframe)
  (parasite_cda 0.03))

(defblock aero_surfaces unlimited_3d_monoplane
  (wing_span 1.30m)
  (wing_chord 0.26m)
  (wing_area 0.338)
  (prop_radius 0.2032m)
  (prop_z 0.55m)
  (tail_arm 0.55m)
  (stab_area 0.060)
  (fin_area 0.045)
  (k_thrust 0.0004520)
  (k_drag 0.000004060)
  (cl_alpha 4.2)
  (cd0 0.035)
  (cd_induced 0.085)
  (elevator_limit 45.0deg)
  (aileron_limit 38.0deg)
  (rudder_limit 48.0deg)
  (k_pitch 0.0000550)
  (k_roll 0.0000400)
  (k_yaw 0.0000500)
  (propwash_share 0.85)
  (inflow_floor 0.35))

(defblock drive_load aerobat_prop_load
  (use swashplate_tail_load)
  (main_base 0.40)
  (tail_base 0.32)
  (tail_pitch 1.15)
  (tail_ceiling 1.15))

(defaerodynamics shared
  (use standard_airframe))
