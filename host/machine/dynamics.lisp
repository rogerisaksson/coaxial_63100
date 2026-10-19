; General rigid-body, cooling, governor, impedance, and observer blocks (`dynamics.lisp`).
;
; Both the quadcopter and the 3D helicopter mount their Coaxial 63100 power stages on finned
; aluminium heatsink blocks that conduct heat from the power switches and current shunts, cooled
; by a temperature-controlled fan (`finned_aluminium_fan_sink`).

(defblock rigid_body base_airframe_2kg
  (mass 2.0kg)
  (gravity 9.81m_s2)
  (inertia_xx 0.045)
  (inertia_yy 0.090)
  (inertia_zz 0.045)
  (skid_height 0.12m)
  (foot_radius 0.015m))

(defblock rigid_body quad_x_body
  (use base_airframe_2kg)
  (hub_x 0.06m)
  (hub_y 0.03m)
  (hub_z 0.06m))

(defblock rigid_body heli_pod_boom_body
  (use base_airframe_2kg)
  (inertia_xx 0.022)
  (inertia_yy 0.030)
  (inertia_zz 0.015)
  (skid_span 0.135m)
  (strut_aft -0.08m)
  (strut_fore 0.13m))

(defblock rigid_body ornithopter_carapace_body
  (use heli_pod_boom_body)
  (inertia_xx 0.024)
  (inertia_yy 0.032)
  (inertia_zz 0.018)
  (skid_height 0.135m)
  (skid_span 0.130m)
  (strut_aft -0.09m)
  (strut_fore 0.14m))

(defblock rigid_body aerobat_monoplane_body
  (use heli_pod_boom_body)
  (inertia_xx 0.090)
  (inertia_yy 0.060)
  (inertia_zz 0.120)
  (skid_height 0.30m)
  (skid_span 0.15m)
  (strut_aft -0.52m)
  (strut_fore 0.09m))

; Aluminium block with cooling fins conducting heat from the power switches (MOSFETs) and
; current shunts, plus a temperature-controlled cooling fan block, puzzled together.
(defblock cooling aluminium_finned_block
  (sink_mass 0.085kg)
  (sink_capacity 180.0)
  (r_switches_to_sink 3.0)
  (r_shunts_to_sink 1.5)
  (r_sink_to_air_natural 2.5)
  (r_sink_to_air_forced 0.25))

(defblock cooling temp_controlled_fan
  (fan_temp_on 45.0)
  (fan_temp_full 80.0)
  (fan_idle_duty 0.0)
  (fan_max_airflow 14.0m_s)
  (fan_max_rpm 8000.0))

(defblock cooling finned_aluminium_fan_sink
  (use aluminium_finned_block)
  (use temp_controlled_fan))

(defblock governor base_governor
  (idle_speed 86.0rad_s)
  (step_time 0.002s))

(defblock governor quad_direct_governor
  (use base_governor)
  (spin_time 0.08s))

(defblock governor heli_3d_governor
  (use base_governor)
  (main_speed 360.0rad_s)
  (tail_speed 340.0rad_s)
  (stunt_boost 1.12)
  (stunt_yaw_threshold 2.5rad_s)
  (tail_ratio 0.88))

(defblock governor ornithopter_governor
  (use heli_3d_governor)
  (main_speed 365.0rad_s)
  (tail_speed 355.0rad_s)
  (stunt_boost 1.14)
  (tail_ratio 0.94))

(defblock governor aerobat_3d_governor
  (use heli_3d_governor)
  (main_speed 365.0rad_s)
  (tail_speed 345.0rad_s)
  (stunt_boost 1.14)
  (tail_ratio 0.90))

(defblock impedance base_attitude_impedance
  (stiffness attitude_pitch 260.0)
  (damping attitude_pitch 20.0)
  (stiffness attitude_roll 260.0)
  (damping attitude_roll 20.0)
  (damping attitude_yaw 10.0))

(defblock impedance quad_attitude_impedance
  (use base_attitude_impedance)
  (stiffness attitude_pitch 300.0)
  (damping attitude_pitch 17.0)
  (stiffness attitude_roll 300.0)
  (damping attitude_roll 17.0)
  (damping attitude_yaw 4.0)
  (stiffness pitch_rate 201.0)
  (damping pitch_rate 17.0)
  (stiffness roll_rate 201.0)
  (damping roll_rate 17.0))

(defblock impedance heli_3d_impedance
  (use base_attitude_impedance)
  (stiffness attitude_pitch 260.0)
  (damping attitude_pitch 22.0)
  (stiffness attitude_roll 260.0)
  (damping attitude_roll 22.0)
  (stiffness attitude_yaw 140.0)
  (damping attitude_yaw 16.0))

(defblock impedance ornithopter_impedance
  (use heli_3d_impedance)
  (stiffness attitude_pitch 265.0)
  (stiffness attitude_roll 265.0)
  (stiffness attitude_yaw 145.0))

(defblock impedance aerobat_3d_impedance
  (use heli_3d_impedance)
  (stiffness attitude_pitch 270.0)
  (stiffness attitude_roll 275.0)
  (stiffness attitude_yaw 150.0))

(defblock observers eso_3d_observers
  (turn_bandwidth 35.0rad_s)
  (pull_bandwidth 10.0rad_s)
  (wind_bandwidth 5.0rad_s)
  (wind_regularization 0.04)
  (min_air_speed 0.2m_s))

(defdynamics shared
  (use base_airframe_2kg)
  (use finned_aluminium_fan_sink)
  (use base_governor)
  (use base_attitude_impedance)
  (use eso_3d_observers))
