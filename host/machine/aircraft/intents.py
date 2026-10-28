"""The quad's flights as text (`machine.aircraft.aerolisp`): what a moment asks, and for how long.

    payloads = aerolisp.compiled(intents.POWER_LOOP, symbols)
    for payload, seconds in zip(payloads, intents.POWER_LOOP_S): ...

A stream says nothing of when: its seconds are its injector's.
"""

#: A hover, a dive forward weaving, a turn over the nose asked faster than any frame turns, a
#: catch on its back edge at all the rotors give: the streams as they were written.
POWER_LOOP = """
(stream_inject
  (sequence_id 201)
  (blend_duration 0ms)
  (latent_targets
    ;; Absolute orthogonal modes
    (mode_collective_thrust 9.81)   ; Exactly counteracting gravity (m/s^2)
    (mode_attitude_pitch 0.0deg)
    (mode_attitude_roll  0.0deg)
    (mode_yaw_rate       0.0rad_s)
  )
  (impedance
    ;; Brutal rotational stiffness. The quad fights the wind like a rock.
    (stiffness mode_attitude_pitch 5000)
    (stiffness mode_attitude_roll  5000)
    (damping   global 80) ; Critical damping to kill frame oscillations
  )
  (task_constraints
    (maintain_altitude 1.5m)
  )
)

(stream_inject
  (sequence_id 202)
  (blend_duration 400ms) ; Smooth transition to prevent prop cavitation
  (latent_targets
    (mode_collective_thrust 18.5)   ; Massive thrust vector increase
    (mode_attitude_pitch   -45.0deg) ; Aggressive forward carve
  )
  (impedance
    ;; Drop roll stiffness. The quad acts like a fluid pendulum laterally.
    (stiffness mode_attitude_roll 1500)
    (damping   mode_attitude_roll 25)
  )
  (cpg_config
    ;; Use the oscillator to generate the slalom weaving!
    ;; No if-statements, just a sine wave in the roll axis.
    (enable true)
    (frequency 2.0Hz)
    (amplitude mode_attitude_roll 35.0deg)
  )
)

(stream_inject
  (sequence_id 203)
  (blend_duration 100ms)
  (cpg_config
    (enable false) ; Kill the slalom oscillator
  )
  (latent_targets
    ;; Switch intent from attitude lock to rate override
    (mode_collective_thrust 2.0)     ; Float through the apex, near zero-G
    (mode_attitude_pitch    ignore)  ; Drop absolute angle tracking
    (mode_pitch_rate       -314.0rad_s) ; Approx 50 Hz backward rotation rate
  )
  (impedance
    (stiffness mode_pitch_rate 4000) ; Lock the rotational velocity strictly
  )
)

(stream_inject
  (sequence_id 204)
  (blend_duration 50ms) ; Lightning fast snap
  (latent_targets
    (mode_attitude_pitch  80.0deg)  ; Pitch back to expose the belly to the wind
    (mode_pitch_rate      0.0rad_s) ; Kill rotation instantly
    (mode_collective_thrust MAX_SYS_THRUST) ; Blast the motors to halt velocity
  )
  (impedance
    (stiffness mode_attitude_pitch 8000) ; Maximum stiffness to hold the airbrake
    (damping   mode_attitude_pitch 100)  ; Overdamped to prevent bounceback
  )
  (task_constraints
    ;; The active-set solver will automatically clip the max thrust
    ;; if the battery voltage starts sagging under the load.
    (limit_vertical_ascent 2.5m)
  )
)
"""

#: Each of its streams' seconds, from the floor: the hover's rise to its height, the dive, the
#: turn over the nose - 0.05 to 0.12 s of it all end in the catch -, the catch.
POWER_LOOP_S = (3.0, 1.5, 0.08, 0.45)
