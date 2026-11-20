"""Blade-element momentum, flapping-wing, thin-airfoil flap, and slipstream solvers.

    cl_a = aero.lift_slope(span, chord)                              # Helmbold 3D lift slope, 1/rad
    e = aero.oswald(aspect)                                          # clean-wing Oswald span factor
    e_af = aero.airframe_oswald(span, chord, cd0)                    # viscous airframe span factor
    pw = aero.propwash_share(prop_d, wing_span, wing_chord, stab_s)  # Rankine-Froude wake immersion
    tau = aero.glauert_flap(chord_share)                             # Glauert plain-flap effectiveness
    j = aero.blade_inertia(diameter_m, rho_kg_m3, wall_m)            # hollow shell prop inertia, kg m^2
    ct, cp = aero.propeller_bemt(diameter_m, pitch_m)                # fixed-pitch propeller polars
    ct, cp = aero.rotor_bemt(diameter_m, pitch_rad, gear_ratio)      # collective rotor polars at shaft
    ct, cp, cl = aero.flapping_polar(span_m, chord_m, ...)           # 4-wing flapping and glide polars
    kp, kr, ky = aero.control_moments(...)                           # slipstream and vector moments

Pure functions of geometry, kinematics, and material constants (`math` alone): a craft's specification
states physical dimensions, deflections, and gear ratios, and the static aerodynamic compiler derives
all nondimensional thrust, torque, slipstream, and moment polars through these laws.
"""
import math

#: Radial quadrature stations across the active blade span `[HUB_SHARE, 1.0]` and 2D viscous profile drag.
STATIONS, HUB_SHARE, SECTION_CD0 = 64, 0.15, 0.004
#: Larrabee minimum-induced-loss blade aspect factor `R / c = LARRABEE_AR * sqrt(P / D)`.
LARRABEE_AR = 7.5
#: Cambered thin-electric airfoil zero-lift advance slip over geometric pitch, and vortex-ring inflow floor.
ZERO_THRUST_SLIP, WAKE_INFLOW_FLOOR = 1.05, 0.35
#: Advancing-blade NACA 0012 transonic drag-divergence Mach ceiling.
TRANSONIC_DD_MACH = 0.73


def lift_slope(span, chord):
    """A wing's lift per radian of incidence from its aspect ratio (Helmbold's equation): thin-airfoil
    theory's 2 pi per radian in two dimensions, less what its trailing vortices' downwash takes of it."""
    aspect = float(span) / float(chord)
    return math.tau * aspect / (2.0 + math.sqrt(aspect * aspect + 4.0))


def oswald(aspect):
    """Clean tapered wing lifting-line Oswald span efficiency `1 / (1 + 0.02 AR)`."""
    return 1.0 / (1.0 + 0.02 * float(aspect))


def airframe_oswald(span, chord, cd0):
    """Whole-airframe span efficiency including Hoerner fuselage carry-over and viscous lift drag."""
    ar = float(span) / float(chord)
    return round(1.0 / (1.0 / oswald(ar) + 0.42 * math.pi * ar * float(cd0)), 2)


def _elliptic_immersion(eta):
    e = max(0.0, min(1.0, float(eta)))
    return 1.0 if e >= 1.0 else (2.0 / math.pi) * (math.asin(e) + e * math.sqrt(1.0 - e * e))


def propwash_share(prop_diameter, wing_span, wing_chord, stab_area):
    """Geometric-mean elliptic tail lift immersion between the actuator disc (`D_p`) and the
    Rankine-Froude contracted far wake (`D_p / sqrt(2)`) across horizontal tail span."""
    pd = float(prop_diameter)
    stab_b = math.sqrt(float(stab_area) * float(wing_span) / float(wing_chord))
    e_disc = _elliptic_immersion(pd / stab_b)
    e_wake = _elliptic_immersion(pd / (math.sqrt(2.0) * stab_b))
    return round(math.sqrt(e_disc * e_wake), 2)


def disc_half_height(diameter_m):
    """Collision cylinder axial half-thickness, m, of a propeller or rotor hub (`0.02 D`)."""
    return round(0.02 * float(diameter_m), 3)


def flapping_beat_pulse(stroke_rad, pitch_rad):
    """Second-harmonic thrust ripple amplitude of a symmetric flapping stroke `0.5 * Phi * cos(theta)`."""
    return round(0.5 * float(stroke_rad) * math.cos(float(pitch_rad)), 2)


def stunt_boost(flight_amps, wep_amps, tip_mach=0.0):
    """Governor WEP speed boost `(I_wep / I_flight)^(1/3)`, bounded by transonic Mach drag divergence."""
    boost = (float(wep_amps) / float(flight_amps)) ** (1.0 / 3.0)
    if float(tip_mach) > 0.0:
        boost = min(boost, TRANSONIC_DD_MACH / float(tip_mach))
    return round(boost, 2)


def glauert_flap(chord_share):
    """Glauert thin-airfoil plain-flap effectiveness `d(alpha_0) / d(delta)` for flap chord ratio `E`."""
    e = max(0.0, min(1.0, float(chord_share)))
    if e <= 0.0 or e >= 1.0:
        return e
    return 1.0 - (math.acos(2.0 * e - 1.0) - 2.0 * math.sqrt(e * (1.0 - e))) / math.pi


def blade_inertia(diameter_m, rho_kg_m3, wall_m, blades=2, chord_m=None, aspect=6.5):
    """Rotational inertia, kg m^2, of a tapered thin-shell propeller of `diameter_m` and `wall_m`."""
    r = 0.5 * float(diameter_m)
    chord = float(chord_m) if chord_m is not None else r / float(aspect)
    m_shell = float(blades) * (2.0 * chord * float(wall_m)) * r * float(rho_kg_m3)
    return 0.2 * m_shell * r * r


def _bemt_strip(blades, sigma, cl_a, cd0, theta_fn, elliptic):
    """Prandtl tip-loss hover blade-element momentum integral returning `(C_T, C_P)` in `n, D` units."""
    dx = (1.0 - HUB_SHARE) / STATIONS
    ct_s = cq_s = 0.0
    for i in range(STATIONS):
        x = HUB_SHARE + (i + 0.5) * dx
        sig_x = sigma * (4.0 / math.pi) * math.sqrt(max(0.0, 1.0 - x * x)) if elliptic else sigma
        th = theta_fn(x)
        s_phi = max(0.03, math.sin(0.25 * th))
        f_tip = max(0.05, (2.0 / math.pi) * math.acos(
            min(1.0, math.exp(-0.5 * blades * (1.0 - x) / (x * s_phi)))))
        b = sig_x * cl_a / (16.0 * f_tip)
        lam = math.sqrt(b * b + 2.0 * b * th * x) - b
        alpha = max(0.0, th - lam / x)
        cl = cl_a * math.sin(alpha) * math.cos(alpha)
        x2 = x * x
        ct_s += 0.5 * sig_x * cl * x2 * dx
        cq_s += 0.5 * sig_x * (cl * lam * x + cd0 * x2 * x) * dx
    pi2 = math.pi * math.pi
    return ct_s * pi2 * math.pi / 4.0, cq_s * pi2 * pi2 / 4.0


def propeller_bemt(diameter_m, pitch_m, blades=2, cd0=SECTION_CD0):
    """Fixed-pitch propeller `(C_T, C_P)` from `diameter_m` and Betz constant-helix `pitch_m`."""
    d, p = float(diameter_m), float(pitch_m)
    r = 0.5 * d
    chord = r / (LARRABEE_AR * math.sqrt(p / d))
    sigma = float(blades) * chord / (math.pi * r)
    ct, cp = _bemt_strip(float(blades), sigma, math.tau, float(cd0),
                         lambda x: math.atan(p / (math.pi * d * x)), True)
    return round(ct, 6), round(cp, 6)


def rotor_bemt(diameter_m, pitch_rad, gear_ratio=1.0, step_up=False, blades=2, chord_m=None,
               aspect=11.0, cd0=SECTION_CD0):
    """Collective-pitch symmetric rotor `(C_T, C_P)` referred to the motor shaft across `gear_ratio`."""
    d, th, g = float(diameter_m), float(pitch_rad), float(gear_ratio)
    r = 0.5 * d
    chord = float(chord_m) if chord_m is not None else r / float(aspect)
    sigma = float(blades) * chord / (math.pi * r)
    ct_h, cp_h = _bemt_strip(float(blades), sigma, lift_slope(d, chord), float(cd0), lambda _x: th, False)
    g2 = g * g
    if step_up:
        return round(ct_h * g2, 6), round(cp_h * g2 * g, 6)
    return round(ct_h / g2, 7), round(0.5 * cp_h / (g2 * g), 8)


def flapping_polar(span_m, chord_m, stroke_rad, pitch_rad, sweep_rad, gear_ratio):
    """Quasi-steady 4-wing `(ct_stroke, cp_stroke, cl_glide)` from Polhamus vortex lift and Ellington moments."""
    cl_a = lift_slope(span_m, chord_m)
    sa, ca = math.sin(float(pitch_rad)), math.cos(float(pitch_rad))
    cs = math.cos(float(sweep_rad))
    cl_flap = cl_a * sa * ca * ca + math.pi * sa * sa * ca
    cd_flap = cl_flap * (sa / max(1e-6, ca)) + 2.0 * SECTION_CD0
    ct_stroke = 0.25 * cl_flap
    cp_stroke = (cd_flap / 12.0) / float(gear_ratio)
    cl_glide = round(sa * cs * cs, 2)
    return round(ct_stroke, 6), round(cp_stroke, 6), cl_glide


def control_moments(wing_span, wing_chord, stab_area, fin_area, tail_arm, prop_diameter, prop_z,
                    elev_rad, ail_rad, rud_rad, propwash_share, flap_share, aileron_share,
                    vector_rad, weight_n, gov_rad_s, vect_gov_rad_s):
    """`(k_pitch, k_roll, k_yaw)` from Glauert control-surface flaps in Rankine-Froude contracted
    propwash (`A_wake = A_disc / 2`, core jet dynamic pressure `2 * q_disc`) plus thrust-vectoring
    deflection `vector_rad` across `(prop_z + tail_arm)`."""
    ws, wc, sa, fa, arm = (float(x) for x in (wing_span, wing_chord, stab_area, fin_area, tail_arm))
    pd, pz, pw = float(prop_diameter), float(prop_z), float(propwash_share)
    r_p = 0.5 * pd
    a_disc = math.pi * r_p * r_p
    stab_b = math.sqrt(sa * ws / wc)
    fin_h = math.sqrt(2.0 * fa)
    cl_stab = lift_slope(stab_b, sa / stab_b)
    cl_fin = lift_slope(2.0 * fin_h, fa / fin_h)
    cl_wing = lift_slope(ws, wc)
    tau_f, tau_a = glauert_flap(flap_share), glauert_flap(aileron_share)
    s_vec = math.sin(float(vector_rad))
    kp_q = 2.0 * sa * arm * pw * cl_stab * tau_f * math.sin(float(elev_rad)) + a_disc * pz * s_vec
    kr_q = 2.0 * (pd * wc) * r_p * pw * cl_wing * tau_a * math.sin(float(ail_rad))
    ky_q = 2.0 * fa * arm * pw * cl_fin * tau_f * math.sin(float(rud_rad)) + a_disc * (pz + arm) * s_vec
    q_hover = float(weight_n) / a_disc
    w_g2, w_v2 = float(gov_rad_s) * float(gov_rad_s), float(vect_gov_rad_s) * float(vect_gov_rad_s)
    return (round(kp_q * q_hover / w_g2, 8),
            round(kr_q * q_hover / w_g2, 8),
            round(ky_q * q_hover / w_v2, 8))
