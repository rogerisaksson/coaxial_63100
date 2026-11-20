"""Wave-drive gearbox, brushless outrunner, switching-decimation, and elastic-cable solvers.

    wb = drivetrain.wave_box(diameter_m, efficiency, g_gpa, allow_mpa, rho_kg_m3, min_wall_m)
    kv = drivetrain.kv_from_flux(pole_pairs, flux_vs)
    tau = drivetrain.peak_torque(pole_pairs, flux_vs, amps)
    j = drivetrain.outrunner_inertia(mass_kg, can_m, ring_share, kv, kv_ref)
    loop_hz, step_s = drivetrain.control_rates(pwm_hz)
    wave, twist = drivetrain.cable_waves(radius_m, kg_m, ei_nm2, gj_nm2)

Pure functions of geometry, kinematics, and material constants (`math` alone): a motor's or a
gearbox's datasheet states outer dimensions, ratio, efficiency, backlash, and electromagnetic
windings, while the static compiler derives torsional compliance, shock capacity, preload drag,
carrier inertia, and explicit cable wave speeds through these laws.
"""
import math
from typing import NamedTuple

#: Hollow roller-race bore share `D_bore / D` of a pancake wave-drive housing (`1 - 0.7^2 = 0.51`).
BORE_SHARE = 0.70
#: Two-lobe Tresca shear root factor `(1 / 8)^3 = 1 / 512` and preload radial strain `1e-5 / 0.01`.
LOBE_SHEAR_INV = 512.0
#: FOC current-loop decimation from PWM carrier (`pwm / 20`) and mechanical step decimation (`loop / 5`).
FOC_DECIMATION, STEP_DECIMATION = 20.0, 5.0
#: Clamped-clamped Euler-Bernoulli fundamental mode factor `(beta_1 L)^2 = 4.73004^2`.
FIRST_CLAMPED_MODE = 22.37


class WaveBox(NamedTuple):
    """Derived physical properties of a one-stage steel roller wave-drive gearbox."""
    length_share: float
    fill_share: float
    mass_kg: float
    stiffness_nm_rad: float
    shock_nm: float
    drag_nm: float
    input_j_share: float
    zeta: float
    friction_share: float
    mesh_rad_s: float


def wave_box(diameter_m, efficiency, g_gpa, allow_mpa, rho_kg_m3, min_wall_m):
    """Solve `WaveBox` mechanics from outer `diameter_m`, `efficiency`, and steel properties."""
    d, eff = float(diameter_m), float(efficiency)
    g_pa, allow_pa = float(g_gpa) * 1e9, float(allow_mpa) * 1e6
    d2 = d * d
    d3 = d2 * d
    fill = round(1.0 - BORE_SHARE * BORE_SHARE, 2)
    length_share = round(0.4 * BORE_SHARE, 2)
    mass_kg = fill * float(rho_kg_m3) * 0.25 * math.pi * length_share * d3
    r_b = 0.5 * BORE_SHARE
    rb3_over_l = (r_b * r_b * r_b) / length_share
    k_raw = (2.0 * math.pi / 35.0) * g_pa * float(min_wall_m) * rb3_over_l * d2
    stiffness = float(round(k_raw, -2))
    shock = round((allow_pa / LOBE_SHEAR_INV) * d3, 1)
    loss = 1.0 - eff
    drag = (loss * (g_pa * 1e-5) / LOBE_SHEAR_INV) * d3
    cam_d = 0.8 * BORE_SHARE
    cam_d2 = cam_d * cam_d
    input_j = round(0.5 * cam_d2 * cam_d2, 2)
    return WaveBox(
        length_share=length_share,
        fill_share=fill,
        mass_kg=mass_kg,
        stiffness_nm_rad=stiffness,
        shock_nm=shock,
        drag_nm=drag,
        input_j_share=input_j,
        zeta=round(3.0 * loss, 4),
        friction_share=round(0.2 * loss, 4),
        mesh_rad_s=round(0.2 * loss, 4),
    )


def kv_from_flux(pole_pairs, flux_vs):
    """Three-phase SVPWM line-to-line velocity constant, rpm/V, from `pole_pairs` and `flux_vs`."""
    return 30.0 / (math.pi * math.sqrt(3.0) * float(pole_pairs) * float(flux_vs))


def peak_torque(pole_pairs, flux_vs, amps):
    """Three-phase PMSM electromagnetic torque `1.5 * p * lambda * I_q`, N m."""
    return 1.5 * float(pole_pairs) * float(flux_vs) * float(amps)


def noload_rad_s(volts, pole_pairs, flux_vs):
    """Unloaded SVPWM mechanical ceiling speed `V_dc / (sqrt(3) * p * lambda)`, rad/s."""
    return float(volts) / (math.sqrt(3.0) * float(pole_pairs) * float(flux_vs))


def outrunner_inertia(mass_kg, can_m, ring_share, kv=None, kv_ref=None):
    """Outrunner rotor inertia, kg m^2, from can diameter, mass, and pole-ring scaling."""
    r = 0.5 * float(can_m)
    scale = 1.0
    if kv is not None and kv_ref is not None and float(kv) > float(kv_ref):
        scale = (float(kv_ref) / float(kv)) ** (1.0 / 6.0)
    j = float(ring_share) * scale * float(mass_kg) * r * r
    return round(j, -int(math.floor(math.log10(j))) + 2)


def control_rates(pwm_hz):
    """`(current_loop_hz, world_step_s)` decimated from inverter switching frequency `pwm_hz`."""
    loop_hz = float(pwm_hz) / FOC_DECIMATION
    return loop_hz, round(STEP_DECIMATION / loop_hz, 6)


def joint_viscous(min_efficiency):
    """Passive drivetrain viscous joint damping, N m s/rad, from gearbox mechanical loss."""
    return round(3.0 * (1.0 - float(min_efficiency)), 4)


def cable_waves(radius_m, kg_m, ei_nm2, gj_nm2):
    """`(bending_wave_m2_s, torsion_wave_m_s)` Courant step scales for an elastic cable."""
    r, kgm = float(radius_m), float(kg_m)
    wave = 0.5 * math.pi * math.pi * math.sqrt(float(ei_nm2) / kgm)
    twist = 0.5 * math.pi * math.sqrt(2.0 * float(gj_nm2) / kgm) / r
    return wave, twist


def cable_damping(length_m, link_m, kg_m, ei_nm2, zeta):
    """`(first_rad_s, modal_nms, critical_nms)` Kelvin-Voigt joint damping of a clamped cable."""
    L, ell, kgm = float(length_m), float(link_m), float(kg_m)
    first = FIRST_CLAMPED_MODE / (L * L) * math.sqrt(float(ei_nm2) / kgm)
    crit = 2.0 * float(ei_nm2) / (ell * first)
    return first, float(zeta) * crit, crit
