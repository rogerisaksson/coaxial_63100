# %% [markdown]
# # coaxial 63100 - the rotor observer on the model
# `# %%` cells: opens as a notebook, runs as a script.
# The drive (0x6E device 10) runs on the board at the PWM rate. With the
# model as its source it needs no motor, no front end and no stage: the
# board integrates a PMSM itself and the observer is watched against a
# rotor whose angle is known. Nothing switches here - MOE stays clear.

# %%
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))          # the library lives here

from coaxial import Coaxial63100

SIMULATED = False
MOTOR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'host', 'motors', 'outrunner_14p.json')

device = Coaxial63100(port='COM4', simulated_device=SIMULATED, power_afe=False)
drive = device.drive
drive.open()
print(device)

# %% [markdown]
# ## The motor, and the model of it
# A profile is the record's drive parameters and the model's, in SI. The
# placeholder ships with the tree; a commissioned motor writes its own.

# %%
drive.off()
drive.source('model')
print(drive.profile(MOTOR)['name'])
params = drive.params()
print('R %.3f ohm  Ld %.1f uH  Lq %.1f uH  lambda %.4f V.s  %d pole pairs'
      % (params['motor_r_uohm'], params['motor_ld_nh'] * 1e6,
         params['motor_lq_nh'] * 1e6, params['motor_lambda_uvs'],
         params['motor_pole_pairs']))

# %% [markdown]
# ## The gains, from the numbers
# A 500 Hz current loop by pole-zero cancellation, a 60 Hz PLL, 2 V of
# fs/2 injection with its demodulated gain from Ld and Lq. The real
# commissioning derives these from the measured noise
# (`coaxial.sensorless`); this is the shape of it.

# %%
ts = drive.state()['ts']
v_inj, bw_i, f_pll = 2.0, 500.0, 60.0
ld, lq = params['motor_ld_nh'], params['motor_lq_nh']
drive.set_params(
    drv_kp_mv_per_a=ld * 2 * math.pi * bw_i,
    drv_ki_v_per_as=params['motor_r_uohm'] * 2 * math.pi * bw_i,
    drv_l1_milli=2 * 0.7 * 2 * math.pi * f_pll * 2 * ts,
    drv_l2_milli=(2 * math.pi * f_pll) ** 2 * 2 * ts,
    drv_inj_mv=v_inj, drv_inj_periods=1,
    drv_eps_gain_ua_per_rad=v_inj * ts * (lq - ld) / (ld * lq))

# %% [markdown]
# ## Lock at standstill
# The rotor sits at theta0; the estimate starts 0.3 rad away and the
# injection pulls it in. `model()` carries the estimate beside the truth
# in one reply, so the error means something.

# %%
drive.setpoint(id_ref=0.0, iq_ref=0.0)
drive.set_theta(drive.model()['theta'] + 0.3)
drive.mode('sensorless')
time.sleep(0.3)
m = drive.model()
print('theta_hat %.3f  rotor %.3f  error %+.4f rad'
      % (m['theta_hat'], m['theta'], m['error']))

# %% [markdown]
# ## Torque, and the crossover
# Half an ampere of q current: the modelled rotor accelerates against its
# friction, the back-EMF error blends in past w_lo and the injection turns
# itself off past w_hi.

# %%
drive.setpoint(iq_ref=0.5)
for _ in range(5):
    time.sleep(0.2)
    m, s = drive.model(), drive.state()
    print('omega_hat %5.0f  rotor %5.0f rad/s   error %+.3f rad   injecting %s'
          % (m['omega_hat'], m['omega'], m['error'], s['injecting']))

# %% [markdown]
# ## What it cost, and what the window says
# Cycles per block of the virtual step, the whole interrupt's exit past
# the trigger, and the innovation's whiteness the verification judges by.

# %%
s, w = drive.state(), drive.window()
print('sample %d  step %d  advance %d cycles;  exit %.1f us of %.0f'
      % (s['cycles']['sample'], s['cycles']['step'], s['cycles']['advance'],
         (s['exit_ticks_max'] or 0) / 237.5, ts * 1e6))
print('innovation sd %.4f rad  rho1 %+.3f  i_peak %.2f A'
      % (w['fields']['eps']['sd'] or 0.0, w['rho'][0], w['i_peak']))

# %% [markdown]
# ## Down
# Mode off releases the compares; the source goes back to the converters.

# %%
drive.off()
drive.source('adc')
device.board.gate_drivers.disarm()
device.close()
print('off')
