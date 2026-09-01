# %% [markdown]
# # The phase current and the gate switches, live
#
# Four axes on one time base: the three phase currents on top, then one
# graph per leg with its high side and low side. Plain pandas and plain
# matplotlib - the only thing the library does here is hand over a rolling
# window of records already in a DataFrame.
#
#     python python_examples/daq_live_plot.py --simulated --seconds 5
#
# **Why duties and not levels.** A pin in a record is the fraction of the
# window it was high, because a level sampled once and decimated is
# aliased by construction - KEEPALIVE toggles at ~100 kHz and read as a
# coin toss. At 50 kHz PWM a gate's duty IS the modulation.
#
# Every point on every axis comes from the SAME record, so a current and
# the switching that produced it line up by construction rather than by
# two runs being started close together.

# %%
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))

from coaxial import Coaxial63100                            # noqa: E402

try:
    from matplotlib import pyplot
except ImportError:
    print('this example needs matplotlib: pip install matplotlib')
    raise SystemExit(0)

p = argparse.ArgumentParser()
p.add_argument('--port', default='COM4')
p.add_argument('--simulated', action='store_true')
p.add_argument('--seconds', type=float, default=10.0)
p.add_argument('--window', type=float, default=2.0, help='seconds on screen')
p.add_argument('--save', default='', help='write the last frame here')
args = p.parse_args()

#: The board's own spelling, high side then low side. `catalogue()` lists
#: them; they are here as a table because the three legs are three axes.
LEGS = (('U', 'TIM1_CH1/PWMUH', 'TIM1_CH1N/PWMUL'),
        ('V', 'TIM1_CH2/PWMVH', 'TIM1_CH2N/PWMVL'),
        ('W', 'TIM1_CH3/PWMWH', 'TIM1_CH3N/PWMWL'))
PHASES = ['Phase U (A)', 'Phase V (A)', 'Phase W (A)']

# %%
pyplot.ion()
figure, axes = pyplot.subplots(4, 1, sharex=True, figsize=(9, 8),
                               gridspec_kw={'height_ratios': [3, 1, 1, 1]})
amps_ax, leg_axes = axes[0], axes[1:]

with Coaxial63100(port=args.port, power_afe=True,
                  simulated_device=args.simulated) as device:
    device.set_time_from_pc()
    daq = device.daq
    daq.configure_buffer(20000)
    daq.configure('phaseU', 'phaseV', 'phaseW', 'AFE_ON', sample_rate=500)

    with daq:
        # The rolling window is the library's: it keeps the last `window`
        # seconds of RECORDS and builds each frame from them, so nothing
        # is concatenated and nothing grows. `scaled` adds the real-unit
        # columns from the board's own calibration beside the codes.
        for df in daq.frames(window=args.window, seconds=args.seconds,
                             scaled=True):
            elapsed = (df.index - df.index[0]).total_seconds()
            plain = df.set_index(elapsed)

            amps_ax.clear()
            plain[PHASES].plot(ax=amps_ax, legend=True)
            amps_ax.set_ylabel('A')
            amps_ax.legend(loc='upper right', fontsize='small')

            for axis, (leg, high, low) in zip(leg_axes, LEGS):
                axis.clear()
                if high in plain.columns:
                    plain[[high, low]].plot(
                        ax=axis, legend=False,
                        style=['-', '--'], color=['tab:red', 'tab:blue'])
                axis.set_ylabel('%s duty' % leg)
                axis.set_ylim(-0.05, 1.05)
                axis.legend(['HS', 'LS'], loc='upper right', ncol=2,
                            fontsize='small')
            leg_axes[-1].set_xlabel('s')
            figure.canvas.draw_idle()
            pyplot.pause(0.001)

    buffered = daq.buffered

print('%d records on screen, %.2f s window' % (len(df), args.window))
print('host peak %d, dropped %d, lost %d'
      % (buffered['peak'], buffered['dropped'], buffered['lost']))
print(df[PHASES].describe().loc[['mean', 'std', 'min', 'max']]
      .to_string(float_format='%.3f'))

if args.save:
    figure.savefig(args.save, dpi=90)
    print('wrote', args.save)
