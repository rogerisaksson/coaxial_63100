# %% [markdown]
# # The phase current and the gate switches, live
#
# Two thirds the phase currents, one third the switches: one graph per leg
# with its high side and low side, all on one time base. Every point on
# every axis comes from the SAME record, so a current and the switching
# that produced it line up by construction rather than by two runs being
# started close together.
#
#     python python_examples/daq_live_plot.py --simulated --seconds 5
#
# **Why duties and not levels.** A pin in a record is the fraction of the
# window it was high, because a level sampled once and decimated is
# aliased by construction - KEEPALIVE toggles at ~100 kHz and read as a
# coin toss. At 50 kHz PWM a gate's duty IS the modulation.
#
# **The axis is fixed and the data scrolls.** `frames()` indexes on seconds
# before now, so the newest sample sits at 0 and older ones trail off to
# the left until they leave the window. Sliding the limits instead makes
# the ticks crawl and the eye follows them rather than the trace.
#
# `--buffer` seconds are held behind `--window` seconds on screen, to pan
# back over after pausing.

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
    print('this example needs pandas and matplotlib: '
          'pip install pandas matplotlib')
    raise SystemExit(0)

p = argparse.ArgumentParser()
p.add_argument('--port', default='COM4')
p.add_argument('--simulated', action='store_true')
p.add_argument('--seconds', type=float, default=10.0)
p.add_argument('--window', type=float, default=2.0, help='seconds on screen')
p.add_argument('--buffer', type=float, default=30.0,
               help='seconds kept behind the window, to pan back over')
p.add_argument('--save', default='', help='write the last frame here')
args = p.parse_args()

# %%
pyplot.ion()
figure, axes = pyplot.subplots(4, 1, sharex=True, figsize=(9, 8),
                               gridspec_kw={'height_ratios': [6, 1, 1, 1]})
amps_ax, leg_axes = axes[0], axes[1:]

with Coaxial63100(port=args.port, power_afe=True,
                  simulated_device=args.simulated) as device:
    device.set_time_from_pc()
    daq = device.daq
    daq.configure_buffer(20000)
    daq.configure('phaseU', 'phaseV', 'phaseW', 'AFE_ON', sample_rate=500)

    amps, legs = daq.currents(), daq.legs()   # both off the board

    with daq:
        for df in daq.frames(window=args.window, buffer=args.buffer,
                             seconds=args.seconds, scaled=True):
            for axis in axes:
                axis.clear()

            df[amps].plot(ax=amps_ax,
                          color=['tab:red', 'tab:green', 'tab:blue'])
            amps_ax.set(ylabel='A', xlim=(-args.window, 0))

            for axis, (leg, high, low) in zip(leg_axes, legs):
                df[[high, low]].plot(ax=axis, legend=False, color='black',
                                     style=['-', '--'], linewidth=0.8)
                axis.set(ylabel=leg, ylim=(-0.05, 1.05), yticks=[0, 1])

            leg_axes[-1].set_xlabel('seconds before now')
            figure.canvas.draw_idle()
            pyplot.pause(0.001)

    held = daq.history(scaled=True)      # the buffer behind the window
    buffered = daq.buffered

print('%d records on screen (%.1f s), %d buffered behind them (%.0f s)'
      % (len(df), args.window, len(held), args.buffer))
print('host peak %d, dropped %d, lost %d'
      % (buffered['peak'], buffered['dropped'], buffered['lost']))
print(held[amps].describe().loc[['mean', 'std', 'min', 'max']]
      .to_string(float_format='%.3f'))

if args.save:
    figure.savefig(args.save, dpi=90)
    print('wrote', args.save)
