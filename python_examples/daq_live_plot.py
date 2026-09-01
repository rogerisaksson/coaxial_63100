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
p.add_argument('--buffer', type=float, default=30.0,
               help='seconds kept behind the window, to pan back over')
p.add_argument('--save', default='', help='write the last frame here')
args = p.parse_args()

#: The board's own spelling, high side then low side. `catalogue()` lists
#: them; they are here as a table because the three legs are three axes.
LEGS = (('U', 'TIM1_CH1/PWMUH', 'TIM1_CH1N/PWMUL'),
        ('V', 'TIM1_CH2/PWMVH', 'TIM1_CH2N/PWMVL'),
        ('W', 'TIM1_CH3/PWMWH', 'TIM1_CH3N/PWMWL'))
PHASES = ['Phase U (A)', 'Phase V (A)', 'Phase W (A)']

#: A colour each, so a leg is told from its neighbours without reading the
#: legend. The switches are black and thin on purpose: they are the
#: modulation that produced the currents, and they should not compete with
#: them for the eye.
PHASE_COLOURS = ('tab:red', 'tab:green', 'tab:blue')

# %% [markdown]
# ## The graph buffers too
# The artists are made ONCE and fed with `set_data`. Clearing an axis and
# replotting costs 39.3 ms a frame against 23.6 for the same three lines,
# measured: 25 fps against 42.
#
# `--buffer` seconds are held; `--window` seconds are shown. The lines
# carry the whole buffer, so pausing and panning back over it costs
# nothing and needs no second run.

# %%
pyplot.ion()
# TWO THIRDS THE CURRENTS, ONE THIRD THE SWITCHES: 6 against 1+1+1.
figure, axes = pyplot.subplots(4, 1, sharex=True, figsize=(9, 8),
                               gridspec_kw={'height_ratios': [6, 1, 1, 1]})
amps_ax, leg_axes = axes[0], axes[1:]

amps_lines = [amps_ax.plot([], [], color=colour, label=name)[0]
              for name, colour in zip(PHASES, PHASE_COLOURS)]
amps_ax.set_ylabel('A')
amps_ax.legend(loc='upper right', fontsize='small')
amps_ax.grid(alpha=0.2)

leg_lines = []
for axis, (leg, _high, _low) in zip(leg_axes, LEGS):
    leg_lines.append((
        axis.plot([], [], '-', color='black', linewidth=0.8, label='HS')[0],
        axis.plot([], [], '--', color='black', linewidth=0.8,
                  label='LS')[0]))
    axis.set_ylabel(leg, rotation=0, labelpad=10, fontsize='small')
    axis.set_ylim(-0.05, 1.05)
    axis.set_yticks([0, 1])
    axis.tick_params(labelsize='x-small')
    axis.legend(loc='upper right', ncol=2, fontsize='x-small',
                framealpha=0.6)
leg_axes[-1].set_xlabel('seconds before now')

# THE AXIS DOES NOT MOVE, THE DATA DOES. Time is measured back from the
# newest sample, so the right edge is always 0 and a record slides left
# until it leaves at -window. Sliding the limits instead makes the ticks
# crawl and the eye follow them rather than the trace.
for axis in axes:
    axis.set_xlim(-args.window, 0.0)

with Coaxial63100(port=args.port, power_afe=True,
                  simulated_device=args.simulated) as device:
    device.set_time_from_pc()
    daq = device.daq
    daq.configure_buffer(20000)
    daq.configure('phaseU', 'phaseV', 'phaseW', 'AFE_ON', sample_rate=500)

    drawn, began = 0, __import__('time').perf_counter()
    with daq:
        # `window` here is the BUFFER - the library keeps that many seconds
        # of records and builds each frame from them. What is on screen is
        # a slice of it, set below, so the data behind the view is already
        # in the lines.
        for df in daq.frames(window=args.buffer, seconds=args.seconds,
                             scaled=True):
            # Seconds BEFORE NOW: newest at 0, older to the left.
            elapsed = (df.index - df.index[-1]).total_seconds()

            # THE LINES CARRY THE WINDOW, THE FRAME CARRIES THE BUFFER.
            # Not for speed at this rate: measured, a redraw of nine lines
            # is 25 ms at 500 points and 25 at 2000, and this loop is
            # data-paced at about 8 fps anyway - `read(-1)` blocks for
            # records, so drawing is not what limits it. It is insurance
            # with a measured shape: 29 ms at 10 000 points and 51 at
            # 50 000, which a deep buffer reaches (50 000 is five minutes
            # at this record rate) and which would then be the bottleneck
            # that fills the board's ring. Sliced, the redraw is the
            # window's size whatever is held behind it, and panning back
            # is a re-slice of `df` the live loop never pays for.
            shown = elapsed > -args.window
            view = elapsed[shown]

            for line, name in zip(amps_lines, PHASES):
                line.set_data(view, df[name][shown])
            for (high_line, low_line), (_leg, high, low) in zip(leg_lines,
                                                                LEGS):
                if high in df.columns:
                    high_line.set_data(view, df[high][shown])
                    low_line.set_data(view, df[low][shown])

            amps_ax.relim()
            amps_ax.autoscale_view(scalex=False)
            figure.canvas.draw_idle()
            pyplot.pause(0.001)
            drawn += 1

    buffered = daq.buffered

spent = __import__('time').perf_counter() - began
print('%d records buffered (%.0f s), %.2f s of it on screen'
      % (len(df), args.buffer, args.window))
print('%d frames in %.1f s = %.1f fps' % (drawn, spent, drawn / spent))
print('host peak %d, dropped %d, lost %d'
      % (buffered['peak'], buffered['dropped'], buffered['lost']))
print(df[PHASES].describe().loc[['mean', 'std', 'min', 'max']]
      .to_string(float_format='%.3f'))

if args.save:
    figure.savefig(args.save, dpi=90)
    print('wrote', args.save)
