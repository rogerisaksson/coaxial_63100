# %% [markdown]
# # The switches and the phase current, live
#
# A rolling window that redraws as records arrive: the six gate signals as
# duties over each record's window, and the phase currents scaled by the
# board's own calibration.
#
#     python python_examples/daq_live_plot.py --simulated --seconds 5
#
# **Why duties and not levels.** A pin in a record is the fraction of the
# window it was high, because a level sampled once and decimated is
# aliased by construction - KEEPALIVE toggles at ~100 kHz and read as a
# coin toss. At 50 kHz PWM a gate's duty IS the modulation, which is the
# thing worth plotting.
#
# pandas and matplotlib are not dependencies of `coaxial`; this file wants
# both and says so rather than failing three frames down.

# %%
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))

from coaxial import Coaxial63100                            # noqa: E402

try:
    import pandas as pd
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
p.add_argument('--save', default='', help='write the last frame here')
args = p.parse_args()

GATES = ['TIM1_CH1N/PWMUL', 'TIM1_CH1/PWMUH',
         'TIM1_CH2N/PWMVL', 'TIM1_CH2/PWMVH',
         'TIM1_CH3N/PWMWL', 'TIM1_CH3/PWMWH']

# %% [markdown]
# ## One task, two axes
# `configure_buffer` sizes the circular buffer in records. Naming a pin
# turns the whole sampled group on, so the gates ride the same records as
# the phases and every point on both axes is the SAME window - which is
# the reason to plot them together rather than in two runs.

# %%
pyplot.ion()
figure, (amps_ax, gates_ax) = pyplot.subplots(2, 1, sharex=True,
                                              figsize=(9, 6))

with Coaxial63100(port=args.port, power_afe=True,
                  simulated_device=args.simulated) as device:
    device.set_time_from_pc()
    daq = device.daq
    daq.configure_buffer(20000)
    daq.configure('phaseU', 'phaseV', 'phaseW', 'AFE_ON', sample_rate=500)
    scale = device.board.analog.scaling()
    phases = ['Phase U', 'Phase V', 'Phase W']

    held = None
    with daq:
        began = time.time()
        while time.time() - began < args.seconds:
            rec = daq.read(-1)              # blocks for the first, takes all
            block = daq.frame(rec)
            held = block if held is None else pd.concat([held, block])
            # KEEP A WINDOW, not the run: a live plot that grows without
            # bound slows down until it is the bottleneck, and the board
            # then fills the ring waiting for it.
            held = held[held.index > held.index[-1] -
                        pd.Timedelta(seconds=args.window)]

            for name in phases:
                held[name + ' A'] = held[name].map(scale['phase'].amps)

            amps_ax.clear()
            gates_ax.clear()
            elapsed = (held.index - held.index[0]).total_seconds()
            for name in phases:
                amps_ax.plot(elapsed, held[name + ' A'], label=name)
            for gate in GATES:
                if gate in held.columns:
                    gates_ax.plot(elapsed, held[gate],
                                  label=gate.split('/')[-1])
            amps_ax.set_ylabel('A')
            amps_ax.legend(loc='upper right', fontsize='small')
            gates_ax.set_ylabel('duty')
            gates_ax.set_ylim(-0.05, 1.05)
            gates_ax.set_xlabel('s')
            gates_ax.legend(loc='upper right', ncol=3, fontsize='small')
            figure.canvas.draw_idle()
            pyplot.pause(0.001)

    b = daq.buffered

print('%d records on screen, %.2f s window' % (len(held), args.window))
print('host peak %d, dropped %d, lost %d' % (b['peak'], b['dropped'],
                                             b['lost']))
print(held[[c for c in held.columns if c.endswith(' A')]]
      .describe().loc[['mean', 'std']].to_string(float_format='%.3f'))

if args.save:
    figure.savefig(args.save, dpi=90)
    print('wrote', args.save)
