# %% [markdown]
# # A run into pandas
#
# What a notebook wants from the board: a DataFrame with a time index and
# one column per channel, scaled by the board's own calibration record.
#
#     python python_examples/daq_pandas.py --simulated
#
# **pandas is not a dependency of `coaxial`.** `daq.frame()` imports it
# where it is called and refuses with the install line when it is absent,
# so a bench that only reads a thermistor never has to have it. This file
# does want it, and says so rather than failing three frames down.

# %%
import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))

from coaxial import Coaxial63100                            # noqa: E402

try:
    import pandas                                           # noqa: F401
except ImportError:
    print('this example needs pandas: pip install pandas')
    raise SystemExit(0)

p = argparse.ArgumentParser()
p.add_argument('--port', default='COM4')
p.add_argument('--simulated', action='store_true')
p.add_argument('--records', type=int, default=200)
args = p.parse_args()

# %% [markdown]
# ## Take a run
# `configure_buffer` sizes the circular buffer in RECORDS - the broker's
# ring when one owns the port, so every other client reads the same
# records from its own cursor.

# %%
with Coaxial63100(port=args.port, power_afe=True,
                  simulated_device=args.simulated) as device:
    device.set_time_from_pc()
    daq = device.daq

    daq.configure_buffer(10000)
    daq.configure('phaseU', 'phaseV', 'ntc', 'DC bus', sample_rate=500)

    with daq:                              # start, and stop however it ends
        rec = daq.read(args.records)

    df = daq.frame(rec)                    # time index, a column per channel
    scale = device.board.analog.scaling()  # the BOARD's calibration

print('%d records, %d columns, index %s' % (df.shape[0], df.shape[1],
                                            df.index.name))
print(df.head(4).to_string(float_format='%.1f'))

# %% [markdown]
# ## Scale it
# Codes are what the board sends. Every conversion lives in the
# calibration record (invariant 7), so the numbers below come from the
# board and not from a constant in this file.

# %%
df['NTC degC'] = df['NTC'].map(scale['ntc'].celsius)
df['U amps'] = df['Phase U'].map(scale['phase'].amps)
df['DC V'] = df['DC bus'].map(scale['dcbus'].volts)

print(df[['NTC degC', 'U amps', 'DC V']]
      .resample('100ms').mean().head(5).to_string(float_format='%.3f'))

# %% [markdown]
# ## What the clock actually did
# `dt` is measured, from the gap to the next record - not the rate that
# was asked for. Its spread is what says whether the loop kept up.

# %%
print('dt: mean %.6f s, std %.2e, min %.6f, max %.6f'
      % (df['dt'].mean(), df['dt'].std(), df['dt'].min(), df['dt'].max()))

# %% [markdown]
# ## And a picture
# `daq.plot()` is one line to matplotlib; anything more is pandas' own.

# %%
try:
    axes = df[['NTC degC']].plot(title='NTC')
    axes.figure.savefig('ntc.png', dpi=90)
    print('wrote ntc.png')
except ImportError:
    print('no matplotlib - the DataFrame above is all of it')
