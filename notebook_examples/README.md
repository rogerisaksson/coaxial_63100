# The notebooks

13 short papers on the board, each run as it stands: no board, no cable.
They are generated: `host/tools/notebooks/` is their source.

## Before the first one

1. `setup.ps1` once: the packages, the kernel `Python (coaxial_63100)`,
   the firmware image, Renode.
2. Open a paper in VS Code and pick that kernel.
3. Run All. `MODE` in its Setup says where it runs: `EMULATED` is this
   host's firmware image on Renode, and the host's own stand-in where no
   emulator runs.

## The papers, in reading order

| paper | what it shows | runs on |
| --- | --- | --- |
| [Acquisition](acquisition.ipynb) | A task configured, read in a loop, scaled into a frame, drawn live. | the image on Renode |
| [The link](link.ipynb) | One port, sessions through the broker: who owns the wire, who else is on it, what a close undoes. | the image on Renode |
| [Sensors](sensors.ipynb) | The BNO085 on SPI2, the A1335 on SPI4: what each reports, what each refuses, whether there is a magnet. | the image on Renode |
| [The power stage](power_stage.ipynb) | Dead time off the silicon, a counted hold, the six gates in one snapshot, a burst, the switching arithmetic. | the image on Renode |
| [Thermal](thermal.ipynb) | Twenty nodes on one thermometer: the fit, the board's estimate and budget, the envelope, the room identified, a trip. | the image on Renode |
| [The drive](drive.ipynb) | Five observers ranked, the firmware's law searched over 23-63 V, the envelope, the rotor observer on the board's model. | the image on Renode |
| [The controller](controller.ipynb) | Sources in, sinks out, feedback loops of prefilter, measure, estimator and regulator between; a panel; saved. | the image on Renode |
| [The cycle](cycle.ipynb) | The loop data-oriented: a frame a row a board and a field a channel, pure steps over every row at once in a table, `poll` and `push` the one edge to the bus; the tempo the loop's own. | the image on Renode |
| [The sequencer](sequencer.ipynb) | Streams in and out, a controller between, and a test stand in miniature: a table of setpoints with limits, jumps, init and cleanup. | the image on Renode |
| [The machines](machines.ipynb) | Every board a node; a humanoid, a quad, a fixed wing, an e-bike over them; programs a model writes, streamed live behind a failsafe. | the stand-in |
| [Motion](motion.ipynb) | A stepper and its ring, a servo and its sag, a sensorless speed loop against the shaft, the loop identified, a propeller. | the image on Renode |
| [Applications](applications.ipynb) | A throttle lane, a cruise through a gust, two joints on one wire, a hold against a load step. | the image on Renode |
| [Commissioning](commissioning.ipynb) | The motor measured step by step, identified, a tune searched against it, the record written, the drive verified. | the image on Renode |

## At the bench

In a paper's Setup: `MODE = ExecutionMode.HARDWARE` and `PORT` the board's
port - `coaxial.comm.ports.find()` names the first that answers.

## Changing one

A paper is written from `host/tools/notebooks/<its name>.py`: edit that, not
the notebook - what is typed into a notebook is gone when it is next laid.

```powershell
python host/tools/notebooks/current.py
```

It lays the papers behind their sources - run again where their code
changed, with the outputs they have where only their prose did; `setup.ps1`
and every pull and rebase do the same.
