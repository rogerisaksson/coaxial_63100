"""An Ollama-driven test runner for the coaxial_63100 bench.

    cd host
    python -m coaxial_ollama --plan coaxial_ollama/plans/bringup.yaml
    python -m coaxial_ollama --ask "what does the NTC channel read right now?"

A local model gets the board's tool surface, a Python scope with the live
`board` in it, and an allowlisted shell. It works one plan step at a time and
finishes each with a reported value. The verdict is not its to give: limits live
in the plan file and `plan.Limit` applies them.

    plan.py       the steps and the limits - the only limits in this repository
    client.py     Ollama over stdlib urllib
    tools.py      coaxial_mcp's eight board tools, plus code, shell and report
    sandbox.py    where model-authored code and commands actually run
    runner.py     the turn loop, the transcript, the verdict

Why a local model at all: bring-up questions like "which channel moves when I
change the link voltage" have no callable behind them, and writing one for each
is how a bench script turns into a second firmware. Why local specifically: a
transcript of a board under test is measurement data, and it stays on the bench
PC.
"""
from .client import Ollama, OllamaError, is_cloud, is_local
from .plan import Limit, Plan, PlanError, Task
from .runner import Record, Runner, Transcript, report_text
from .sandbox import Scope, Shell
from .tools import TOOLS, Toolbox

__all__ = ['Ollama', 'OllamaError', 'is_cloud', 'is_local',
           'Plan', 'PlanError', 'Task', 'Limit',
           'Runner', 'Record', 'Transcript', 'report_text', 'Scope', 'Shell',
           'Toolbox', 'TOOLS']

__version__ = '0.1.0'
