"""An Ollama-driven test runner for the coaxial_63100 bench."""
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
