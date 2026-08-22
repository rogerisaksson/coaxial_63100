"""The plan an Ollama-driven run executes, and the only place limits exist.

This is testline/plan.py's idea with one field changed and one rule added.

The changed field: a `testline` step names a `source`, a Python callable that
produces the number. Here a step carries an `ask` instead - prose, aimed at a
model that has the board's tool surface in front of it. That is the whole reason
this runner exists: a step like "find out which channel actually moves when the
DC link changes" has no callable behind it, and writing one for every such
question is how bring-up scripts turn into a second firmware.

The added rule, and it is the important one: **the model never sees the limit.**
It is asked to measure and to report a number in a stated unit; `Limit.judge`
in Python decides the verdict, from a file under revision control. A language
model told "pass is under 0.25 V" will, sooner or later, report a number that
passes. Keeping the limit out of the context is not distrust of any particular
model, it is what makes the verdict traceable to the plan instead of to a
sampling temperature.

Everything testline/plan.py says about where limits come from applies unchanged,
so `measurement_system_study` is required here too. A plan without one is not
loaded.
"""
import sys

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

import yaml                                       # noqa: E402

from testline.plan import Limit, PlanError        # noqa: E402  re-exported

__all__ = ['Limit', 'PlanError', 'Task', 'Plan']

# Enough turns for read-orient-measure-report, and few enough that a model stuck
# in a loop costs a step rather than an afternoon.
DEFAULT_TURNS = 12


class Task:
    """One thing the model is asked to establish about the board.

    `ask` is the prompt. `unit` is told to the model so its number arrives in
    the unit the limit is written in; the limit itself is not.
    """

    def __init__(self, ident, name, ask, limit=None, record_only=False,
                 max_turns=DEFAULT_TURNS, unit='', needs_writes=False):
        if not ask:
            raise PlanError('task %s has no `ask`' % ident)
        self.id = ident
        self.name = name
        self.ask = ask
        self.limit = limit
        self.record_only = record_only or limit is None
        self.max_turns = max_turns
        self.unit = unit or (limit.unit if limit else '')
        self.needs_writes = needs_writes

    def __repr__(self):
        return '<Task %s %s>' % (self.id, 'record' if self.record_only
                                 else self.limit.describe())

    def brief(self):
        """What the model is told. Note what is absent: the limit."""
        lines = ['Step %s - %s' % (self.id, self.name), self.ask]
        if self.unit:
            lines.append('Report the value as a number in %s.' % self.unit)
        else:
            lines.append('Report a value if there is one to report; otherwise '
                         'report your finding as text.')
        return '\n'.join(lines)


class Plan:
    def __init__(self, data, path=None):
        self.path = path
        try:
            self.product = data['product']
            self.revision = data['revision']
            self.plan_version = data['plan_version']
            self.study = data['measurement_system_study']
        except KeyError as exc:
            raise PlanError(
                'plan is missing %s. Without a product, a revision and a '
                'measurement_system_study reference a plan cannot be audited, '
                'so it is not accepted.' % exc) from exc

        self.description = data.get('description', '')
        self.model = data.get('model')
        self.allow = list(data.get('allow_commands', []))
        self.allow_writes = bool(data.get('allow_writes', False))
        self.context = data.get('context', '')
        self.tasks = []

        for raw in data.get('tasks', []):
            limit = None
            if 'limit' in raw:
                spec = raw['limit']
                limit = Limit(spec.get('low'), spec.get('high'),
                              spec.get('unit', ''))
            self.tasks.append(Task(
                raw['id'], raw.get('name', raw['id']), raw.get('ask'),
                limit, raw.get('record_only', False),
                raw.get('max_turns', DEFAULT_TURNS), raw.get('unit', ''),
                raw.get('needs_writes', False)))

        if not self.tasks:
            raise PlanError('plan defines no tasks')

    def __repr__(self):
        return '<Plan %s rev %s v%s, %d tasks>' % (
            self.product, self.revision, self.plan_version, len(self.tasks))

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as handle:
            return cls(yaml.safe_load(handle), path=str(path))

    @classmethod
    def single(cls, ask, unit='', name='ad-hoc', max_turns=DEFAULT_TURNS):
        """One question from the command line.

        Still a plan, and still unjudged: an ad-hoc question has no study behind
        it, so it can only ever be record_only. That is why the study field
        below says so instead of being left blank.
        """
        return cls({
            'product': 'coaxial_63100 BLDC inverter',
            'revision': 'unspecified',
            'plan_version': 'ad-hoc',
            'measurement_system_study': 'NONE - ad-hoc question, records only',
            'tasks': [{'id': 'A010', 'name': name, 'ask': ask, 'unit': unit,
                       'record_only': True, 'max_turns': max_turns}],
        })

    def header(self):
        return {
            'product': self.product,
            'pcba_revision': self.revision,
            'plan_version': self.plan_version,
            'plan_file': self.path or '(inline)',
            'measurement_system_study': self.study,
            'description': self.description,
            'tasks': len(self.tasks),
        }
