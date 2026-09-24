"""The plan an Ollama-driven run executes, and the only place limits exist."""
import yaml

from testline.plan import Limit, PlanError

__all__ = ['Limit', 'PlanError', 'Task', 'Plan']

# Enough turns for read-orient-measure-report, and few enough that a model
# stuck in a loop costs a step rather than an afternoon.
DEFAULT_TURNS = 12


class Task:
    """One thing the model is asked to establish about the board."""

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
                                 else (self.limit.describe() if self.limit
                                       else 'no limit'))
    def brief(self):
        """What the model is told: everything but the limit."""
        lines = ['Step %s - %s' % (self.id, self.name), self.ask]
        if self.unit:
            lines.append('Report the value as a number in %s.' % self.unit)
        else:
            lines.append('Report a value if there is one to report; otherwise '
                         'report your finding as text.')
        return '\n'.join(lines)


class Plan:
    """A YAML test plan: the steps, and the limits the model is never shown."""
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
        """One question from the command line."""
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
