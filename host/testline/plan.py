"""The test plan: what to measure, and what counts as passing."""
import yaml


class PlanError(Exception):
    """A plan that cannot be audited: a missing product, revision or study
    reference, or a step whose limit does not parse.
    """


class Limit:
    """A one- or two-sided limit, in the measurement's own unit."""

    def __init__(self, low=None, high=None, unit=''):
        if low is None and high is None:
            raise PlanError('a limit needs at least one bound')
        self.low = low
        self.high = high
        self.unit = unit

    def __repr__(self):
        return '<Limit %s>' % self.describe()

    def describe(self):
        if self.low is not None and self.high is not None:
            return '%g .. %g %s' % (self.low, self.high, self.unit)
        if self.low is not None:
            return '>= %g %s' % (self.low, self.unit)
        return '<= %g %s' % (self.high, self.unit)

    def judge(self, value):
        if value is None:
            return False
        if self.low is not None and value < self.low:
            return False
        if self.high is not None and value > self.high:
            return False
        return True


class Step:
    """One measurement and the limit it is judged against."""

    def __init__(self, ident, name, source, limit=None, args=None,
                 record_only=False):
        self.id = ident
        self.name = name
        self.source = source
        self.limit = limit
        self.args = args or {}
        self.record_only = record_only

    def __repr__(self):
        return '<Step %s %s>' % (self.id, self.source)


class TestPlan:
    """One YAML plan, loaded and checked: what to measure, in what order,
    and what counts as passing.
    """

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
        self.steps = []

        for raw in data.get('steps', []):
            limit = None
            if 'limit' in raw:
                spec = raw['limit']
                limit = Limit(spec.get('low'), spec.get('high'),
                              spec.get('unit', ''))
            self.steps.append(Step(raw['id'], raw['name'], raw['source'],
                                   limit, raw.get('args'),
                                   raw.get('record_only', False)))

        if not self.steps:
            raise PlanError('plan defines no steps')

    def __repr__(self):
        return '<TestPlan %s rev %s v%s, %d steps>' % (
            self.product, self.revision, self.plan_version, len(self.steps))

    @classmethod
    def load(cls, path):
        with open(path, encoding='utf-8') as handle:
            return cls(yaml.safe_load(handle), path=str(path))

    def header(self):
        """What identifies this plan in a report header."""
        return {
            'product': self.product,
            'pcba_revision': self.revision,
            'plan_version': self.plan_version,
            'plan_file': self.path or '(inline)',
            'measurement_system_study': self.study,
            'description': self.description,
        }
