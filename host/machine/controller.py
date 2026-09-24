"""A controller: feedback loops over named float channels, sources in, sinks out.

    loop = Loop(sources={'drive': Polled(drive.state)}, sinks={'drive': drive})
    loop.add('speed', Feedback(SpeedPI(...), setpoint='w_target',
                               measured='drive.omega_hat', command='iq_ref',
                               sink='drive.iq_ref', prefilter=Slew(157.0),
                               measure=Gain(1 / 7), estimator=SpeedKalman(...)))
    loop.write(w_target=100.0)       # the hook: any channel - a script, a plan, a model
    loop.run(2.0); loop.move(2.0, w_target=50.0); loop.follow(plan)
    loop.read()                      # every channel after the last pass
    loop.save(path); Loop.load(path, sources, sinks); with loop.saving(path): ...

A feedback steps prefilter, measure, estimator, regulator; loops step in the order added.
A source's read() lands as '<source>.<key>' - nested dicts and lists as '<key>.<sub>',
numbers and bools (0/1) only; a source or sink name may itself be dotted ('knee.drive').
A channel read before its writer steps holds the last pass's value. Under it: parts wired
port by port (plug, wire, route) for what is not a feedback.
"""
import atexit
import contextlib
import json
import threading
import time

from machine.errors import MachineError
from machine.parts import PI
from machine.roles import Controller, Estimator, Filter, Input, Part, Regulator


def _float(value):
    """A number as a float, a bool as 0/1; anything else None."""
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    return None


def flat(reading, prefix=''):
    """{channel: float} of a reading: nested dicts and lists by dotted key, numbers only."""
    items = reading.items() if isinstance(reading, dict) else enumerate(reading)
    out = {}
    for key, value in items:
        name = '%s%s' % (prefix, key)
        if isinstance(value, (dict, list, tuple)):
            out.update(flat(value, name + '.'))
        else:
            f = _float(value)
            if f is not None:
                out[name] = f
    return out


class Feedback:

    """One feedback loop: setpoint -> prefilter -> (+) -> regulator -> command -> sink;
    measured -> measure -> estimator -> back into (+). A slot is a part or None; a channel
    None is unwired, but `command`, `ref`, `value`, `estimate` default to '<loop>/<name>'."""

    SLOTS = {'prefilter': Filter, 'measure': Filter, 'estimator': Estimator,
             'regulator': Regulator}
    IO = ('setpoint', 'measured', 'command', 'sink', 'ref', 'value', 'estimate')

    def __init__(self, regulator=None, setpoint=None, measured=None, command=None, sink=None,
                 prefilter=None, measure=None, estimator=None, ref=None, value=None,
                 estimate=None):
        self.regulator = regulator or PI()
        self.prefilter, self.measure, self.estimator = prefilter, measure, estimator
        self.setpoint, self.measured, self.command, self.sink = setpoint, measured, command, sink
        self.ref, self.value, self.estimate = ref, value, estimate

    def slots(self):
        return {slot: getattr(self, slot) for slot in self.SLOTS}

    def config(self):
        return dict({k: getattr(self, k) for k in self.IO},
                    slots={s: _spec(p) if p is not None else None
                           for s, p in self.slots().items()})

    @classmethod
    def of(cls, cfg):
        slots = {s: _part(spec) if spec else None for s, spec in cfg['slots'].items()}
        return cls(**slots, **{k: cfg[k] for k in cls.IO})


class Loop(Controller):

    """Sources, feedback loops in order, sinks; one pass = read, step, write."""

    def __init__(self, sources, sinks, feedbacks=None, parts=None, wires=None, outputs=None,
                 rate_hz=25.0, clock=time.monotonic, sleep=time.sleep):
        self.sources, self.sinks = dict(sources), dict(sinks)
        self.parts, self.feedbacks = dict(parts or {}), {}
        self.wires, self.outputs, self.setpoints = {}, {}, set()
        self.pause = 1.0 / float(rate_hz)
        self._clock, self._sleep = clock, sleep
        self.bus = {'t': 0.0}
        self.wire(**(wires or {}))
        self.route(**(outputs or {}))
        for name, feedback in (feedbacks or {}).items():
            self.add(name, feedback)

    # -- feedback loops ----------------------------------------------------------------

    def add(self, name, feedback):
        """A feedback loop in as `name` - in place of one so named, else last."""
        if '.' in name or '/' in name:
            raise MachineError('a loop name has no . or /: %r' % name)
        order = list(self.feedbacks)
        if name in self.feedbacks:
            self.remove(name)
        else:
            order.append(name)
        self.feedbacks[name] = feedback
        self.feedbacks = {n: self.feedbacks[n] for n in order}
        f = feedback
        command, ref, value, est = (getattr(f, k) or '%s/%s' % (name, k)
                                    for k in ('command', 'ref', 'value', 'estimate'))
        setpoint, measured = f.setpoint, f.measured
        if f.prefilter is not None:
            self.plug(name + '/prefilter', f.prefilter, x=setpoint, y=ref)
            setpoint = ref
        if f.measure is not None:
            self.plug(name + '/measure', f.measure, x=measured, y=value)
            measured = value
        if f.estimator is not None:
            self.plug(name + '/estimator', f.estimator, measured=measured, command=command,
                      estimate=est)
            measured = est
        self.plug(name + '/regulator', f.regulator, setpoint=setpoint, measured=measured,
                  command=command)
        if f.sink:
            self.route(**{f.sink: command})
        self._order()
        return feedback

    def remove(self, name):
        """The feedback loop `name` out, its parts and its sink route with it."""
        feedback = self.feedbacks.pop(name)
        for part in [p for p in self.parts if p.startswith(name + '/')]:
            self.unplug(part)
        if feedback.sink and feedback.sink in self.outputs:
            self.route(**{feedback.sink: None})
        return feedback

    def _order(self):
        """Feedback loops' parts in loop order and slot order, then the free parts."""
        owned = ['%s/%s' % (n, s) for n in self.feedbacks for s in Feedback.SLOTS]
        self.parts = dict([(p, self.parts[p]) for p in owned if p in self.parts]
                          + [(p, q) for p, q in self.parts.items() if p not in owned])

    # -- wiring ------------------------------------------------------------------------

    def ports(self):
        return ['%s.%s' % (n, p) for n, part in self.parts.items()
                for p in part.INPUTS + part.OUTPUTS]

    def wire(self, **wires):
        """'<part>.<port>': channel; None unwires it."""
        for key, channel in wires.items():
            if key not in self.ports():
                raise MachineError('no port %s - there are %s' % (key, ', '.join(self.ports())))
            if channel is None:
                self.wires.pop(key, None)
            else:
                self.wires[key] = channel
        return dict(self.wires)

    def route(self, **outputs):
        """'<sink>.<key>': channel, written to that sink every pass; None drops it."""
        for key, channel in outputs.items():
            if key.rpartition('.')[0] not in self.sinks:
                raise MachineError('no sink %s - there are %s' % (key, ', '.join(self.sinks)))
            if channel is None:
                self.outputs.pop(key, None)
            else:
                self.outputs[key] = channel
        return dict(self.outputs)

    def plug(self, name, part, before=None, **ports):
        """`part` in as `name` - in place of one so named, else before `before`, else last -
        and its ports wired: plug('kf', SpeedKalman(...), before='pi', measured='w')."""
        items = list(self.parts.items())
        names = [n for n, _ in items]
        if name in names:
            items[names.index(name)] = (name, part)
        else:
            items.insert(names.index(before) if before else len(items), (name, part))
        self.parts = dict(items)
        own = set(part.INPUTS + part.OUTPUTS)
        for key in [k for k in self.wires if k.partition('.')[0] == name]:
            if key.partition('.')[2] not in own:
                del self.wires[key]
        return self.wire(**{'%s.%s' % (name, port): ch for port, ch in ports.items()})

    def unplug(self, name):
        part = self.parts.pop(name)
        for key in [k for k in self.wires if k.partition('.')[0] == name]:
            del self.wires[key]
        return part

    def source_of(self, channel):
        """The source a channel comes from, or None."""
        return next((s for s in sorted(self.sources, key=len, reverse=True)
                     if channel.startswith(s + '.')), None)

    def reads(self):
        """Every channel a part reads or a sink is written from."""
        return ({ch for key, ch in self.wires.items()
                 if key.rpartition('.')[2] in self.parts[key.rpartition('.')[0]].INPUTS}
                | set(self.outputs.values()))

    def targets(self):
        """What a feedback loop is told: each one's setpoint channel."""
        return [f.setpoint for f in self.feedbacks.values() if f.setpoint]

    def controlled(self):
        """What each feedback loop holds: the channel its regulator measures."""
        return [self.wires['%s/regulator.measured' % n] for n in self.feedbacks
                if '%s/regulator.measured' % n in self.wires]

    def channel_of(self, name, port):
        """Where a part's port reads or publishes."""
        return self.wires.get('%s.%s' % (name, port), '%s.%s' % (name, port))

    def channels(self):
        """Every channel a pass knows: the sources' (read now if never read), the parts',
        the setpoints'."""
        if not any(self.source_of(k) for k in self.bus):
            self._read_sources()
        return sorted(set(self.bus) | set(self.wires.values()) | set(self.outputs.values())
                      | {self.channel_of(n, p) for n, part in self.parts.items()
                         for p in part.OUTPUTS})

    # -- the verbs ---------------------------------------------------------------------

    def state(self):
        return {'bus': dict(self.bus), 'wires': dict(self.wires), 'outputs': dict(self.outputs)}

    def write(self, **channels):
        """The hook: setpoints on any channel, floats (a bool is 0/1)."""
        for key, value in channels.items():
            f = _float(value)
            if f is None:
                raise MachineError('%s: a channel carries a float, not %r' % (key, value))
            self.bus[key] = f
            self.setpoints.add(key)
        return {k: self.bus[k] for k in channels}

    def read(self, count=None, timeout=None):
        return dict(self.bus)

    def reset(self):
        """Every part back to rest; the bus to its setpoints."""
        for part in self.parts.values():
            part.reset()
        self.bus = {'t': 0.0, **{k: self.bus[k] for k in self.setpoints if k in self.bus}}

    def _read_sources(self):
        for name, source in self.sources.items():
            got = source.read()
            if got.get('fault'):
                raise MachineError('%s faulted mid-loop - %s; the loop is over'
                               % (name, got['fault']))
            self.bus.update(flat(got, name + '.'))

    def step(self, dt):
        """One pass; every channel after it."""
        self._read_sources()
        for name, part in self.parts.items():
            inputs = {p: self.bus.get(self.wires['%s.%s' % (name, p)], 0.0)
                      for p in part.INPUTS if '%s.%s' % (name, p) in self.wires}
            for port, value in part.step(dt, **inputs).items():
                self.bus[self.channel_of(name, port)] = float(value)
        written = {}
        for key, channel in self.outputs.items():
            sink, _, what = key.rpartition('.')
            written.setdefault(sink, {})[what] = self.bus.get(channel, 0.0)
        for sink, values in written.items():
            self.sinks[sink].write(**values)
        self.bus['t'] += dt
        return dict(self.bus)

    def run(self, seconds, watch=None):
        """Serve the loop for `seconds` of the clock; `watch(loop)` after every pass, and a
        true answer ends the block."""
        rows = []
        last = start = self._clock()
        while self._clock() - start < seconds:
            now = self._clock()
            rows.append(self.step(max(now - last, 1e-6)))
            last = now
            if watch is not None and watch(self):
                break
            self._sleep(self.pause)
        return rows

    def move(self, seconds, watch=None, **setpoints):
        self.write(**setpoints)
        return self.run(seconds, watch)

    def follow(self, plan, watch=None):
        """Setpoints in sequence: (seconds, {channel: value}) pairs, or a planner
        `plan(loop) -> (seconds, {channel: value})`, None when done."""
        steps = iter(lambda: plan(self), None) if callable(plan) else iter(plan)
        rows = []
        for seconds, setpoints in steps:
            rows += self.move(seconds, watch, **setpoints)
        return rows

    def off(self):
        """Every paced part stopped, every sink to its safe state."""
        for part in self.parts.values():
            if isinstance(part, Paced):
                part.stop()
        return {name: sink.off() for name, sink in self.sinks.items()}

    # -- kept ---------------------------------------------------------------------------

    def config(self):
        """Feedback loops, free parts and their wires, setpoints: what save() writes."""
        routed = {f.sink for f in self.feedbacks.values() if f.sink}
        free = [p for p in self.parts if p.partition('/')[0] not in self.feedbacks]
        return {'rate_hz': 1.0 / self.pause,
                'feedbacks': {n: f.config() for n, f in self.feedbacks.items()},
                'parts': {n: _spec(self.parts[n]) for n in free},
                'wires': {k: v for k, v in self.wires.items() if k.partition('.')[0] in free},
                'outputs': {k: v for k, v in self.outputs.items() if k not in routed},
                'setpoints': {k: self.bus[k] for k in sorted(self.setpoints) if k in self.bus}}

    def save(self, path):
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(self.config(), handle, indent=1)
        return path

    @classmethod
    def load(cls, path, sources, sinks, **kw):
        """A saved loop on live sources and sinks, by their names."""
        with open(path, encoding='utf-8') as handle:
            cfg = json.load(handle)
        loop = cls(sources, sinks, {n: Feedback.of(f) for n, f in cfg['feedbacks'].items()},
                   {n: _part(s) for n, s in cfg['parts'].items()}, cfg['wires'],
                   cfg['outputs'], rate_hz=cfg['rate_hz'], **kw)
        loop.write(**cfg['setpoints'])
        return loop

    @contextlib.contextmanager
    def saving(self, path):
        """The block, then save(path) however it ended."""
        try:
            yield self
        finally:
            self.save(path)

    def save_at_exit(self, path):
        atexit.register(self.save, path)
        return path

    def __str__(self):
        from machine.wiring import diagram
        return diagram(self, colour=False)


def _spec(part):
    if isinstance(part, Paced):
        return dict(_spec(part.part), hz=1.0 / part.pause)
    return {'kind': type(part).__name__, 'params': part.params()}


def _part(spec):
    kind = Part.KINDS.get(spec['kind'])
    if kind is None:
        raise MachineError('no part kind %s in this process - import the module that defines it; '
                       'known: %s' % (spec['kind'], ', '.join(sorted(Part.KINDS))))
    part = kind(**spec['params'])
    return Paced(part, spec['hz']) if 'hz' in spec else part


class Paced:

    """A part on its own thread at its own rate, answering the loop with its latest outputs.
    `feed()` -> {port: float} refreshes inputs between passes (an estimator faster than
    the loop reading its own measurement)."""

    def __init__(self, part, hz, feed=None, clock=time.monotonic, sleep=time.sleep):
        self.part, self.pause, self.feed = part, 1.0 / float(hz), feed
        self._clock, self._sleep = clock, sleep
        self._inputs, self._out, self._thread = {}, None, None
        self._lock, self._part_lock = threading.Lock(), threading.Lock()
        self._ready = threading.Event()
        self._stopping = False
        self.steps = 0

    INPUTS = property(lambda self: self.part.INPUTS)
    OUTPUTS = property(lambda self: self.part.OUTPUTS)
    PARAMS = property(lambda self: self.part.PARAMS)

    def params(self):
        return self.part.params()

    def configure(self, **params):
        with self._part_lock:
            return self.part.configure(**params)

    def step(self, dt, **inputs):
        with self._lock:
            self._inputs = inputs
        if self._thread is None:
            self._stopping = False
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()
        self._ready.wait()
        with self._lock:
            return dict(self._out)

    def _run(self):
        last = self._clock()
        while not self._stopping:
            now = self._clock()
            with self._lock:
                inputs = dict(self._inputs)
            if self.feed is not None:
                inputs.update(self.feed())
            with self._part_lock:
                out = self.part.step(max(now - last, 1e-6), **inputs)
            last = now
            with self._lock:
                self._out = out
                self.steps += 1
            self._ready.set()
            self._sleep(self.pause)

    def reset(self):
        with self._part_lock:
            self.part.reset()

    def stop(self):
        if self._thread is not None:
            self._stopping = True
            self._thread.join()
            self._thread = None
            self._ready.clear()


class Polled(Input):

    """Any call that answers a dict, as a source: `Polled(drive.state)`."""

    def __init__(self, call):
        self.call = call

    def state(self):
        return self.call()

    def read(self, count=None, timeout=None):
        return self.call()
