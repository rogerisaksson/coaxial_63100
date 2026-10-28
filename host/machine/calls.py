"""What her law calls, by name, a count each, between two takes: her own functions and her C cores.

    tracer = calls.Calls().start(s)     # s['calls']: each snapshot (`introspect.Tap`) carries its pass's
    counts = tracer.take()              # {'motion.control': 1, 'c.poses': 1, 'c.wbh_gmo_step': 1, ..}
    tracer.stop(s)

`sys.monitoring`: PY_START on her modules' code (machine/), CALL where her code or her cores' wrappers
(tools/) call into C - a compiled function (`bodied`) by its name, a library's by its symbol, `c.` before
each. Any other code, and any call site that calls into Python, disabled at its first event; the tracer's
own and her telemetry's (QUIET) never counted.
"""
import ctypes
import os
import sys

#: Her modules' folder, and the host's: code under the first is hers, a call into C under the second.
ROOT = os.path.normcase(os.path.dirname(os.path.abspath(__file__)))
HOST = os.path.dirname(ROOT)
#: Her telemetry's functions, uncounted: a snapshot's own calls are no pass of hers.
QUIET = ('calls.', 'introspect.', 'watch.', 'gate.', 'analyzers.', 'observer.named', 'observer.skin')
#: The tool `sys.monitoring` counts under.
TOOL = sys.monitoring.PROFILER_ID
_EVENTS = sys.monitoring.events


class Calls:

    """A tracer of her calls: started on a session, taken a snapshot at a time, stopped."""

    def __init__(self):
        self.counts, self.names, self.compiled = {}, {}, ()

    def _name(self, code):
        """Code `code`'s name among hers ('module.qualname'), '' for any other."""
        name = self.names.get(code)
        if name is None:
            path = os.path.normcase(code.co_filename)
            module = os.path.splitext(os.path.relpath(path, ROOT))[0].replace(os.sep, '.') if path.startswith(ROOT) else ''
            name = module and module + '.' + code.co_qualname
            name = self.names[code] = '' if name.startswith(QUIET) else name
        return name

    def _started(self, code, _offset):
        name = self._name(code)
        if not name:
            return sys.monitoring.DISABLE
        self.counts[name] = self.counts.get(name, 0) + 1

    def _called(self, code, _offset, fn, _arg0):
        if not os.path.normcase(code.co_filename).startswith(HOST):
            return sys.monitoring.DISABLE
        name = getattr(fn, 'name', None) if isinstance(fn, self.compiled) else (
            fn.__name__ if isinstance(fn, ctypes._CFuncPtr) else None)
        if name is None:
            return sys.monitoring.DISABLE
        self.counts['c.' + name] = self.counts.get('c.' + name, 0) + 1

    def start(self, s):
        """Counting from now, the tracer `s['calls']`."""
        from tools.cores.bodied import Compiled
        self.compiled = Compiled
        sys.monitoring.use_tool_id(TOOL, 'calls')
        sys.monitoring.register_callback(TOOL, _EVENTS.PY_START, self._started)
        sys.monitoring.register_callback(TOOL, _EVENTS.CALL, self._called)
        sys.monitoring.set_events(TOOL, _EVENTS.PY_START | _EVENTS.CALL)
        s['calls'] = self
        return self

    def take(self):
        """{name: count} since the last take."""
        out, self.counts = self.counts, {}
        return out

    def stop(self, s):
        """Counting no more; `s['calls']` gone."""
        sys.monitoring.set_events(TOOL, 0)
        for event in (_EVENTS.PY_START, _EVENTS.CALL):
            sys.monitoring.register_callback(TOOL, event, None)
        sys.monitoring.free_tool_id(TOOL)
        sys.monitoring.restart_events()
        s.pop('calls', None)
