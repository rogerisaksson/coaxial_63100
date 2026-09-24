"""One lock for the stand-in's rotor: every method that touches it runs under it."""
import functools


def _rotor_locked(method):
    """Run a rotor-touching method under `self._lock`."""
    @functools.wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapped
