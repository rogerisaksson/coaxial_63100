"""The card a process draws on: its adapter and the one device every crew and lit raster shares.

Asked once a process: a device a crew re-ran wgpu's start - its C types parsed, the shader
compiled -, 1.9 s of a cold attitude page's 5.2 s to its first frame, the bench measured.
COAXIAL_GPU=0 keeps every frame on the CPU.
"""
import os
import threading

#: The switch: 0 keeps the pages on the CPU crew.
ENV = 'COAXIAL_GPU'

#: The adapter, and by its id the device on it.
_CARD: dict = {}
_LOCK = threading.RLock()


def adapter():
    """The card's adapter, or None: wgpu absent, no GPU but a software one, or
    COAXIAL_GPU=0. Asked once a process."""
    if os.environ.get(ENV, '1') == '0':
        return None
    with _LOCK:
        if 'adapter' not in _CARD:
            _CARD['adapter'] = _asked()
        return _CARD['adapter']


def _asked():
    """wgpu's high-performance adapter, or None."""
    try:
        import wgpu
    except ImportError:
        return None
    try:
        found = wgpu.gpu.request_adapter_sync(power_preference='high-performance')
    except (RuntimeError, OSError, wgpu.GPUError):
        return None
    if found is None or found.info.get('adapter_type') == 'CPU':
        return None
    return found


def device_of(found):
    """`found`'s one device in this process, requested on first ask."""
    with _LOCK:
        got = _CARD.get(id(found))
        if got is None or got[0] is not found:
            got = _CARD[id(found)] = (found, found.request_device_sync())
        return got[1]
