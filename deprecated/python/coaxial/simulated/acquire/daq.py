"""Moved from host/coaxial/simulated/acquire/daq.py: _source_mask - nothing in the tree named it."""
from coaxial.simulated.acquire.daq import SOURCES


def _source_mask(sources):
    """A source mask from names, or the mask itself when given one."""
    if isinstance(sources, int):
        return sources
    unknown = [s for s in sources if s not in SOURCES]
    if unknown:
        raise ValueError('no such source: %s - have %s'
                         % (', '.join(unknown), ', '.join(SOURCES)))
    return sum(1 << SOURCES[s] for s in set(sources))
