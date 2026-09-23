"""The example notebooks: one module per functional area, each a short paper.

An abstract, numbered sections that measure something and read the number
back, conclusions with the numbers in them, and what to do with them at the
bench.
"""
from . import (acquisition, applications, commissioning, drive, link, motion,
               power_stage, sensors, thermal)
from .parts import paper


def _laid_out(area):
    """One area's cells, in the shape every paper has."""
    return paper(area.TITLE, area.SUBTITLE, area.ABSTRACT, area.SECTIONS,
                 area.CONCLUSIONS, area.BENCH, area.REFERENCES)


#: Name -> cells, in the README's order.
AREAS = {
    'acquisition': _laid_out(acquisition),
    'link': _laid_out(link),
    'sensors': _laid_out(sensors),
    'power_stage': _laid_out(power_stage),
    'thermal': _laid_out(thermal),
    'drive': _laid_out(drive),
    'motion': _laid_out(motion),
    'applications': _laid_out(applications),
    'commissioning': _laid_out(commissioning),
}
