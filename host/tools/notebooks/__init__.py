"""The example notebooks: one module per functional area, each a short paper."""
from . import (acquisition, applications, commissioning, controller, drive, link, motion,
               power_stage, sensors, thermal)
from .parts import paper


def _laid_out(area):
    return paper(area.TITLE, area.SUMMARY, area.SECTIONS, area.RESULTS, area.BENCH,
                 area.REFERENCES)


#: Name -> cells, in the README's order.
AREAS = {name: _laid_out(area) for name, area in (
    ('acquisition', acquisition), ('link', link), ('sensors', sensors),
    ('power_stage', power_stage), ('thermal', thermal), ('drive', drive),
    ('controller', controller), ('motion', motion), ('applications', applications),
    ('commissioning', commissioning))}
