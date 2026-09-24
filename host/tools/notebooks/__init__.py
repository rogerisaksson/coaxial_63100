"""The example notebooks: one module per functional area, each a short paper."""
from . import (acquisition, applications, commissioning, controller, drive, link, machines,
               motion, power_stage, sensors, sequencer, thermal)
from .parts import paper


def _laid_out(area):
    return paper(area.TITLE, area.SUMMARY, area.SECTIONS, area.RESULTS, area.BENCH,
                 area.REFERENCES, getattr(area, 'DEVICE', True))


#: Name -> cells, in the README's order.
AREAS = {name: _laid_out(area) for name, area in (
    ('acquisition', acquisition), ('link', link), ('sensors', sensors),
    ('power_stage', power_stage), ('thermal', thermal), ('drive', drive),
    ('controller', controller), ('sequencer', sequencer), ('machines', machines),
    ('motion', motion), ('applications', applications),
    ('commissioning', commissioning))}
