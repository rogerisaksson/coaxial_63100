"""The example notebooks: one module per functional area, each a short paper."""
from . import (acquisition, applications, commissioning, controller, cycle, director, drive,
               link, machines, motion, power_stage, sensors, sequencer, thermal)
from .parts import paper


#: Papers over discovered nodes: no single device opened.
NODES = ('controller', 'cycle', 'machines', 'sequencer', 'director')


def _laid_out(name, area):
    return paper(area.TITLE, area.SUMMARY, area.SECTIONS, area.RESULTS, area.BENCH,
                 area.REFERENCES, name not in NODES)


#: Name -> cells, in the README's order.
AREAS = {name: _laid_out(name, area) for name, area in (
    ('acquisition', acquisition), ('link', link), ('sensors', sensors),
    ('power_stage', power_stage), ('thermal', thermal), ('drive', drive),
    ('controller', controller), ('cycle', cycle), ('sequencer', sequencer), ('machines', machines),
    ('director', director),
    ('motion', motion), ('applications', applications),
    ('commissioning', commissioning))}
