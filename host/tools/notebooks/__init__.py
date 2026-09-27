"""The example notebooks: one module per functional area, each a short paper."""
from . import (acquisition, applications, commissioning, controller, cycle, director, drive,
               link, machines, motion, power_stage, sensors, sequencer, thermal)
from .parts import EMULATED, paper


#: Papers over discovered nodes: no single device opened.
NODES = ('controller', 'cycle', 'machines', 'sequencer', 'director')

#: Papers on the stand-in's rotor: their J and load are set through `drive.model`, which an
#: emulated drive keeps to itself - the world's flywheel never feels them (docs/TODO.md).
ROTOR = ('motion', 'applications')
STAND_IN = ('SIMULATED', 'the stand-in: J and the load are set on its rotor through '
            '`drive.model`, which an emulated drive keeps to itself')


#: Name -> module, in the README's order.
MODULES = {'acquisition': acquisition, 'link': link, 'sensors': sensors,
           'power_stage': power_stage, 'thermal': thermal, 'drive': drive,
           'controller': controller, 'cycle': cycle, 'sequencer': sequencer,
           'machines': machines, 'director': director, 'motion': motion,
           'applications': applications, 'commissioning': commissioning}


def mode_of(name):
    """A paper's own mode: (the ExecutionMode's name, why)."""
    return STAND_IN if name in ROTOR else EMULATED


def paper_of(name, mode=None):
    """A paper's cells on its own mode, or on `mode`: (the ExecutionMode's name, why)."""
    area = MODULES[name]
    return paper(area.TITLE, area.SUMMARY, area.SECTIONS, area.RESULTS, area.BENCH,
                 area.REFERENCES, name not in NODES, mode or mode_of(name))


#: Name -> cells, in the README's order.
AREAS = {name: paper_of(name) for name in MODULES}
