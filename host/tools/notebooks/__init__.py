"""The example notebooks, one module per functional area, each a short
paper: an abstract, numbered sections that measure something and read
the number back, conclusions with the numbers in them, and what to do
with them at the bench. Every module declares the same parts - TITLE,
SUBTITLE, ABSTRACT, SECTIONS, CONCLUSIONS, BENCH, REFERENCES - and
`parts.paper` lays every one out here, in one place;
`make_notebooks.py` writes and executes them.

The areas, in the order the README tables them:

    acquisition   the converters into records, frames and a live plot
    link          one port shared by sessions, and who else is attached
    sensors       the BNO085 and the A1335, and what each refuses over
    power_stage   the gate drivers armed and driven, and what switching costs
    thermal       the node network, its budget, and the room identified
    drive         the sensorless observers and the firmware's law, measured
    motion        the drive as verbs: stepper, servo, velocity
    applications  four missions on the verbs
    commissioning the bench-day procedure, end to end on the stand-in
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
