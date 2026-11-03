"""The crafts: the quad, the helicopter, the ornithopter and the aerobat, their air and courses."""
import glob
import os

from machine.errors import MachineError

#: The crafts' texts (lisp/aircraft), each kind's own by its kind: COAXIAL_QUAD, COAXIAL_HELICOPTER,
#: COAXIAL_ORNITHOPTER or COAXIAL_AEROBAT in the environment names another a process flies as that kind - a
#: variant, `<own>_<variant>.lisp`: its own vehicle (its blocks: frame, rotors, governor, masses) on its kind's
#: flights (lisp/aircraft/flights/<own>.lisp).
LISP = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lisp', 'aircraft')
CRAFTS = {'quad': 'quadcopter', 'helicopter': 'helicopter', 'ornithopter': 'ornithopter', 'aerobat': 'aerobat'}


def variants(kind):
    """Every text that flies as `kind`: its own, then its variants by name."""
    own = CRAFTS[kind]
    return [own] + sorted(os.path.basename(p)[:-5] for p in glob.glob(os.path.join(LISP, own + '_*.lisp')))


def text(kind):
    """The path of the text this process flies as `kind`: its own, or the one COAXIAL_<KIND> names."""
    name = os.environ.get('COAXIAL_' + kind.upper(), CRAFTS[kind])
    path = os.path.join(LISP, name + '.lisp')
    if not os.path.isfile(path):
        raise MachineError('no %s text %s: %s' % (kind, name, ', '.join(variants(kind))))
    return path
