"""Her gait's numbers of her build: each as it works on her, Froude-similar on another's leg and mass.

    STRIDE = gait.table('stride')     # {name: number} of `[gait.stride]` at this build's leg and mass

Her build's `[gait]` (`machine.body.mechatronics`) states each number as it works on her, at the leg
`leg_m` (thigh and shank) and the mass as built `mass_kg` (`machine.body.build`), with what it was found
at. A build of another leg and mass has it Froude-similar: it goes as the powers of the leg and the mass
its unit's dimensions are (Alexander & Jayes 1983) - a length as the leg, a time or a speed as its root,
a rate as its inverse root, a stiffness per kg as its inverse, a force or a mass as the mass, a torque as
both, an angle or a share as it is -, nothing of it tuned to that build. Her law reads them here, its C
twin through `tools/cores/motion.py`.
"""
from machine.body import figure, mechatronics
from machine.body.figure import SHANK, THIGH

_G = mechatronics.robot()['gait']
#: This build's leg and mass as built over those her numbers are stated at: 1 and 1 on her.
SIMILAR = (THIGH + SHANK) / float(_G['leg_m'])
MASSY = figure.mass() / float(_G['mass_kg'])
#: A unit's dimension as powers of the leg and the mass, by its name's ending: m/s and s as the leg's root,
#: rad/s, Hz, 1/s and s/m as its inverse root, 1/s^2 and rad/s^2 as its inverse, m as it, N and kg as the
#: mass, N m as both; anything else - rad, deg, a share, m/s^2 - as it is.
POWERS = (('_m_s', 0.5, 0.0), ('_rad_s', -0.5, 0.0), ('_hz', -0.5, 0.0), ('_1_s2', -1.0, 0.0), ('_rad_s2', -1.0, 0.0),
          ('_1_s', -0.5, 0.0), ('_s_m', -0.5, 0.0), ('_nm', 1.0, 1.0), ('_n', 0.0, 1.0), ('_kg', 0.0, 1.0),
          ('_m', 1.0, 0.0), ('_s', 0.5, 0.0))


def power(name):
    """The powers of the leg and of the mass a number named `name` goes as (`POWERS`), 0 for none."""
    return next(((leg, mass) for end, leg, mass in POWERS if name.endswith(end)), (0.0, 0.0))


def table(part):
    """`[gait.<part>]` at this build's leg and mass: {name: number}, each hers times `SIMILAR` and `MASSY`
    to its powers."""
    return {name: float(v) * SIMILAR ** power(name)[0] * MASSY ** power(name)[1] for name, v in _G[part].items()}
