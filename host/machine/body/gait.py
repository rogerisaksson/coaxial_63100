"""Her gait's numbers of her build: each derived on her, Froude-similar on another's leg and mass.

    STRIDE = gait.table('stride')     # {name: number} of `[gait.stride]` at this build's leg and mass

`machine.body.compile` derives her tables at this build's leg `leg_m` (thigh and shank) and mass as
built `mass_kg` (`machine.body.build`), merged with any `[gait]` overrides in her build file. Where
an override states another `leg_m` or `mass_kg`, each number goes as the powers of the leg and the
mass its unit's dimensions are (Alexander & Jayes 1983). Her law reads them here, its C twin through
`tools/cores/motion.py`.
"""
from machine.body import compile, figure, mechatronics
from machine.body.figure import SHANK, THIGH

_G = compile.gait(mechatronics.robot().get('gait'))
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
