"""Her gait's numbers of her build: each as it works on her, Froude-similar on another's leg.

    STRIDE = gait.table('stride')     # {name: number} of `[gait.stride]` at this build's leg

Her build's `[gait]` (`machine.body.mechatronics`) states each number as it works on her, at the leg
`leg_m` (thigh and shank), with what it was found at. A build of another leg has it Froude-similar: it goes
as the power of the leg its unit's dimension is (Alexander & Jayes 1983) - a length as the leg, a time or a
speed as its root, a rate as its inverse root, a stiffness per kg as its inverse, an angle, a share or a
force as it is -, nothing of it tuned to that build. Her law reads them here, its C twin through
`tools/cores/motion.py`. A force waits on her mass as built, which is her world's (`machine.body.build`).
"""
from machine.body import mechatronics
from machine.body.figure import SHANK, THIGH

_G = mechatronics.robot()['gait']
#: This build's leg over the leg her numbers are stated at: 1 on her.
SIMILAR = (THIGH + SHANK) / float(_G['leg_m'])
#: A unit's dimension as a power of the leg, by its name's ending: m/s and s as its root, rad/s, Hz and
#: 1/s as its inverse root, 1/s^2 and rad/s^2 as its inverse, m as it; anything else - rad, deg, a share,
#: m/s^2, N - as it is.
POWERS = (('_m_s', 0.5), ('_rad_s', -0.5), ('_hz', -0.5), ('_1_s2', -1.0), ('_rad_s2', -1.0), ('_1_s', -0.5),
          ('_m', 1.0), ('_s', 0.5))


def power(name):
    """The power of the leg a number named `name` goes as (`POWERS`), 0 for none."""
    return next((p for end, p in POWERS if name.endswith(end)), 0.0)


def table(part):
    """`[gait.<part>]` at this build's leg: {name: number}, each hers times `SIMILAR` to its power."""
    return {name: float(v) * SIMILAR ** power(name) for name, v in _G[part].items()}
