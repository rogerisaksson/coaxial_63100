"""A chair's body as the floor parks it: its seat at a height, its back, its four legs."""
from machine.body import figure

#: The chair, as wide as her hands hang and a fist: 0.44 m wide they hung 4-5 cm past it.
CHAIR_SEAT, CHAIR_SEAT_H, CHAIR_BACK = (figure.ASIDE + figure.FIST_R, 0.02, 0.22), 0.45, (figure.ASIDE + figure.FIST_R, 0.20, 0.02)
CHAIR_LEG, CHAIR_LEG_IN = 0.02, 0.03


def chair(name, seat_h, contacts):
    """A chair's body, parked: its seat `seat_h` m up, its back, four legs."""
    leg_h = (seat_h - 2 * CHAIR_SEAT[1]) / 2.0
    return ('<body name="%s" mocap="true" pos="0 -5 0">'
            '<geom name="%s_seat" type="box" size="%g %g %g" pos="0 %g 0"%s/>'
            '<geom name="%s_back" type="box" size="%g %g %g" pos="0 %g %g"%s/>' % (
                (name, name) + CHAIR_SEAT + (seat_h - CHAIR_SEAT[1], contacts, name)
                + CHAIR_BACK + (seat_h + CHAIR_BACK[1], CHAIR_SEAT[2], contacts))
            + ''.join('<geom name="%s_leg_%s" type="box" size="%g %g %g" pos="%g %g %g"%s/>' % (
                name, leg, CHAIR_LEG, leg_h, CHAIR_LEG, sx * (CHAIR_SEAT[0] - CHAIR_LEG_IN), leg_h,
                sz * (CHAIR_SEAT[2] - CHAIR_LEG_IN), contacts)
                for leg, sz, sx in (('fl', -1, -1), ('fr', -1, 1), ('bl', 1, -1), ('br', 1, 1))) + '</body>')
