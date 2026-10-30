"""Her errand's words to her: a word told once, a turn in place toward her way while off it."""
import math


def tell(g, gym, heard_fn, cmd):
    """`cmd` as the errand's word to her, once: the same twist again would restart her gait."""
    if g.get('asked') != cmd.get('twist'):
        heard_fn(g, gym, dict(cmd, _errand_cmd=True))


def steer(g, gym, heard_fn, err, wz_dir, speed, off, lag):
    """Turn in place toward `err` rad while she is `off` her way past her list, standing, or past
    `route.CONE` as far as her rhythm runs (`latent.GAIN`) toward `speed`; within it, walk `speed`,
    on her arc while she `lag`s `route.ARCED` or more, else by `err`."""
    from machine.senses import edge
    from machine.law import latent, modal, se2
    from machine.way import route
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    turn = abs(off) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and abs(lag) >= route.ARCED
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(edge.yaw(g['sense']) + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, math.copysign(se2.YAW_RATE, lag))), (True, (speed, 0.0, 0.0)))
    tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})
