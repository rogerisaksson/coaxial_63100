"""Her errand's words to her: a word told once, a turn in place toward her way while off it."""
import functools
import math


def tell(g, gym, heard_fn, cmd):
    """`cmd` as the errand's word to her, once: the same twist again would restart her gait."""
    if g.get('asked') != cmd.get('twist'):
        heard_fn(g, gym, dict(cmd, _errand_cmd=True))


@functools.cache
def _mods():
    """What `steer` reads, imported at its first call and not looked up again each pass."""
    from machine.senses import edge
    from machine.law import latent, modal, se2
    from machine.way import route
    return edge, latent, modal, se2, route


def steer(g, gym, heard_fn, err, wz_dir, speed, off, bend=0.0, aim_m=1.0):
    """Turn in place toward `err` rad while she is off both her aim (`err`) and her way (`off`) past her
    list, standing, or past `route.CONE` as far as her rhythm runs (`latent.GAIN`) toward `speed`; within
    either, walk `speed`: where her way bends at her aim (`bend`, 1/m) on her turn told, pure pursuit's
    to her aim `aim_m` on - 2 sin(err) / aim_m her bend, from her heading - over her stance's share of a
    turn, YAW_RATE at most; else by `err` her heading. Held by her heading round a bend her frame turned
    and her gait not: told 25-40 deg/s so she crabbed 26-46 deg and fell; told the turn, 2-5 deg."""
    edge, latent, modal, se2, route = _mods()
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    turn = min(abs(err), abs(off)) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and bend > 0.0 and speed > 0.0
    err_f = se2.wrap_angle(float(edge.yaw(g['sense'])) + err - float(g['heading']))
    wz = max(-se2.YAW_RATE, min(se2.YAW_RATE, speed * 2.0 * math.sin(err_f) / max(aim_m, 1e-6) / se2.STEER['stance_turn']))
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(edge.yaw(g['sense']) + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, wz)), (True, (speed, 0.0, 0.0)))
    tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})
