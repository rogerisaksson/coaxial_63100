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


def _away(a, b):
    """rad outside the minor arc between `a` and `b` from 0 (`0` inside it)."""
    return min(abs(a), abs(b)) * float(a * b > 0.0 or abs(a) + abs(b) > math.pi)


def steer(g, gym, heard_fn, err, wz_dir, speed, off, aim_m=1.0, pushing=False):
    """Turn in place toward `err` rad while she lies outside the minor arc between her aim (`err`) and
    her way (`off`) past her list, standing, or past `route.CONE` as far as her rhythm runs
    (`latent.GAIN`) toward `speed` - from her walking `heading` while held (`has_heading`), else her
    pelvis -; within it, walk `speed` on her turn told, pure pursuit's to her aim `aim_m` on - 2 sin(err)
    / aim_m her bend, from her heading - over her stance's share of a turn, YAW_RATE at most; `pushing`
    through a door, by `err` her heading held instead: her turn told, her heading follows her pelvis and
    the panel's shove turned her in the doorway. Held by her heading to her aim's bearing on a straight
    leg too her frame turned 1.6 rad/s (`se2.twisted`'s slew) and her gait not: she walked 0.3-0.65 rad
    askew of it, and of her errand's stops from her walk's speed 64 % fell where she went 0.15 m/s or more
    across her heading for 16 % under it - a straight walk's stop none of 33; round a bend, told 25-40
    deg/s, she crabbed 26-46 deg and fell; told the turn, 2-5 deg."""
    edge, latent, modal, se2, route = _mods()
    pyaw = float(edge.yaw(g['sense']))
    w = min(1.0, float(g['st'][latent.NOW][latent.GAIN])) * float(speed > 0.0)
    dyaw = w * se2.wrap_angle(float(g['heading']) - pyaw) if g.get('has_heading') else 0.0
    turn = _away(se2.wrap_angle(err - dyaw), se2.wrap_angle(off - dyaw)) > (1.0 - w) * modal.LIST_RAD + w * route.CONE
    arc = not turn and speed > 0.0 and not pushing
    err_f = se2.wrap_angle(pyaw + err - float(g['heading']))
    wz = max(-se2.YAW_RATE, min(se2.YAW_RATE, speed * 2.0 * math.sin(err_f) / max(aim_m, 1e-6) / se2.STEER['stance_turn']))
    steer = float(not turn) * float(not arc)
    g['has_heading'] = steer * float(speed > 0.0)
    g['target_heading'] = se2.wrap_angle(pyaw + err) * steer + float(g.get('target_heading', 0.0)) * (1.0 - steer)
    twists = ((turn, (0.0, 0.0, se2.YAW_RATE * wz_dir)), (arc, (speed, 0.0, wz)), (True, (speed, 0.0, 0.0)))
    tell(g, gym, heard_fn, {'twist': next(t for on, t in twists if on)})
