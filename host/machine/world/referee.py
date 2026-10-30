"""Her fall judged, the referee's: what of her touches her ground, beside her law and not of it.

    fallen = referee.fallen(g, world)                # a part of her above her shins on her ground, untold
    paired, averaged, address = referee.passing()    # wbc_pass.c's referee and the page's averages, bound once
"""
import functools

from machine.world import floor
from machine.senses import edge
from machine.body import figure
from machine.law import modal
from machine.senses import observer


def fallen(g, world):
    """Fallen, the referee's: a part of her above her shins - thighs, pelvis, trunk, head, arms,
    hands - touches her ground (`floor.grounds`) under her knees' height over her floor
    (`modal.FLOOR`), unless her text tells it bears her (`observer.parts`): by her doing's name she was fallen
    sat on her hands; a tilt past 45 deg on a bed was a fall, thrown back onto it she was down at 3.7 s;
    touched at any height, her hand on the loft by the chair, 0.75 m up, felled 25 of 33 errands."""
    m, d = world.model, world.data
    paired, _averaged, address = passing()
    if g.get('_falls_by', (None,))[0] is not m:
        np = world._np
        down = {m.body(n).id for n in edge.SKIN if not n.endswith(('_shank', '_foot', '_toes'))}
        masks = (np.isin(m.geom_bodyid, list(down)), np.isin(m.geom_bodyid, list(floor.grounds(m))))
        g['_falls_by'], g['_falls_down'] = (m, masks, [address(mask) for mask in masks]), down
    # her law lays g['told'] anew a law pass, written nowhere after (`motion.control`): said once a law pass
    if g.get('_said_of', (None,))[0] is not g['told']:
        g['_said_of'] = (g['told'], observer.said(g['told']))
    told = g['_said_of'][1]
    # the most told her entry long, laid anew where her law tells anew or her entry changes: between, the
    # same told's max over it is itself (nan's too), its numpy and its tuple 5 us a pass for nothing
    entry, kept = g.get('active_entry'), g.get('_told_with')
    if kept is None or kept[0] is not told or kept[1] is not entry:
        was = g.get('_told_by', (object(), told))
        g['_told_by'] = (entry, world._np.maximum(told, was[1]) if was[0] is entry else told)
        g['_told_with'] = (told, entry, tuple(g['_told_by'][1] > 0.0))
    on = g['_told_with'][2]
    if g.get('_falls_of') != on:
        g['_falls_of'], g['_falls_by'][1][0][:] = on, world._np.isin(m.geom_bodyid, observer.unborne(m, on, g['_falls_down']))
    contact = d.contact
    geom, pos = contact.geom, contact.pos
    below = float(g['fr'][modal.FLOOR]) + figure.ANKLE_H + figure.SHANK
    return bool(paired(address(geom), address(pos), geom.strides[0], d.ncon, *g['_falls_by'][2], below))


@functools.cache
def passing():
    """wbc_pass.c's referee and the page's averages, bound once; `tools.cores.wbc.address`."""
    from tools.cores import wbc as cores
    lib = cores.library()
    return lib.wbh_pass_paired, lib.wbh_pass_averaged, cores.address
