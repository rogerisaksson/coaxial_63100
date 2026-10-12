"""Her walk's form and its price, of a steady walk's measures (`strides.measured`).

    looks.broken(measures)      # [(measure, value, bound)]: what of FORM it does not meet
    looks.priced(measures)      # its energy a metre and what it is off its form, one number
"""
import math

#: Her walk's form on a flat, smooth floor: a condition, never weighed against a fall -
#: (measure, least, most), `strides.measured`'s names. The stance leg a strut, its knee all but
#: straight while her foot bears her alone behind the plumb line; the leg on behind the plumb
#: line; the pelvis rolled up over the standing hip; the toes not back as they lift; her head
#: still and her feet quiet. The landing knee stands 1-3 deg on a take of a woman's
#: (`mocap.py`): its bound comes down as the landing does (docs/TODO.md).
FORM = (('knee behind plumb', None, 10.0), ('knee at landing', None, 30.0),
        ('leg behind plumb', 12.0, None), ('hip over stance', 3.0, None),
        ('toes back at lift', None, 2.0), ('head bob', None, 30.0),
        ('head fore-aft', None, 45.0), ('head aside', None, 35.0), ('strike', None, 450.0))

#: A walk's price (`priced`): its energy a metre, J/m over ENERGY_J_M, and FORM_K a measure's
#: share past its bound - a walk off its form costs more than any energy saves; fallen, FELL.
ENERGY_J_M, FORM_K, FELL = 100.0, 20.0, 1000.0


def broken(measures):
    """[(measure, value, bound)]: FORM's conditions `measures` does not meet; one unmeasured
    is not met."""
    out = []
    for name, least, most in FORM:
        v = measures.get(name, math.nan)
        if v != v or (least is not None and v < least) or (most is not None and v > most):
            out.append((name, v, least if least is not None else most))
    return out


def priced(measures):
    """A steady walk's price: its energy a metre and what it is off its form."""
    if measures.get('fell'):
        return FELL
    off = sum(1.0 if v != v else abs(v - bound) / max(1.0, abs(bound))
              for _name, v, bound in broken(measures))
    return measures.get('energy', math.nan) / ENERGY_J_M + FORM_K * off
