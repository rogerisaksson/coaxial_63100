"""Her movements by their orthopaedic names, read from static arrays (`term_rows`).

    kind, sign, low, high = terms.find('knee', 'flexion')   # deg; None: she has it not
    name = terms.joint(limb, kind)                          # a limb's joint; the trunk's, its own
    term_rows.TOWARD, term_rows.ABSENT                      # told toward a side; named, no joint

A part and a movement name a joint kind and the sign its angle takes the movement's own way;
its amount's stops with it, None where nothing stops it. A movement toward a side is told for
limb 0's and takes a limb's mirror (`reach_rows.LIMBS`) for another's. A limb is an index:
which joint of hers is its knee is data, laid from her figure (tools/cores/terms.py).
"""
from machine.term_rows import LIMBS, TERMS


def find(part, term):
    """(joint kind, sign, low, high) of a movement of a part, deg - or None: no driven joint of
    hers makes it."""
    return TERMS.get((part, term))


def joint(limb, kind):
    """The joint of that kind of a limb's index; a kind no limb has - the trunk's, the neck's -
    is its own joint."""
    return LIMBS[limb].get(kind, kind)
