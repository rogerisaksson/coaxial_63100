"""One rule both character renderers need: which cell a coordinate lands in.

Its own file because it has two callers and no owner - `orientation` draws
an attitude and `dial` draws a shaft, and rounding belongs to neither.
"""
import math


def cell(value):
    """`value` to the nearest cell, halves always upward.

    Not round(): Python rounds halves to even, so 15.5 and 16.5 both land on
    16 and two adjacent positions collide. Measured, that wrote a label as
    "o x a  3 0 0".
    """
    return int(math.floor(value + 0.5))
