"""Moved from host/machine/identify/wear.py: serviced - nothing in the tree named it."""


def serviced(w):
    """A service done: the drifts' trends start anew from here."""
    w['serviced_at'] = w['hours']
