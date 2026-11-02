"""Moved from host/terminal/stand.py: SWELL_BASE - nothing in the tree named it."""


#: render()'s zoom 1.0 fits the bounding sphere at any attitude, which in
#: the box is 56% of its width - measured, and 2.0 is the first zoom
#: that reaches every edge. The envelope is in shares of the box, so
#: it rides on this base: 1.2 lands at 2.16, past the box, so the
#: board clips into its frame; 0.75 at 1.35, four fifths of it.
SWELL_BASE = 1.8
