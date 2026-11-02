"""Moved from host/coaxial/draw/ascii3d.py: BAND_FIRST - nothing in the tree named it."""


#: Why `rasterise` asks whether a triangle touches its band before it asks
#: anything else: every worker walks the whole model, so whatever is done
#: before that question is paid for by every worker for every triangle,
#: however far it is from that worker's rows. Measured at 150x44 - asking it
#: last, after the winding, the area and the x bounds, cost 23 ms of each
#: worker's 31 ms of fixed overhead and held the parallel ceiling at 3.5x.
#: Asked first, the fixed cost is 14.5 ms and the ceiling is 6x.
BAND_FIRST = True
