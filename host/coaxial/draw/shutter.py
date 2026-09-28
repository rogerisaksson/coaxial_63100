"""What a turning can shows through a shutter: a magnet's share of the exposure, dithered."""

#: A 4x4 ordered dither in sixteenths: a blurred part lights the dots whose threshold its share of
#: the exposure passes - fixed to the screen's dots, so a band at speed stands still rather than
#: sparkling.
BAYER = ((0, 8, 2, 10), (12, 4, 14, 6), (3, 11, 1, 9), (15, 7, 13, 5))

#: Where a magnet sits in its pitch: the gap between two is a tenth either side.
MAGNET_FROM, MAGNET_TO = 0.1, 0.9


def exposed(place, blur, pole):
    """The share of the exposure a `pole` magnet (0 north, 1 south) covers `place` - pitches
    from the can's zero - while the can turns `blur` pitches: its span of every second pitch
    integrated over [place, place + blur], over blur; the magnet itself where blur is 0."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    width = MAGNET_TO - MAGNET_FROM

    def swept(x):
        x = x - pole
        k = np.floor(x / 2.0)
        return k * width + np.clip(x - 2.0 * k - MAGNET_FROM, 0.0, width)
    if abs(blur) < 1e-9:
        x = place - pole
        at = x - 2.0 * np.floor(x / 2.0)
        return ((MAGNET_FROM <= at) & (at < MAGNET_TO)).astype(float)
    return (swept(place + blur) - swept(place)) / blur


def threshold(x, y):
    """What a dot's share must pass to light it, by its place on the screen."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    return (np.asarray(BAYER)[np.asarray(y, int) % 4, np.asarray(x, int) % 4] + 0.5) / 16.0
