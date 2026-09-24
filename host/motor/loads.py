"""What a motor turns: a propeller, off a measured curve."""


class Propeller:

    """A load that grows with the square of speed, off a measured curve."""

    def __init__(self, k, name='', source=''):
        self.k, self.name, self.source = k, name, source

    def torque(self, omega_electrical, pole_pairs):
        wm = omega_electrical / pole_pairs
        return self.k * wm * wm

    def __repr__(self):
        return '<%s: %.3e N.m/(rad/s)^2>' % (self.name or 'propeller', self.k)


#: The thrust stand `APC20x10E.k` was fitted over: Hobbywing's own 190KV
#: table at 37 V and 25 C, 22 rows of (rpm, shaft torque N.m, input W).
#: Kept whole so the fit can be re-done or argued with rather than taken
#: on trust - and so a model has something to be checked against that it
#: was not built from.
APC20X10E_CURVE = (
    (2534, 0.32, 125.2), (2684, 0.37, 143.2), (2827, 0.41, 162.7),
    (2964, 0.45, 183.9), (3163, 0.51, 219.1), (3359, 0.58, 258.8),
    (3554, 0.65, 303.6), (3750, 0.73, 354.1), (3946, 0.81, 410.7),
    (4144, 0.90, 473.7), (4342, 1.00, 543.5), (4541, 1.10, 620.0),
    (4740, 1.20, 703.7), (4938, 1.32, 794.8), (5134, 1.43, 893.7),
    (5328, 1.56, 1001.1), (5518, 1.69, 1117.7), (5703, 1.82, 1244.1),
    (5883, 1.96, 1380.8), (6058, 2.11, 1527.6), (6226, 2.27, 1683.8),
    (6717, 2.78, 2214.0),
)

#: The APC20x10E the 190KV was tested on, fitted over its 22 points.
#: Worst point 13.2 % off a pure square, which is the propeller and not
#: the fit - efficiency moves along the curve.
APC20x10E = Propeller(
    k=5.143e-6, name='APC20x10E',
    source='least squares over the 190KV sheet, 37 V, 25 C, sea level')
