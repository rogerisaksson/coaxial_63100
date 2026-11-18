"""What test_libm holds the robot's libm to, laid as data: Annex F's values and the powers a double holds exactly."""
import math

INF, NAN = math.inf, math.nan

#: Annex F's values (F.10): (function, arguments, the value to the bit; NaN any NaN).
ANNEX = [('sin', (0.0,), 0.0), ('sin', (-0.0,), -0.0), ('sin', (INF,), NAN), ('sin', (-INF,), NAN), ('sin', (NAN,), NAN),
         ('sin', (5e-324,), 5e-324), ('sin', (-1e-300,), -1e-300),
         ('cos', (0.0,), 1.0), ('cos', (-0.0,), 1.0), ('cos', (INF,), NAN), ('cos', (NAN,), NAN),
         ('tan', (0.0,), 0.0), ('tan', (-0.0,), -0.0), ('tan', (INF,), NAN), ('tan', (NAN,), NAN), ('tan', (-1e-300,), -1e-300),
         ('exp', (0.0,), 1.0), ('exp', (-0.0,), 1.0), ('exp', (INF,), INF), ('exp', (-INF,), 0.0), ('exp', (NAN,), NAN),
         ('exp', (710.0,), INF), ('exp', (1e300,), INF), ('exp', (-746.0,), 0.0), ('exp', (-1e300,), 0.0),
         ('exp', (709.782712893384,), 1.7976931348622732e308), ('exp', (1.0,), math.e),
         ('expm1', (0.0,), 0.0), ('expm1', (-0.0,), -0.0), ('expm1', (INF,), INF), ('expm1', (-INF,), -1.0),
         ('expm1', (NAN,), NAN), ('expm1', (5e-324,), 5e-324), ('expm1', (-40.0,), -1.0), ('expm1', (710.0,), INF),
         ('tanh', (0.0,), 0.0), ('tanh', (-0.0,), -0.0), ('tanh', (INF,), 1.0), ('tanh', (-INF,), -1.0), ('tanh', (NAN,), NAN),
         ('tanh', (5e-324,), 5e-324), ('tanh', (-1e-300,), -1e-300), ('tanh', (40.0,), 1.0), ('tanh', (-1e300,), -1.0),
         ('log', (0.0,), -INF), ('log', (-0.0,), -INF), ('log', (1.0,), 0.0), ('log', (-1.0,), NAN), ('log', (-INF,), NAN),
         ('log', (INF,), INF), ('log', (NAN,), NAN), ('log', (math.e,), 1.0), ('log', (-5e-324,), NAN),
         ('arctan', (0.0,), 0.0), ('arctan', (-0.0,), -0.0), ('arctan', (INF,), math.pi / 2), ('arctan', (-INF,), -math.pi / 2),
         ('arctan', (NAN,), NAN), ('arctan', (1.0,), math.pi / 4), ('arctan', (-5e-324,), -5e-324),
         ('arcsin', (0.0,), 0.0), ('arcsin', (-0.0,), -0.0), ('arcsin', (1.0,), math.pi / 2), ('arcsin', (-1.0,), -math.pi / 2),
         ('arcsin', (1.0000000000000002,), NAN), ('arcsin', (INF,), NAN), ('arcsin', (NAN,), NAN), ('arcsin', (5e-324,), 5e-324),
         ('arccos', (1.0,), 0.0), ('arccos', (-1.0,), math.pi), ('arccos', (0.0,), math.pi / 2), ('arccos', (-0.0,), math.pi / 2),
         ('arccos', (1.0000000000000002,), NAN), ('arccos', (-INF,), NAN), ('arccos', (NAN,), NAN),
         ('hypot', (INF, NAN), INF), ('hypot', (NAN, -INF), INF), ('hypot', (NAN, 1.0), NAN), ('hypot', (-3.0, 4.0), 5.0),
         ('hypot', (0.0, -0.0), 0.0), ('hypot', (-2.5, 0.0), 2.5), ('hypot', (1e308, 1e308), 1.4142135623730951e308),
         ('hypot', (3e-320, 4e-320), 5e-320)]
for _y, _sy in ((0.0, 1.0), (-0.0, -1.0)):
    ANNEX += [('arctan2', (_y, -0.0), _sy * math.pi), ('arctan2', (_y, 0.0), _y), ('arctan2', (_y, -2.0), _sy * math.pi),
              ('arctan2', (_y, 2.0), _y), ('arctan2', (_sy * 2.0, -INF), _sy * math.pi), ('arctan2', (_sy * 2.0, INF), _y),
              ('arctan2', (_sy * INF, 3.0), _sy * math.pi / 2), ('arctan2', (_sy * INF, -INF), _sy * 3 * math.pi / 4),
              ('arctan2', (_sy * INF, INF), _sy * math.pi / 4)]
ANNEX += [('arctan2', (-2.0, 0.0), -math.pi / 2), ('arctan2', (-2.0, -0.0), -math.pi / 2), ('arctan2', (2.0, 0.0), math.pi / 2),
          ('arctan2', (NAN, 1.0), NAN), ('arctan2', (1.0, NAN), NAN), ('arctan2', (1.0, 1.0), math.pi / 4)]
#: pow's (F.10.4.4), as power(x, y): zeros and infinities by y's sign and whether it is an odd whole number, 1 at y
#: = 0 and at x = 1 whatever the other, a NaN for a negative x at a y not whole; whole from 2^53 on, each even.
ANNEX += [('power', args, want) for args, want in (
    ((0.0, -3.0), INF), ((-0.0, -3.0), -INF), ((0.0, -2.0), INF), ((-0.0, -2.0), INF), ((-0.0, -2.5), INF), ((0.0, 3.0), 0.0),
    ((-0.0, 3.0), -0.0), ((-0.0, 2.0), 0.0), ((-0.0, 0.5), 0.0), ((0.0, -INF), INF), ((-0.0, -INF), INF), ((0.0, INF), 0.0),
    ((-1.0, INF), 1.0), ((-1.0, -INF), 1.0), ((1.0, NAN), 1.0), ((1.0, INF), 1.0), ((NAN, 0.0), 1.0), ((NAN, -0.0), 1.0),
    ((INF, 0.0), 1.0), ((-2.0, 0.5), NAN), ((-2.0, -1.5), NAN), ((-5e-324, 0.5), NAN), ((0.5, -INF), INF), ((2.0, -INF), 0.0),
    ((0.5, INF), 0.0), ((2.0, INF), INF), ((-0.5, INF), 0.0), ((-2.0, INF), INF), ((-2.0, -INF), 0.0), ((-INF, -3.0), -0.0),
    ((-INF, -2.0), 0.0), ((-INF, -2.5), 0.0), ((-INF, 3.0), -INF), ((-INF, 2.0), INF), ((-INF, 2.5), INF), ((INF, -1.0), 0.0),
    ((INF, 0.5), INF), ((NAN, 1.0), NAN), ((2.0, NAN), NAN), ((-1.0, NAN), NAN), ((-2.0, 3.0), -8.0), ((-2.0, 2.0), 4.0),
    ((-2.0, -3.0), -0.125), ((2.0, 1024.0), INF), ((2.0, -1075.0), 0.0), ((2.0, -1074.0), 5e-324), ((0.5, 1e300), 0.0),
    ((-1.0, 9007199254740992.0), 1.0), ((-1.0, 9007199254740991.0), -1.0), ((-1.0, 4503599627370497.0), -1.0),
    ((-1.0, 2251799813685248.5), NAN), ((-1.0, 1e300), 1.0), ((1.0000000000000002, 1e300), INF),
    ((0.9999999999999999, 1e300), 0.0), ((1e300, 1e300), INF), ((1e-300, 1e300), 0.0), ((1e300, -1e300), 0.0))]

#: Powers a double holds exactly, (x, y, the value): 2^k across the doubles, whole bases while their power is a whole
#: number under 2^53, roots of squares, and x^1.
EXACT = ([(2.0, float(k), math.ldexp(1.0, k)) for k in range(-1074, 1024, 13)]
         + [(float(b), float(k), float(b ** k)) for b in (3, 5, 6, 7, 10, 12, 4097) for k in range(40) if b ** k < 2 ** 53]
         + [(float(b * b), 0.5, float(b)) for b in (2, 3, 10, 12345, 94906265)]
         + [(16.0, 0.25, 2.0), (0.25, -0.5, 2.0), (1e22, 0.5, 1e11), (0.5, 1074.0, 5e-324), (8.0, -1.0, 0.125)]
         + [(v, 1.0, v) for v in (3.0, 0.1, 1e-310, 5e-324, 1.7976931348623157e308, 123456.789)])
