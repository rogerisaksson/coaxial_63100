"""Mathematical and physical constants shared across host models, compilers and tools."""
import math

#: Circle, natural base and algebraic constants (IEEE 754 binary64, HALF_SQRT3 rounded as float32).
PI = math.pi
TAU = math.tau
TWO_PI = TAU
HALF_PI = 0.5 * PI
INV_TAU = 1.0 / TAU
E = math.e
LN2 = math.log(2.0)
SQRT2 = math.sqrt(2.0)
INV_SQRT2 = math.sqrt(0.5)
BUTTERWORTH_Q = round(INV_SQRT2, 8)
SQRT3 = math.sqrt(3.0)
HALF_SQRT3 = 0.8660254
INV_SQRT3 = 1.0 / SQRT3

#: Angular and rotational unit conversions.
DEG_TO_RAD = PI / 180.0
RAD_TO_DEG = 180.0 / PI
RAD_S_PER_RPM = TAU / 60.0
RPM_PER_RAD_S = 60.0 / TAU

#: Universal, thermodynamic and material reference constants (SI).
STANDARD_GRAVITY = 9.80665
KELVIN_AT_ZERO_C = 273.15
GAS_CONSTANT = 8.314462618
AIR_GAS_CONSTANT = 287.05
AIR_GAMMA = 1.4
COPPER_TEMPCO_PER_K = 0.00393
