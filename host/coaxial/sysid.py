"""Recover a machine's constants from what the board recorded."""
import math

# One definition each, where they already lived: the transform's turn is the
# observer's, and the channel names are the commissioning's.
from .commission import PHASES                              # noqa: F401
from .errors import RigError
from .motor import Parameters
GATES = (('TIM1_CH1/PWMUH', 'TIM1_CH1N/PWMUL'),
         ('TIM1_CH2/PWMVH', 'TIM1_CH2N/PWMVL'),
         ('TIM1_CH3/PWMWH', 'TIM1_CH3N/PWMWL'))


def _numpy():
    try:
        import numpy
    except ImportError:
        raise RigError('system identification needs numpy - '
                       '`pip install numpy`') from None
    return numpy


def to_dq(values, theta, amplitude_invariant=True):
    """Three phase quantities to (d, q) at electrical angle `theta`."""
    np = _numpy()
    a, b, c = (np.asarray(v, dtype=float) for v in values)
    theta = np.asarray(theta, dtype=float)
    alpha = (2.0 * a - b - c) / 3.0
    beta = (b - c) / math.sqrt(3.0)
    if not amplitude_invariant:
        alpha *= math.sqrt(1.5)
        beta *= math.sqrt(1.5)
    cos, sin = np.cos(theta), np.sin(theta)
    return alpha * cos + beta * sin, beta * cos - alpha * sin


def phase_voltages(duties, vdc):
    """Phase voltages from the six gate duties and the link."""
    np = _numpy()
    legs = np.asarray([np.asarray(d, dtype=float) for d in duties])
    vdc = np.asarray(vdc, dtype=float)
    return (legs - legs.mean(axis=0)) * vdc


def _derivative(x, t):
    """dx/dt on an unevenly sampled series, centred where it can be."""
    np = _numpy()
    return np.gradient(np.asarray(x, dtype=float), np.asarray(t, dtype=float))


def identify(vd, vq, id_, iq, omega, t, min_condition=1e-6):
    """(R, Ld, Lq, lambda) by least squares, with what it is worth."""
    np = _numpy()
    vd, vq = np.asarray(vd, float), np.asarray(vq, float)
    id_, iq = np.asarray(id_, float), np.asarray(iq, float)
    omega, t = np.asarray(omega, float), np.asarray(t, float)

    did, diq = _derivative(id_, t), _derivative(iq, t)
    zero = np.zeros_like(id_)
    one = np.ones_like(id_)

    # Two equations a sample, stacked. Columns: R, Ld, Lq, lambda.
    top = np.column_stack([id_, did, -omega * iq, zero])
    bottom = np.column_stack([iq, omega * id_, diq, omega * one])
    matrix = np.vstack([top, bottom])
    target = np.concatenate([vd, vq])

    scale = np.linalg.norm(matrix, axis=0)
    scale[scale == 0.0] = 1.0
    singular = np.linalg.svd(matrix / scale, compute_uv=False)
    condition = float(singular[-1] / singular[0]) if singular[0] else 0.0


    if condition < min_condition:
        raise RigError(
            'this run cannot separate the four constants - condition %.2e '
            'against a floor of %.0e. omega has to VARY for lambda to come '
            'out of it, and the currents have to change fast somewhere for '
            'the inductances to come out of R. A steady run identifies R '
            'and nothing else.' % (condition, min_condition))

    fit, _residues, _rank, _sv = np.linalg.lstsq(matrix, target, rcond=None)
    r, ld, lq, lam = (float(v) for v in fit)
    predicted = matrix @ fit
    rms = float(np.sqrt(np.mean((target - predicted) ** 2)))

    # PER PARAMETER, because one number for the whole fit hides the case that
    # matters.
    freedom = max(1, matrix.shape[0] - matrix.shape[1])
    variance = float(np.sum((target - predicted) ** 2)) / freedom
    try:
        covariance = variance * np.linalg.inv(matrix.T @ matrix)
        errors = np.sqrt(np.abs(np.diag(covariance)))
    except np.linalg.LinAlgError:
        errors = np.full(4, float('inf'))
    names = ('r', 'ld', 'lq', 'lam')
    got = {'r': r, 'ld': ld, 'lq': lq, 'lam': lam,
           'condition': condition, 'residual_v': rms,
           'samples': int(len(id_))}
    # Relative, since that is what a reader compares: 5 % on an inductance is
    # a number, 300 % is a column the run did not excite.
    got['uncertainty'] = {
        name: (float(err / abs(value)) if value else float('inf'))
        for name, value, err in zip(names, fit, errors)}
    got['trusted'] = {name: got['uncertainty'][name] < 0.10 for name in names}
    return got


def from_frame(frame, theta, poles, vdc='DC bus (V)', name='identified'):
    """Identify straight off a `daq.frame()`."""
    np = _numpy()

    currents = [frame[c].to_numpy() for c in
                ('%s (A)' % p for p in PHASES)]
    duties = [frame[high].to_numpy() for high, _low in GATES]
    link = (frame[vdc].to_numpy() if vdc in frame
            else np.full(len(frame), 31.0))

    volts = phase_voltages(duties, link)
    vd, vq = to_dq(volts, theta)
    id_, iq = to_dq(currents, theta)

    seconds = np.asarray(frame.index.to_numpy(), dtype='datetime64[ns]')
    seconds = (seconds - seconds[0]) / np.timedelta64(1, 's')
    omega = _derivative(np.unwrap(np.asarray(theta, float)), seconds)

    got = identify(vd, vq, id_, iq, omega, seconds)
    return Parameters(name=name, r=got['r'], ld=got['ld'], lq=got['lq'],
                      lam=got['lam'], poles=poles, measured=True,
                      source='least squares over %d records, condition %.1e, '
                             'residual %.3f V'
                             % (got['samples'], got['condition'],
                                got['residual_v'])), got
