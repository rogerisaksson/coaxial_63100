"""A PMSM's constants recovered from its dq voltages and currents: least squares."""


def _derivative(x, t):
    """dx/dt on an unevenly sampled series, centred where it can be."""
    import numpy as np
    return np.gradient(np.asarray(x, dtype=float), np.asarray(t, dtype=float))


def identify(vd, vq, id_, iq, omega, t, min_condition=1e-6):
    """(R, Ld, Lq, lambda) by least squares, with what it is worth."""
    import numpy as np
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
        raise ValueError(
            'this run cannot separate the four constants - condition %.2e '
            'against a floor of %.0e. omega has to VARY for lambda to come '
            'out of it, and the currents have to change fast somewhere for '
            'the inductances to come out of R. A steady run identifies R '
            'and nothing else.' % (condition, min_condition))

    fit, _residues, _rank, _sv = np.linalg.lstsq(matrix, target, rcond=None)
    r, ld, lq, lam = (float(v) for v in fit)
    predicted = matrix @ fit
    rms = float(np.sqrt(np.mean((target - predicted) ** 2)))

    # Per parameter: one number for the whole fit hides the case that
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
    # Relative, since that is what a reader compares: 5 % on an inductance is a
    # number, 300 % is a column the run did not excite.
    got['uncertainty'] = {
        name: (float(err / abs(value)) if value else float('inf'))
        for name, value, err in zip(names, fit, errors)}
    got['trusted'] = {name: got['uncertainty'][name] < 0.10 for name in names}
    return got
