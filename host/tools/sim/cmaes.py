"""CMA-ES over a few constants, each scaled to its span: the scoreboards' search."""
import json
import math
import time


def search(pool, spans, generations, lam, log, runs, now, sigma=0.08, start=None):
    """CMA-ES over `spans` {name: (low, high)}, each scaled to its span, from the constants as
    they are (`now(name)`, or `start`, {name: value}), `sigma` of a span its first step: a walk
    that looks right is refined, not searched away - begun from the spans' middles at a quarter,
    the searches found tiptoeing (2026-09-28). `runs(pool, candidates)` scores them: [(cost,
    held, stir, results)]; every candidate a line of `log`."""
    import numpy as np
    start = start or {}
    names = list(spans)
    dim = len(names)
    mean = np.array([min(1.0, max(0.0, ((start[n] if n in start else now(n)) - spans[n][0])
                                  / (spans[n][1] - spans[n][0]))) for n in names])
    mu = lam // 2
    weights = math.log(mu + 0.5) - np.log(np.arange(1, mu + 1))
    weights /= weights.sum()
    mueff = 1.0 / (weights ** 2).sum()
    cc, cs = (4 + mueff / dim) / (dim + 4 + 2 * mueff / dim), (mueff + 2) / (dim + mueff + 5)
    c1 = 2 / ((dim + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((dim + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0.0, math.sqrt((mueff - 1) / (dim + 1)) - 1) + cs
    chi = math.sqrt(dim) * (1 - 1 / (4 * dim) + 1 / (21 * dim * dim))
    pc, ps, cov = np.zeros(dim), np.zeros(dim), np.eye(dim)
    rng = np.random.default_rng(7)
    best = (math.inf, None)
    for gen in range(generations):
        began = time.time()
        vals, vecs = np.linalg.eigh(cov)
        root = vecs @ np.diag(np.sqrt(np.maximum(vals, 1e-20)))
        xs = np.clip(mean + sigma * rng.standard_normal((lam, dim)) @ root.T, 0.0, 1.0)
        cands = [{n: float(spans[n][0] + x[i] * (spans[n][1] - spans[n][0]))
                  for i, n in enumerate(names)} for x in xs]
        got = runs(pool, cands)
        costs = np.array([g[0] for g in got])
        for values, (cost, held, stir, _r) in zip(cands, got):
            log.write(json.dumps({'gen': gen, 'values': values, 'cost': cost, 'held': held,
                                  'stir': stir}) + '\n')
            if cost < best[0]:
                best = (cost, values)
        log.flush()
        order = np.argsort(costs)
        old = mean
        mean = weights @ xs[order[:mu]]
        step = (mean - old) / sigma
        inv = vecs @ np.diag(1 / np.sqrt(np.maximum(vals, 1e-20))) @ vecs.T
        ps = (1 - cs) * ps + math.sqrt(cs * (2 - cs) * mueff) * inv @ step
        hsig = (np.linalg.norm(ps) / math.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) / chi
                < 1.4 + 2 / (dim + 1))
        pc = (1 - cc) * pc + hsig * math.sqrt(cc * (2 - cc) * mueff) * step
        art = (xs[order[:mu]] - old) / sigma
        cov = ((1 - c1 - cmu) * cov + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * cov)
               + cmu * art.T @ np.diag(weights) @ art)
        sigma *= math.exp((cs / damps) * (np.linalg.norm(ps) / chi - 1))
        print('gen %2d  best %.2f  median %.2f  sigma %.3f  %.0f s  | best so far %.2f %s' % (
            gen, costs.min(), float(np.median(costs)), sigma, time.time() - began, best[0],
            {k: round(v, 3) for k, v in (best[1] or {}).items()}), flush=True)
    return best
