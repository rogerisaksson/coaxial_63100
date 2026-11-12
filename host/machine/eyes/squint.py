"""Her right eye's turn past her calibration's model of it, told look by look from her pictures.

    sq = squint.Squint()                            # what she knows of it: her church's, nothing past it
    right_R = told_R @ sq.turn()                    # her right eye's turn as she holds it now
    sq.told(laid, a, b, disp, kept, left_R, right_R, line)   # a look's matches told on it

Her eyes' line is held to their build (`gimbal.place`): what a squint moves is her right eye's turn off her left's,
`w` in its own frame. A point matched in both of her pictures lies on one plane with her eyes' line - its rays'
triple product with it nought, (q x b) . (R p) -, a turn of the model by `d` moving that by (R p) x (q x b) . d.
Points a look: in her left eye's laid picture the most textured of each TILE (Shi-Tomasi's least eigenvalue), each
matched in her right eye's laid picture about her stereo's disparity REACH rows and SIDE columns either way by
normalised correlation, placed within a pixel by a parabola each way, read back on each eye's own picture (`Laid.at`)
as its ray. Weighed against what she knew before the look - a Kalman's step on a turn her pushrods play between
looks -, robust (Huber's, HUBER times a misfit's spread), STEPS of Gauss-Newton's a look. Pitch and roll misfit her
rows at once; her squint across, her vergence, shows only at her pictures' corners: held to her church's word on it
and her pushrods' play, it moves as her pictures show it and no further.
"""
import math

from machine.eyes import epipolar, gimbal
from machine.eyes.stereo import H, W, WINDOW, _boxed, _grey, _np, indices

#: A point a TILE x TILE of her left eye's laid picture, px; matched REACH rows and SIDE columns either way of her
#: stereo's match (`stereo.CENSUS`' rows' half), its window her stereo's (`stereo.WINDOW`).
TILE, REACH, SIDE = 32, 3, 2
#: Huber's constant, a misfit's spread's (95 % of least squares' efficiency on normal misfits); Gauss-Newton's steps a look.
HUBER, STEPS = 1.345, 3
#: Her pictures blurred by a normal SMOOTH px wide for their matches' places (`_smoothed`), px.
SMOOTH = 1.0


def _vee(R):
    """(3,) the rotation vector of `R` (the log on SO(3))."""
    np = _np()
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    a = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return 0.5 * v if a < 1e-8 else a / (2.0 * math.sin(a)) * v


def _known():
    """((3, 3) what her church told her of her right eye's turn off her left's, as a covariance in its frame - each
    axis its squint's CHURCH share, rad squared -, (3, 3) what her pushrods' play moves of it past her word on it when
    her servos turn: each axis' play's CHURCH share)."""
    np = _np()
    sq = {s: np.radians(np.abs(np.asarray(gimbal.SQUINT[s]))) * abs(gimbal.CHURCH[s]) for s in ('left', 'right')}
    # her camera's x the pitch's axis (down), its y the across's: each gimbal's play she tells (`gimbal.played`)
    play = {s: np.radians(np.asarray(gimbal.PLAY[s])) * abs(gimbal.CHURCH[s]) for s in ('left', 'right')}
    both = np.array([play['left'][1] + play['right'][1], play['left'][0] + play['right'][0], 0.0])
    return np.diag((sq['left'] + sq['right']) ** 2), np.diag(both ** 2)


def _rays(at, rows, cols):
    """(n, 3) unit: the rays in an eye's frame of the places `at` ((rows, columns) in fixed point, `Laid.at`) holds at
    the planes' (`rows`, `cols`), bilinear between its pixels."""
    np = _np()
    v, u = (np.asarray(a, float) / float(1 << epipolar.BITS) for a in at)
    r0, c0 = np.clip(np.floor(rows).astype(np.int64), 0, H - 2), np.clip(np.floor(cols).astype(np.int64), 0, W - 2)
    fr, fc = rows - r0, cols - c0

    def at_(a):
        return ((1 - fr) * (1 - fc) * a[r0, c0] + (1 - fr) * fc * a[r0, c0 + 1] + fr * (1 - fc) * a[r0 + 1, c0]
                + fr * fc * a[r0 + 1, c0 + 1])
    d = np.stack([(at_(u) - W / 2.0) / epipolar.F, (H / 2.0 - at_(v)) / epipolar.F, -np.ones(len(rows))], axis=1)
    return d / np.linalg.norm(d, axis=1, keepdims=True)


def _smoothed(g, xp=None):
    """`g` (H, W) blurred by a normal SMOOTH px wide each way, three of them its reach, edges held: a place read between
    a picture's pixels read on what its pixels hold - sharp, a bilinear reading of it drew rows to whole pixels -; on
    array module `xp` (numpy's without)."""
    np = _np()
    xp = xp or np
    r = int(math.ceil(3.0 * SMOOTH))
    k = np.exp(-0.5 * (np.arange(-r, r + 1) / SMOOTH) ** 2)
    k = k / k.sum()
    p, rows, out = xp.pad(g, r, mode='edge'), xp.zeros((g.shape[0], g.shape[1] + 2 * r)), xp.zeros(g.shape)
    for i in range(2 * r + 1):
        rows += float(k[i]) * p[i:i + g.shape[0], :]
    for i in range(2 * r + 1):
        out += float(k[i]) * rows[:, i:i + g.shape[1]]
    return out


def matches(laid, a, b, disp, kept):
    """((n,) rows, (n,) columns of her left eye's laid picture, (n,) rows, (n,) columns of her right's they matched,
    each within a pixel) of a look's laid pictures `a`, `b` (H, W, 3) matched at `disp` where `kept`: the pictures'
    blur and texture on `laid`'s array module (the card's where one answers), each tile's match on the CPU."""
    np, xp = _np(), laid.xp
    gl, gr = (_smoothed(_grey(x, xp).astype(float), xp) for x in (a, b))
    r0 = WINDOW // 2
    gx, gy = xp.zeros_like(gl), xp.zeros_like(gl)
    gx[:, 1:-1], gy[1:-1, :] = (gl[:, 2:] - gl[:, :-2]) / 2.0, (gl[2:, :] - gl[:-2, :]) / 2.0
    sxx, syy, sxy = (_boxed(g, r0, xp) for g in (gx * gx, gy * gy, gx * gy))
    least = epipolar.host((sxx + syy) / 2.0 - xp.sqrt(((sxx - syy) / 2.0) ** 2 + sxy * sxy))
    gl, gr = epipolar.host(gl), epipolar.host(gr)
    rows, cols = indices(np)
    right = cols - np.rint(disp).astype(np.int64)
    room = (kept & laid.on[0] & (rows >= r0 + REACH + 1) & (rows < H - r0 - REACH - 2) & (right >= r0 + SIDE + 1)
            & (right < W - r0 - SIDE - 2) & (cols >= r0) & (cols < W - r0))
    tiled = np.where(room, least, -np.inf).reshape(H // TILE, TILE, W // TILE, TILE).transpose(0, 2, 1, 3)
    k = tiled.reshape(H // TILE, W // TILE, -1).argmax(axis=-1)
    tr, tc = np.indices(k.shape)
    pr, pc = (tr * TILE + k // TILE).ravel(), (tc * TILE + k % TILE).ravel()
    sel = room[pr, pc]
    pr, pc = pr[sel], pc[sel]
    o = np.arange(-r0, r0 + 1)
    lw = gl[pr[:, None, None] + o[None, :, None], pc[:, None, None] + o[None, None, :]]
    lw = lw - lw.mean(axis=(1, 2), keepdims=True)
    dy, dx = np.arange(-REACH, REACH + 1), np.arange(-SIDE, SIDE + 1)
    yy = pr[:, None, None, None, None] + dy[None, :, None, None, None] + o[None, None, None, :, None]
    xx = right[pr, pc][:, None, None, None, None] + dx[None, None, :, None, None] + o[None, None, None, None, :]
    # a row of tiles at a time, its windows in a core's caches: all of a look's at once were tens of MB a temporary
    step = W // TILE
    ncc = np.concatenate([_ncc(gr, lw[s:s + step], yy[s:s + step], xx[s:s + step]) for s in range(0, len(pr), step)]
                         or [np.zeros((0, len(dy), len(dx)))])
    best = ncc.reshape(len(pr), len(dy) * len(dx)).argmax(axis=1)
    iy, ix = best // len(dx), best % len(dx)
    i = np.flatnonzero((iy > 0) & (iy < len(dy) - 1) & (ix > 0) & (ix < len(dx) - 1))
    y, x = (pr[i] + dy[iy[i]]).astype(float), (right[pr[i], pc[i]] + dx[ix[i]]).astype(float)
    y, x, held = _followed(gr, lw[i], y, x, _slopes(disp, kept, pr[i], pc[i], o))
    return pr[i][held], pc[i][held], y[held], x[held]


def _ncc(gr, lw, yy, xx):
    """(n, rows, columns) each left window `lw` (n, w, w, its mean off) matched on her right picture `gr` at the
    windows `yy`, `xx` reach: their normalized cross-correlation."""
    rw = gr[yy, xx]
    rw = rw - rw.mean(axis=(3, 4), keepdims=True)
    return (rw * lw[:, None, None]).sum(axis=(3, 4)) / _np().maximum(
        _np().sqrt((rw * rw).sum(axis=(3, 4)) * (lw * lw).sum(axis=(1, 2))[:, None, None]), 1e-12)


def _slopes(disp, kept, rows, cols, o):
    """(n, 2) each window's disparities' slope a row and a column on, a plane through its kept matches about (`rows`,
    `cols`) (`o` the window's offsets): a wall aslant moves a window's right half sideways past its left's, 0.45 px
    over hers, and a window followed whole read her squint across on it 0.3 of its turn."""
    np = _np()
    r, c = rows[:, None, None] + o[None, :, None], cols[:, None, None] + o[None, None, :]
    k = kept[r, c].astype(float)
    d = np.where(kept[r, c], disp[r, c], 0.0)
    u = np.stack(np.broadcast_arrays(1.0, o[:, None] + 0.0, o[None, :] + 0.0), axis=-1)
    a = np.einsum('nij,ijp,ijq->npq', k, u, u)
    # whole numbers' determinant: a half or more where it is not none
    flat = np.abs(np.linalg.det(a)) >= 0.5
    b = np.einsum('nij,ijp,nij->np', k, u, d)
    plane = np.linalg.solve(np.where(flat[:, None, None], a, np.eye(3)), b[..., None])[..., 0]
    return np.where(flat[:, None], plane[:, 1:], 0.0)


def _followed(g, lw, y, x, slope):
    """((n,) rows, (n,) columns, (n,) whether held) of windows `lw` (n, WINDOW, WINDOW, means apart) followed on picture
    `g` from (`y`, `x`) by Lucas-Kanade both ways - each step its window bilinear where it stands, laid along its
    matches' `slope` (n, 2), moved by its own fall against `lw` - till each moves a 2^BITS'th of a pixel or less, held
    within a pixel of where it began: a correlation's parabola placed 66 of 182 rows within a fifth of a pixel of a
    whole one, and her squint read on them 0.2-0.7 of its turn."""
    np = _np()
    r0, (y0, x0) = WINDOW // 2, (y.copy(), x.copy())
    o = np.arange(-r0, r0 + 1)
    lean = -slope[:, 0, None, None] * o[None, :, None] + (1.0 - slope[:, 1, None, None]) * o[None, None, :]
    for _ in range(WINDOW):
        yy, xx = y[:, None, None] + o[None, :, None] + 0.0 * lean, x[:, None, None] + lean
        iy, ix = np.floor(yy).astype(np.int64), np.floor(xx).astype(np.int64)
        fy, fx = yy - iy, xx - ix
        a, b, c, d = g[iy, ix], g[iy, ix + 1], g[iy + 1, ix], g[iy + 1, ix + 1]
        top, low = a + (b - a) * fx, c + (d - c) * fx
        w = top + (low - top) * fy
        gx, gy = (b - a) * (1.0 - fy) + (d - c) * fy, low - top
        e = lw - (w - w.mean(axis=(1, 2), keepdims=True))
        sxx, syy, sxy = (gx * gx).sum(axis=(1, 2)), (gy * gy).sum(axis=(1, 2)), (gx * gy).sum(axis=(1, 2))
        bx, by = (gx * e).sum(axis=(1, 2)), (gy * e).sum(axis=(1, 2))
        det = sxx * syy - sxy * sxy
        flat = det > 0.0
        det = np.where(flat, det, 1.0)
        sx, sy = np.where(flat, (syy * bx - sxy * by) / det, 0.0), np.where(flat, (sxx * by - sxy * bx) / det, 0.0)
        y, x = np.clip(y + sy, y0 - 1.0, y0 + 1.0), np.clip(x + sx, x0 - 1.0, x0 + 1.0)
        if float(np.max(np.hypot(sx, sy), initial=0.0)) <= 1.0 / (1 << epipolar.BITS):
            break
    return y, x, flat & (np.abs(y - y0) < 1.0) & (np.abs(x - x0) < 1.0)


class Squint:

    """Her right eye's turn past her calibration's model of it, `w` (3,) rad in its own frame, and what she knows of it
    (`P`, its covariance): her church's word at first (`_known`), told on look by look (`told`)."""

    def __init__(self):
        np = _np()
        self.w, (self.P, self.Q), self.gz = np.zeros(3), _known(), None

    def turn(self):
        """(3, 3) her right eye's turn past her calibration's model of it as she holds it now, in its own frame."""
        return epipolar.turn(-self.w)

    def told(self, laid, a, b, disp, kept, left_R, right_R, line, gz, start=None):
        """(the look's start, whether to lay it anew): her squint told on by a look's laid pictures `a`, `b` matched at
        `disp` where `kept` (`epipolar.matched`), its eyes held turned `left_R`, `right_R` (her right one's this
        squint's) and `line` apart at her servos' gaze `gz`: what its matches' planes say (`matches`) weighed against
        what she knew before the look (`start`, (w, P): her pushrods' play `Q` past `P` where her servos moved since -
        held still, it plays not -, a look laid anew on its own word weighing its pictures once); laid anew where the
        step moved her matches past their own misfits' spread - read on pictures laid off by a misfit, a match's rows
        came 13 % short of it. Nothing where a look tells fewer than three."""
        np = _np()
        if start is None:
            moved, self.gz = self.gz is not None and tuple(gz) != self.gz, tuple(gz)
            start = self.w.copy(), self.P + self.Q * float(moved)
        lr, lc, rr, rc = matches(laid, a, b, disp, kept)
        # her eyes told her square to the bit - no squint, no play - her pictures tell her nothing past it
        if len(lr) <= 3 or not bool((np.linalg.eigvalsh(start[1]) > 0.0).all()):
            return start, False
        p, q = _rays(laid.at[0], lr.astype(float), lc.astype(float)), _rays(laid.at[1], rr, rc)
        R = np.asarray(right_R, float).T @ np.asarray(left_R, float)
        bl = np.asarray(right_R, float).T @ np.asarray(line, float)
        v = np.cross(q, bl / np.linalg.norm(bl))
        rp = p @ R.T
        r, J = np.einsum('ij,ij->i', v, rp), np.cross(rp, v)
        prior, off, d = np.linalg.inv(start[1]), _vee(epipolar.turn(self.w) @ epipolar.turn(-start[0])), np.zeros(3)
        for _ in range(STEPS):
            e = r + J @ d
            spread = 1.4826 * float(np.median(np.abs(e)))
            if spread <= 0.0:
                return start, False
            k = HUBER * spread
            wt = np.where(np.abs(e) > k, k / np.maximum(np.abs(e), 1e-300), 1.0) / (spread * spread)
            seen = (J * wt[:, None]).T @ J
            d = d + np.linalg.solve(prior + seen, -((J * wt[:, None]).T @ e + prior @ (off + d)))
        self.w, self.P = _vee(epipolar.turn(d) @ epipolar.turn(self.w)), np.linalg.inv(prior + seen)
        return start, float(np.abs(d).max()) > spread
