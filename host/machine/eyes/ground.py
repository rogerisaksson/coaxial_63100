"""Her floor as her looks lay it: a top-down mosaic of its grey and height, each look laid onto it.

Each look's floor is laid onto her mosaic before it is kept - its height where her floor's was, its grey where her
mosaic's lies - the text's visual innovation (`lie_group_slam_core.lisp`, `machine.senses.inekf.seen`): her soles
carry her place between looks, what her floor shows - its speckle, its shadows, its colours' shifts - holds it. Kept
where her estimate had it, her floor's points stood 2-6 cm high and her map told a bed of them 4.6 m off the bed she
was told.

    floor = ground.Mosaic(span)
    fix, info = floor.fixed(xyz, grey, eye)   # (turn about up through eye, x, y, z) laying a look onto it, information
    floor.add(xyz, grey)                      # a look's floor into it

Her floor is what lies within FLOOR_M of her map's ground (`voxels.FLOOR_M`, what her map leaves out). A look's floor
points are laid onto her mosaic's grey by Gauss-Newton on their grey - her mosaic's under them, scaled and shifted
as the look's light has it -, its turn about up through her eye and its shift along her floor, her mosaic blurred
from half her map's voxel to its own cell, ROUNDS a blur; up by their heights against her mosaic's. Each is Huber's
past HUBER of the look's own scatter; only what her floor resolves is fixed - a direction whose error is under a
cell, the turn weighed by its points' reach -, nothing on a first look or on a floor not seen before. Her blurs are
kept between looks, laid anew where a look changed her mosaic alone (`add`): blurred whole each look, her 347 x 347
cells were half of a look's 10-12 ms.
"""
import math

from machine.eyes import voxels
from machine.eyes.stereo import _np

#: A mosaic cell HITS points at least is hers to lay a look onto (her map's own count, `voxels.HITS`).
HITS = voxels.HITS
#: Her floor's speckle as her kinds' (`sight.assets`): plain, it gave her stereo 1.2 grey levels a pixel of gradient
#: where her things gave 7, its height read 0.018 m, sd 0.027 - nothing to hold her looks to.
LOOK = ((0.62, 0.5, 0.38), (0.36, 0.28, 0.2))
#: Her world's floor geoms (`machine.world.floor.ground`'s), drawn in LOOK for her eyes (`speckled`).
FLOORED = ('floor', 'slab_a', 'slab_b')
#: Gauss-Newton rounds a blur; Huber's knee in the look's scatter (95 % of the mean's efficiency under a normal
#: scatter, the text's robust kernel); the scale of a normal scatter its median absolute deviation measures.
ROUNDS, HUBER, MAD = 3, 1.345, 1.4826


def _turned(w, angle):
    """Floor offsets `w` (n, 2: x, z) turned `angle` rad about up: x cos + z sin, z cos - x sin."""
    np = _np()
    c, s = math.cos(angle), math.sin(angle)
    return np.column_stack([c * w[:, 0] + s * w[:, 1], c * w[:, 1] - s * w[:, 0]])


def laid(xyz, eye, fix):
    """Points `xyz` (n, 3) laid by a look's `fix` (`Mosaic.fixed`): turned about up through `eye`, then shifted."""
    np = _np()
    e = np.asarray(eye, float)
    out = xyz + fix[1:]
    out[:, [0, 2]] = e[[0, 2]] + _turned(xyz[:, [0, 2]] - e[[0, 2]], fix[0]) + fix[[1, 3]]
    return out


def _box(a, r):
    """`a` (nx, nz) summed over each cell's (2 r + 1) square, its edges' as far as they go: along x, then z, each
    cell's sum in the same order whatever `a` is cut from (`Mosaic._kept`)."""
    np = _np()
    if r == 0:
        return a
    p = np.pad(a, r)
    along = p[:a.shape[0]].copy()
    for d in range(1, 2 * r + 1):
        along += p[d:d + a.shape[0]]
    out = along[:, :a.shape[1]].copy()
    for d in range(1, 2 * r + 1):
        out += along[:, d:d + a.shape[1]]
    return out


def _median(a):
    """`a`'s median as numpy's (its two middle values' mean where even), its partition alone: numpy's own checks
    for nans were a third of a look's medians."""
    np = _np()
    h = len(a) // 2
    if len(a) % 2:
        return float(np.partition(a, h)[h])
    p = np.partition(a, (h - 1, h))
    return 0.5 * float(p[h - 1] + p[h])


def _scale(r):
    """A residual's scatter: its median absolute deviation's (MAD), a hair at the least."""
    np = _np()
    return max(MAD * _median(np.abs(r - _median(r))), 1e-9)


def _huber(r, s):
    """Huber's weights of residuals `r` over scatter `s` (HUBER)."""
    np = _np()
    return np.minimum(1.0, HUBER * s / np.maximum(np.abs(r), 1e-12))


def speckled(model):
    """Her world's floor (FLOORED) drawn in her floor's speckle (`sight.assets`'s 'floor') in `model`, her eyes'
    renderer's (`sight.Eyes`): its look alone - its contacts, her laying and her page's view her world's own."""
    model.geom_matid[[model.geom(name).id for name in FLOORED]] = model.material('floor').id


class Mosaic:

    """Her floor's grey and height over her map's floor (`voxels.X_M`, `Z_M`), its cell what a look's points `span`
    rad apart (`sight.EVERY` pixels) span midway through her map's reach (`voxels.NEAR_M`, `FAR_M`): each cell's
    points' count and sums of grey, grey squared and height; each blur's grey and whether it holds HITS, and the cells
    a look changed since (`_kept`)."""

    def __init__(self, span):
        # a direction her floor resolves is told to a cell of it, no finer (`fixed`): looks of her true head laid onto a
        # mosaic of looks 0.4-0.6 m away scattered 1.2-2.1 cm (median) and 3.5-5.3 cm at worst - her speckle seen from
        # elsewhere, its depth, her shadows -; told uniform within a cell (sd 0.58 cm) her gate let 23-50 % of them in
        np = _np()
        cell = span * 0.5 * (voxels.NEAR_M + voxels.FAR_M)
        self.cell, self.finest = cell, 1.0 / cell ** 2
        self.nx = int(round((voxels.X_M[1] - voxels.X_M[0]) / cell))
        self.nz = int(round((voxels.Z_M[1] - voxels.Z_M[0]) / cell))
        self.sums = np.zeros((4, self.nx, self.nz))
        self.looks = 0
        # her blurs, coarse to fine: half her map's voxel, halved to her mosaic's own cell
        self.blurs = [int(r) for r in 2 ** np.arange(int(math.log2(max(voxels.CELL_M / cell / 2.0, 1.0))), -1, -1)] + [0]
        self._blurred = [(np.zeros((self.nx, self.nz)), np.zeros((self.nx, self.nz), bool)) for _ in self.blurs]
        self._dirty = None

    @staticmethod
    def floor(xyz, up=0.0, band=voxels.FLOOR_M):
        """Which of points `xyz` (n, 3) lie on her floor: within `band` of her map's ground, they shifted `up`."""
        np = _np()
        return np.abs(xyz[:, 1] + up) < band

    def _cells(self, xz):
        """(flat cell index, whether on her mosaic) of floor places `xz` (n, 2)."""
        np = _np()
        i = np.floor((xz[:, 0] - voxels.X_M[0]) / self.cell).astype(int)
        k = np.floor((xz[:, 1] - voxels.Z_M[0]) / self.cell).astype(int)
        return i * self.nz + k, (i >= 0) & (i < self.nx) & (k >= 0) & (k < self.nz)

    def _up(self, xyz):
        """(the shift up laying points `xyz` (n, 3) onto her mosaic's floor heights, its information): Huber's mean of
        their heights less her mosaic's where it holds them; each cell under them one sample of its height - the points
        within a cell share its error -; (0, 0) where it holds none."""
        np = _np()
        at, inside = self._cells(xyz[:, [0, 2]])
        at, y = at[inside], xyz[inside, 1]
        count = self.sums[0].ravel()[at]
        held = count >= HITS
        if not held.any():
            return 0.0, 0.0
        at = at[held]
        r = y[held] - self.sums[3].ravel()[at] / count[held]
        s_r = _scale(r)
        wgt = _huber(r - _median(r), s_r)
        return -float(wgt @ r / wgt.sum()), min(len(np.unique(at)) / s_r ** 2, self.finest)

    def add(self, xyz, grey):
        """A look's floor points `xyz` (n, 3) and their `grey` (n,) into her mosaic, summed over the cells they span
        alone; those cells its blurs' to lay anew (`_kept`)."""
        np = _np()
        self.looks += 1
        on = self.floor(xyz)
        i = np.floor((xyz[on, 0] - voxels.X_M[0]) / self.cell).astype(int)
        k = np.floor((xyz[on, 2] - voxels.Z_M[0]) / self.cell).astype(int)
        inside = (i >= 0) & (i < self.nx) & (k >= 0) & (k < self.nz)
        i, k, g, y = i[inside], k[inside], grey[on][inside], xyz[on, 1][inside]
        if not len(i):
            return
        i0, i1, k0, k1 = int(i.min()), int(i.max()) + 1, int(k.min()), int(k.max()) + 1
        at, shape = (i - i0) * (k1 - k0) + (k - k0), (i1 - i0, k1 - k0)
        spanned = self.sums[:, i0:i1, k0:k1]
        for c, weights in enumerate((None, g, g * g, y)):
            spanned[c] += np.bincount(at, weights, shape[0] * shape[1]).reshape(shape)
        was = self._dirty or (i0, i1, k0, k1)
        self._dirty = (min(was[0], i0), max(was[1], i1), min(was[2], k0), max(was[3], k1))

    def _kept(self):
        """[(her mosaic's grey blurred, whether its blur holds HITS points) each blur], laid anew over the cells a look
        changed since (`add`) and as far round as a blur reaches."""
        np = _np()
        if self._dirty is not None:
            i0, i1, k0, k1 = self._dirty
            for (mean, told), r in zip(self._blurred, self.blurs):
                # the cells whose blur changed, and the cells those blurs sum
                a0, a1, b0, b1 = max(i0 - r, 0), min(i1 + r, self.nx), max(k0 - r, 0), min(k1 + r, self.nz)
                c0, c1, d0, d1 = max(a0 - r, 0), min(a1 + r, self.nx), max(b0 - r, 0), min(b1 + r, self.nz)
                inner = (slice(a0 - c0, a1 - c0), slice(b0 - d0, b1 - d0))
                count = _box(self.sums[0, c0:c1, d0:d1], r)[inner]
                mean[a0:a1, b0:b1] = _box(self.sums[1, c0:c1, d0:d1], r)[inner] / np.maximum(count, 1.0)
                told[a0:a1, b0:b1] = count >= HITS
            self._dirty = None
        return self._blurred

    def _sampled(self, mean, told, xz):
        """(her mosaic's `mean` (nx, nz) at floor places `xz` (n, 2) - bilinear -, its gradient along x and z per m,
        and whether all four cells round each are `told`)."""
        np = _np()
        u = (xz[:, 0] - voxels.X_M[0]) / self.cell - 0.5
        v = (xz[:, 1] - voxels.Z_M[0]) / self.cell - 0.5
        fu, fv = np.floor(u), np.floor(v)
        i, k = fu.astype(int), fv.astype(int)
        fu, fv = u - fu, v - fv
        ok = (i >= 0) & (i < self.nx - 1) & (k >= 0) & (k < self.nz - 1)
        at, flat, held = np.where(ok, i * self.nz + k, 0), mean.ravel(), told.ravel()
        m00, m10, m01, m11 = flat[at], flat[at + self.nz], flat[at + 1], flat[at + self.nz + 1]
        ok &= held[at] & held[at + self.nz] & held[at + 1] & held[at + self.nz + 1]
        m = (m00 * (1 - fu) + m10 * fu) * (1 - fv) + (m01 * (1 - fu) + m11 * fu) * fv
        du = ((m10 - m00) * (1 - fv) + (m11 - m01) * fv) / self.cell
        dv = ((m01 - m00) * (1 - fu) + (m11 - m10) * fu) / self.cell
        return m, du, dv, ok

    def fixed(self, xyz, grey, eye):
        """(fix (4,), its information (4, 4), its grey's fit): the turn about up through `eye`, rad, and the shift along x,
        y, z, m, that lays a look's floor - points `xyz` (n, 3) in her estimate's frame, their `grey` (n,) - onto her
        mosaic, and how much of its grey her mosaic's tells (R^2 at the last, nan laid onto none)."""
        np = _np()
        fix, info = np.zeros(4), np.zeros((4, 4))
        if not self.looks:
            return fix, info, math.nan
        # up first, over twice her floor's band: chosen in a look's own heights her floor shifted out of its band
        wide = self.floor(xyz, band=2.0 * voxels.FLOOR_M)
        fix[2] = self._up(xyz[wide])[0]
        on = self.floor(xyz, fix[2])
        if not on.any():
            return np.zeros(4), info, math.nan
        p, g, e = xyz[on], grey[on], np.asarray(eye, float)
        w = p[:, [0, 2]] - e[[0, 2]]
        for mean, told in self._kept():
            for _ in range(ROUNDS):
                q = _turned(w, fix[0])
                m, du, dv, ok = self._sampled(mean, told, e[[0, 2]] + q + fix[[1, 3]])
                if ok.sum() < 4:
                    return np.zeros(4), np.zeros((4, 4)), math.nan
                # the look's grey against her mosaic's, scaled and shifted by its light: least squares in closed form
                mo, go = m[ok], g[ok]
                dm, dg = mo - mo.sum() / len(mo), go - go.sum() / len(go)
                gain = float(dm @ dg) / max(float(dm @ dm), 1e-300)
                r = dg - gain * dm
                fit = 1.0 - float(r @ r) / max(float(dg @ dg), 1e-12)
                s_r = _scale(r)
                wgt = _huber(r, s_r) / s_r ** 2
                qo, duo, dvo = q[ok], du[ok], dv[ok]
                J = -gain * np.column_stack([duo * qo[:, 1] - dvo * qo[:, 0], duo, dvo])
                reach = math.sqrt(float(np.mean(qo[:, 0] ** 2 + qo[:, 1] ** 2)))
                scale = np.array([reach, 1.0, 1.0])
                Jw = J * wgt[:, None]
                lam, U = np.linalg.eigh(Jw.T @ J / np.outer(scale, scale))
                U, lam = U[:, lam > self.finest], lam[lam > self.finest]
                step = -U @ ((U.T @ (Jw.T @ r / scale)) / lam) / scale
                fix[[0, 1, 3]] += step
                # told no finer than her mosaic's cell (`finest`): her points' errors within a look are one look's, not
                # each their own - their sum's information said 1-4 mm
                info[np.ix_([0, 1, 3], [0, 1, 3])] = np.outer(scale, scale) * self.finest * (U @ U.T)
        # up again where they lie laid along her floor
        fix[2], info[2, 2] = self._up(laid(p, e, np.array([fix[0], fix[1], 0.0, fix[3]])))
        return fix, info, fit
