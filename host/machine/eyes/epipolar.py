"""The robot's pictures laid on epipolar planes however its eyes turn: a row a plane through both.

    laid = epipolar.Laid(left_R, right_R)                     # the robot's eyes' turns in its head's frame, as it holds them
    left, right = laid.warped(a, 0), laid.warped(b, 1)        # each eye's picture on the planes, (H, W, ..) as it was
    kept &= laid.kept(disp)                                    # matches lying on both eyes' pictures
    xyz = epipolar.points(rows, cols, disp, laid.beta, laid.fv)   # (n, 3) in `laid.E`, the left eye its origin
    left, right, disp, kept, laid = epipolar.matched(a, b, left_R, right_R)   # a look laid and matched

The robot's stereo matches rows (`stereo.disparity`): a parallel pair's rows are its planes through both eyes, a
turned eye's are not - turned 17 deg across the robot's median depth was 8 % off, 50 deg 32 %; its eyes 0.5
deg cross-eyed and held a parallel pair, 37 %. A plane is a row here (`Laid`): its angle about the line between the
robot's eyes the arctangent of its row's height off the middle over `fv`; along it a column the tangent of its
bearing off the robot's left eye's gaze (`beta`) over F - where its eyes look square to the line between
them, unsquinted, each pixel where it was. Each eye's picture is laid by its turn as the robot holds it - the robot's
gaze through its pushrods and each eye's squint as its calibration told it and the robot's
pictures since (`sight.Eyes.stereo`, `squint`) -, so a squint it knows is no misfit of its rows. A
match's point lies on its row's plane where its bearings from both eyes meet: its distance off their line the
baseline over their tangents' gap, along it that times the left's (`points`).
"""
import math
import threading

from machine import libm
from machine.eyes.stereo import BASELINE_M, FOVY_DEG, H, W, WINDOW, _array_disparity, _boxed, _np, card, cpu_disparity, indices

#: The robot's eyes' focal length, px (`sight.focal`).
F = (H / 2.0) / libm.tan(math.radians(FOVY_DEG) / 2.0)
#: The line from the robot's left eye to its right in its head's frame at rest: its right (`gimbal.cameras`'
#: places).
LINE = (-1.0, 0.0, 0.0)
#: A pixel's place on an eye's picture in fixed point, a 2^BITS'th of a pixel: integers on the card and the CPU.
BITS = 5
#: The robot's eyes' pictures are drawn FINE times finer each way (`sight.Eyes.see`), a pixel of the planes the mean
#: of the FINE x FINE of them where it falls - a sensor's pixel takes its area: laid bilinear from a picture a pixel a
#: point, each eye's picture blurred by its own place within a pixel, the robot's matches' ninth decile 1.9 % off
#: against 1.3 %.
FINE = 2
#: The finer picture's areas a pixel of the robot's own wide each, (rows, columns): one at each of its pixels
#: but the last FINE - 1.
_AREAS = (FINE * H - FINE + 1, FINE * W - FINE + 1)


def host(a):
    """`a` on the CPU: a card's array fetched, numpy's as it is."""
    return a.get() if hasattr(a, 'get') else a


def _areas(a, xp=None):
    """(FINE H - FINE + 1, FINE W - FINE + 1, ..) int64: a finer picture's sums over FINE x FINE from each pixel on,
    on array module `xp` (numpy's without)."""
    xp = xp or _np()
    i = xp.asarray(a).astype(xp.int64)
    h, w = _AREAS
    return sum((i[r:r + h, c:c + w] for r in range(FINE) for c in range(FINE)), xp.zeros((h, w) + i.shape[2:], xp.int64))


def turn(w):
    """(3, 3) the turn about `w` (3,) by its length, rad (Rodrigues')."""
    np = _np()
    w = np.asarray(w, float)
    a = float(np.linalg.norm(w))
    if a == 0.0:
        return np.eye(3)
    k = np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])
    return np.eye(3) + libm.sin(a) / a * k + (1.0 - libm.cos(a)) / (a * a) * k @ k


#: Cached CPU `Laid` geometry keyed by `(left_R, right_R, line)` bytes so steady-gaze looks skip recomputing maps.
_LAID_CACHE: dict = {}
_LAID_LOCK = threading.Lock()


def _freeze_arrays(item):
    """Mark every cached numpy array read-only so shared `Laid` geometry cannot be mutated in place."""
    if hasattr(item, 'setflags'):
        item.setflags(write=False)
    elif isinstance(item, tuple):
        for part in item:
            _freeze_arrays(part)


class Laid:

    """The robot's eyes' pictures laid on its epipolar planes: the planes' frame `E` (3, 3) in the
    robot's head's - x along the line from its left eye to its right (`line`, the robot's
    eyes' build's: `B` m long), z back, square to it under the robot's eyes' mean gaze, each eye's turn off it
    its half (Bouguet's) -, that gaze's bearing off square `beta` rad, the rows' focal length `fv` px (F times
    its cosine: a row as high as an eye's pixel where the robot looks); each eye's picture's places of the
    planes' pixels in fixed point (`at`, (rows, columns) (H, W) each) and whether they lie on it (`on`), its
    four pixels' flat places and weights a pixel of the planes (`_four`) - laid on array module `xp`, the
    card's where one answers (`matched`), `at` and `on` read on the CPU."""

    def __init__(self, left_R, right_R, line=None, xp=None):
        np = _np()
        self.xp, self._host, self._inner, self._key = xp or np, {}, None, None
        xp = self.xp
        lr = np.ascontiguousarray(left_R, dtype=float)
        rr = np.ascontiguousarray(right_R, dtype=float)
        ln = np.array(LINE) * BASELINE_M if line is None else np.ascontiguousarray(line, dtype=float)
        key = ((lr.tobytes(), rr.tobytes(), ln.tobytes())
               if xp is np and lr.shape == (3, 3) and rr.shape == (3, 3) and ln.shape == (3,) else None)
        self._key = key
        with _LAID_LOCK:
            cached = _LAID_CACHE.get(key) if key is not None else None
        if cached is not None:
            self.B, self.E, self.beta, self.fv, self._at, self._on, self._four, self._inner = cached
            return
        self.B = float(np.linalg.norm(ln))
        e1 = ln / self.B
        o = -(lr[:, 2] + rr[:, 2])
        o = o / np.linalg.norm(o)
        e3 = -(o - (o @ e1) * e1)
        e3 = e3 / np.linalg.norm(e3)
        self.E = np.column_stack([e1, np.cross(e3, e1), e3])
        self.beta = libm.arcsin(max(-1.0, min(1.0, float(o @ e1))))
        self.fv = F * libm.cos(self.beta)
        rows, cols = indices(xp, float)
        t = (H / 2.0 - rows[:, :1]) / self.fv if xp is np else (H / 2.0 - rows) / self.fv
        c = 1.0 / xp.sqrt(1.0 + t * t)
        b = self.beta + xp.arctan((cols - W / 2.0) * c / F)
        cb = xp.cos(b)
        rays = xp.stack([xp.sin(b), cb * t * c, -cb * c], axis=-1)
        self._at, self._on, self._four = zip(*(self._map(rays, R) for R in (lr, rr)))
        if key is not None:
            _freeze_arrays((self.E, self._at, self._on, self._four))
            with _LAID_LOCK:
                if len(_LAID_CACHE) >= 8 and key not in _LAID_CACHE:
                    _LAID_CACHE.pop(next(iter(_LAID_CACHE)))
                _LAID_CACHE[key] = (self.B, self.E, self.beta, self.fv, self._at, self._on, self._four, None)

    @property
    def at(self):
        """Each eye's picture's places of the planes' pixels in fixed point, ((rows, columns) (H, W)) each, on the CPU."""
        return self._held('at', lambda: tuple(tuple(host(a) for a in vu) for vu in self._at))

    @property
    def on(self):
        """Whether each of the planes' pixels lies on each eye's picture, (H, W) each, on the CPU."""
        return self._held('on', lambda: tuple(host(a) for a in self._on))

    def _held(self, name, read):
        if name not in self._host:
            self._host[name] = read()
        return self._host[name]

    def _map(self, rays, R):
        """(rows, columns) (H, W) in fixed point where each of the planes' `rays` falls on the picture of the eye
        turned `R` (its axes the columns), (H, W) whether it falls on it, and its four areas' (flat places (4, H W) on
        the finer picture's FINE x FINE sums `_areas`, weights (4, H W) in 2^BITS squared) about it there."""
        xp = self.xp
        d = rays @ xp.asarray((R.T @ self.E).T)
        if xp is _np():
            from tools.cores import vision
            got = vision.epi_from_d(d, H, W, F, BITS, FINE)
            if got is not None:
                return got
        ahead = xp.where(d[..., 2] < 0.0, -d[..., 2], 1.0)
        q, (h, w) = 1 << BITS, _AREAS
        # placed in fixed point first, on its picture by those: a pixel on its last row a float's last bit past it was off
        V, U = (xp.rint(xp.clip(a, -1.0, n) * q).astype(xp.int64)
                for a, n in ((H / 2.0 - F * d[..., 1] / ahead, H), (W / 2.0 + F * d[..., 0] / ahead, W)))
        on = (d[..., 2] < 0.0) & (U >= 0) & (U <= (W - 1) * q) & (V >= 0) & (V <= (H - 1) * q)
        V, U = xp.where(on, V, 0), xp.where(on, U, 0)
        # a pixel's area the finer picture's FINE of its pixels on from FINE times its place
        u, v = (U * FINE).ravel(), (V * FINE).ravel()
        u0, v0, fu, fv = u >> BITS, v >> BITS, u & (q - 1), v & (q - 1)
        u1, v1 = xp.minimum(u0 + 1, w - 1), xp.minimum(v0 + 1, h - 1)
        four = (xp.stack([v0 * w + u0, v0 * w + u1, v1 * w + u0, v1 * w + u1]).astype(xp.int32),
                xp.stack([(q - fu) * (q - fv), fu * (q - fv), (q - fu) * fv, fu * fv]).astype(xp.int32))
        return (V, U), on, four

    def warped(self, img, k):
        """Eye `k`'s (0 left, 1 right) finer picture `img` (FINE H, FINE W) or (.., 3) on the planes: each pixel its four
        areas' bilinear (`_areas`) in fixed point - a pixel falling where one is that one's area -, 0 off its picture;
        its type kept, on the CPU."""
        return host(self._warped(img, k))

    def _warped(self, img, k, in_c=True):
        """`warped` on its array module."""
        xp = self.xp
        (at, wt), a = self._four[k], xp.asarray(img)
        if in_c and xp is _np() and a.dtype == xp.uint8 and a.shape == (FINE * H, FINE * W, 3) and FINE == 2:
            from tools.cores import vision
            got = vision.warp_u8(a, at, wt, self._on[k], H, W, BITS)
            if got is not None:
                return got
        flat, whole = _areas(a, xp).reshape(_AREAS[0] * _AREAS[1], -1), (1 << (2 * BITS)) * FINE * FINE
        out = (sum((flat[at[i]] * wt[i][:, None] for i in range(4)), xp.zeros((H * W, flat.shape[1]), xp.int64))
               + whole // 2) // whole
        return xp.where(self._on[k].reshape(-1, 1), out, 0).reshape((H, W) + a.shape[2:]).astype(a.dtype)

    def kept(self, disp):
        """(H, W) whether a match at `disp` lies a window's half (`stereo.WINDOW`) inside both eyes' pictures: its
        left pixel's and the right one it matched; on the CPU."""
        return host(self._kept(self.xp.asarray(disp)))

    def _kept(self, disp):
        """`kept` on its array module."""
        xp = self.xp
        if self._inner is None:
            inner = tuple(_boxed((~on).astype(xp.int64), WINDOW // 2, xp) == 0 for on in self._on)
            if self._key is not None:
                _freeze_arrays(inner)
                with _LAID_LOCK:
                    if self._key in _LAID_CACHE:
                        _LAID_CACHE[self._key] = _LAID_CACHE[self._key][:-1] + (inner,)
            self._inner = inner
        if xp is _np():
            from tools.cores import vision
            got = vision.epi_kept(self._inner[0], self._inner[1], disp, H, W)
            if got is not None:
                return got
        rows, cols = indices(xp)
        return self._inner[0] & self._inner[1][rows, xp.clip(xp.rint(cols - disp).astype(xp.int64), 0, W - 1)]


def points(rows, cols, disp, beta, fv, B=BASELINE_M):
    """(n, 3) each pixel's point at its disparity in the planes' frame (`Laid.E`), the left eye its origin, the robot's
    eyes `B` m apart: on its row's plane where its bearings from both eyes meet. Square to the robot's eyes' line
    (`beta` 0, `fv` F) a parallel pair's, B times its pixel's offsets over its disparity - a disparity under a
    thousandth of a pixel its thousandth."""
    np = _np()
    t = (H / 2.0 - rows) / fv
    c = 1.0 / np.sqrt(1.0 + t * t)
    tb = libm.tan(beta)
    gl, gr = ((tb + a) / (1.0 - tb * a) for a in ((cols - W / 2.0) * c / F, (cols - np.maximum(disp, 1e-3) - W / 2.0) * c / F))
    rho = B / (gl - gr)
    return np.stack([rho * gl, rho * t * c, -rho * c], axis=1)


def matched(left, right, left_R, right_R, line=None):
    """(left, right, disparity, kept, laid) of a look: both eyes' pictures `left`, `right` (H, W, 3) laid on
    the robot's planes by its eyes' turns `left_R`, `right_R` in its head's frame as the robot
    holds them and the `line` from its left eye's middle to its right's (`Laid`), matched
    (`stereo.disparity`'s) where both lie (`Laid.kept`) - laid and matched on the card where one answers
    (`stereo.card`), else on the CPU, CI's: laid on the CPU beside the card's match a look's rays and warps
    were 386 ms of its 782 -; each returned on the CPU."""
    xp = card() or _np()
    laid = Laid(left_R, right_R, line, xp)
    a, b = laid._warped(left, 0), laid._warped(right, 1)
    disp, kept = cpu_disparity(a, b) if xp is _np() else _array_disparity(a, b, xp, card=True)
    return host(a), host(b), host(disp), host(kept & laid._kept(disp)), laid
