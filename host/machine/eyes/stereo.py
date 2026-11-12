"""Her stereo disparity by semi-global matching, on the card or the CPU, in integers alike.

`disparity(left, right)` matches the left eye's picture against the right's as the text asks
(`configure-phase-gated-stereo`, `:semi-global-matching-disparity`): a pixel's cost at a disparity the bits its
census differs in from the right eye's there (CENSUS), aggregated along PATHS - the cost, plus the least of the
path's last pixel's at that disparity, one off for P1 or any for P2, less that pixel's least - and summed; the least
sum a pixel's disparity, placed within a pixel by the parabola through its window's own costs about it (WINDOW),
kept where textured, the best UNIQUE times under the next, the right eye's own match agreeing and no depth's edge
within half a window. Greys in fixed point, sums in integers, the paths' arithmetic float32 on whole numbers under
2^24: the card's kernel (`_KERNEL`, a block a path's chain - each independent of the others, its pixels in turn -,
a thread a disparity) and numpy's (a path's front at a time) in one order, bit for bit alike - a grey in float32
rounded was not (the dot product's order: 2 of 76 800 pixels a level apart, a subpixel 0.99 px). Against the
renderer's depth (five looks at the furniture, 320 x 240): 1.13 % of the kept pixels matched a pixel or more off,
the block match alone 3.61 % (two thirds by depth edges, its windows over both sides); within a pixel 0.121 px, the
block match's 0.122. `COAXIAL_GPU=0` keeps it on the CPU.
"""
import math
import os

from machine.eyes.voxels import NEAR_M

#: Her eyes (`machine.eyes.sight`): W x H pixels, FOVY_DEG high, BASELINE_M apart - the text's 120 mm
#: (`configure-phase-gated-stereo`); at 320 x 240 and 100 mm her map told a seat's depth 0.20-0.30 m of 0.44, at
#: 640 x 480 and 120 mm, its points' footprints read (`voxels.Map._box`), 0.34-0.43.
W, H, FOVY_DEG, BASELINE_M = 640, 480, 60.0, 0.12
#: The match's bounds: its disparities, those of whatever stands NEAR_M off her eye or further and two over for the
#: parabola about the best; the window placing a match within a pixel and telling its texture, that texture's least
#: spread, the best's margin under the next.
DISPARITIES = int(math.ceil((H / 2.0) / math.tan(math.radians(FOVY_DEG) / 2.0) * BASELINE_M / NEAR_M)) + 2
WINDOW, TEXTURE, UNIQUE = 9, 1.5, 1.15
#: A pixel's census, px (columns, rows): a bit a neighbour darker than it past QUANTUM grey levels, another one
#: brighter past it, 62 of each. The aggregation's penalties, a step of one disparity along a path and of more, the
#: customary ones on such a census, its two planes'.
CENSUS, P1, P2 = (9, 7), 20, 240
#: A neighbour within this of a pixel's grey is told neither darker nor brighter: her pictures laid on her planes are
#: rounded a grey level each (`epipolar.Laid.warped`) and a bit read on a level flipped - looked round four ways, her
#: squinted eyes told 9 of the room's 20 seats and beds on one plane, 14 on two; square, 17 either way.
QUANTUM = 1
#: The paths aggregated, (dx, dy) a pixel each step, in this order on both the card and the CPU.
PATHS = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1))
#: A grey in fixed point: the luma's weights in thousandths, rounded half up.
LUMA = (299, 587, 114)

#: The switch: 0 keeps stereo matching on the CPU.
ENV = 'COAXIAL_GPU'

#: One path's aggregation: a block a chain of it - from where its pixel before leaves the picture, a pixel along it at
#: a time -, a thread a disparity; the costs and the sums laid (H, W, D), a pixel's disparities side by side; the
#: chain's last L and this one in shared memory. A block a path's front over all its pixels, a thread one across it,
#: ran a 640 x 480 picture's paths on one of the card's multiprocessors, 413 ms a look.
_KERNEL = r"""
extern "C" __global__ void sgm_chains(const float* C, float* S, int H, int W, int D, int dx, int dy,
                                      float p1, float p2) {
    extern __shared__ float fronts[];
    const float inf = __int_as_float(0x7f800000);
    int b = blockIdx.x, d = threadIdx.x, rows = dx != 0 ? H : 0, x, y;
    if (b < rows) {
        x = dx > 0 ? 0 : W - 1;
        y = b;
    } else {
        int t = b - rows;
        x = dx == 0 ? t : (dx > 0 ? t + 1 : t);
        y = dy > 0 ? 0 : H - 1;
    }
    float* prev = fronts;
    float* cur = fronts + D;
    bool first = true;
    while (x >= 0 && x < W && y >= 0 && y < H) {
        if (d < D) {
            size_t at = ((size_t)y * W + x) * D + d;
            float c = C[at], l = c;
            if (!first) {
                float least = inf;
                for (int e = 0; e < D; ++e) least = fminf(least, prev[e]);
                float p = prev[d];
                p = fminf(p, (d > 0 ? prev[d - 1] : inf) + p1);
                p = fminf(p, (d < D - 1 ? prev[d + 1] : inf) + p1);
                p = fminf(p, least + p2);
                l = c + p - least;
            }
            cur[d] = l;
            S[at] += l;
        }
        __syncthreads();
        float* swap = prev;
        prev = cur;
        cur = swap;
        first = false;
        x += dx;
        y += dy;
    }
}
"""


#: The rest of the match on the card in three launches where numpy's arithmetic took some 2 500 small ones - its
#: census a neighbour at a time, its costs and windows a disparity at a time, its right eye's sums a disparity at a
#: time: 157 ms a match, launch-bound on her card's driver. Integers alike, so the same bits: each pixel's census (the
#: neighbours in `_census`'s order, a picture's edge held as numpy's 'edge' pad holds it); each disparity's cost - the
#: bits its census differs in - laid (H, W, D) as the paths read it, and its window's grey differences summed over
#: that disparity's own overlap, its edges held as `_boxed` holds them; each pixel's best as the right eye sees it,
#: numpy's argmin of `back`: its first least, nan first.
_FUSED = r"""
extern "C" __global__ void censused(const long long* g, unsigned long long* dark, unsigned long long* bright,
                                    int H, int W, int rx, int ry, long long q) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (long long)H * W) return;
    int y = (int)(i / W), x = (int)(i % W);
    long long c = g[i];
    unsigned long long dk = 0, bt = 0;
    for (int dy = -ry; dy <= ry; ++dy) {
        int yy = min(max(y + dy, 0), H - 1);
        for (int dx = -rx; dx <= rx; ++dx) {
            if (dx == 0 && dy == 0) continue;
            long long n = g[(long long)yy * W + min(max(x + dx, 0), W - 1)];
            dk = (dk << 1) | (unsigned long long)(n < c - q);
            bt = (bt << 1) | (unsigned long long)(n > c + q);
        }
    }
    dark[i] = dk;
    bright[i] = bt;
}

extern "C" __global__ void costed(const long long* gl, const long long* gr, const unsigned long long* dl,
                                  const unsigned long long* dr, const unsigned long long* bl,
                                  const unsigned long long* br, float* cost, float* block, int H, int W, int D, int r) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (long long)D * H * W) return;
    int x = (int)(i % W), y = (int)((i / W) % H), d = (int)(i / ((long long)W * H));
    float c = __int_as_float(0x7f800000), b = c;
    if (x >= d) {
        long long j = (long long)y * W + x, k = j - d, s = 0;
        c = (float)(__popcll(dl[j] ^ dr[k]) + __popcll(bl[j] ^ br[k]));
        for (int dy = -r; dy <= r; ++dy) {
            long long row = (long long)min(max(y + dy, 0), H - 1) * W;
            for (int dx = -r; dx <= r; ++dx) {
                int at = min(max(x - d + dx, 0), W - d - 1);
                long long a = gl[row + at + d] - gr[row + at];
                s += a < 0 ? -a : a;
            }
        }
        b = (float)s;
    }
    cost[((long long)y * W + x) * D + d] = c;
    block[i] = b;
}

extern "C" __global__ void backed(const float* total, long long* best, int H, int W, int D) {
    long long i = (long long)blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= (long long)H * W) return;
    int y = (int)(i / W), x = (int)(i % W);
    float least = __int_as_float(0x7f800000);
    long long at = 0;
    for (int d = 0; d < D; ++d) {
        float v = x + d < W ? total[((long long)y * W + x + d) * D + d] : __int_as_float(0x7f800000);
        if (isnan(v)) { at = d; break; }
        if (v < least) { least = v; at = d; }
    }
    best[i] = at;
}
"""
#: Threads a block of the fused launches.
_BLOCK = 256


def _np():
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    import numpy
    return numpy


#: Her pictures' pixels' (rows, columns) by array module and type, once laid (`indices`).
_GRIDS: dict = {}


def indices(xp, dtype: type = int):
    """(rows, columns) (H, W) of her pictures' pixels on array module `xp`, laid once each - five of a look's passes
    laid them anew, 4.9 MB a time -: read, never written."""
    key = (xp.__name__, dtype)
    if key not in _GRIDS:
        _GRIDS[key] = xp.indices((H, W), dtype=dtype)
    return _GRIDS[key]


def _grey(rgb, xp):
    """(H, W) int64: a picture's grey in fixed point (LUMA), the same integers on the card and the CPU."""
    c = xp.asarray(rgb).astype(xp.int64)
    return (LUMA[0] * c[..., 0] + LUMA[1] * c[..., 1] + LUMA[2] * c[..., 2] + 500) // 1000


def _boxed(a, r, xp=None):
    """`a` summed over (2r+1)^2 windows, edges clipped."""
    xp = xp or _np()
    p = xp.pad(a, r, mode='edge')
    c = xp.cumsum(xp.cumsum(p, axis=0), axis=1)
    c = xp.pad(c, ((1, 0), (1, 0)))
    k = 2 * r + 1
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def _census(g, xp):
    """((H, W) uint64, (H, W) uint64): each pixel's census (CENSUS) - a bit a neighbour darker than it past QUANTUM,
    a bit one brighter -, edges clipped."""
    rx, ry = CENSUS[0] // 2, CENSUS[1] // 2
    p = xp.pad(g, ((ry, ry), (rx, rx)), mode='edge')
    dark, bright = xp.zeros((H, W), dtype=xp.uint64), xp.zeros((H, W), dtype=xp.uint64)
    for dy in range(-ry, ry + 1):
        for dx in range(-rx, rx + 1):
            if dx or dy:
                n = p[ry + dy:ry + dy + H, rx + dx:rx + dx + W]
                dark = (dark << xp.uint64(1)) | (n < g - QUANTUM).astype(xp.uint64)
                bright = (bright << xp.uint64(1)) | (n > g + QUANTUM).astype(xp.uint64)
    return dark, bright


def _aggregated(cost, xp):
    """(D, H, W) the cost's sums along PATHS, in numpy's arithmetic as the card's kernel's: each path's front
    a step at a time, the last step's L at the pixel the path came from - none at its start, the cost alone."""
    total, rows, cols = xp.zeros_like(cost), xp.arange(H), xp.arange(W)
    edge = xp.full((1, 1), xp.inf, dtype=cost.dtype)
    for dx, dy in PATHS:
        across, along, prev = (H, W, None) if dx else (W, H, None)
        for s in range(along):
            a = s if (dx or dy) > 0 else along - 1 - s
            c = cost[:, :, a] if dx else cost[:, a, :]
            if prev is None:
                l = c.copy()
            else:
                q = (rows if dx else cols) - (dy if dx else dx)
                came = (q >= 0) & (q < across)
                p0 = prev[:, xp.clip(q, 0, across - 1)]
                least = p0.min(axis=0)
                down = xp.concatenate([xp.broadcast_to(edge, (1, across)), p0[:-1]], axis=0)
                up = xp.concatenate([p0[1:], xp.broadcast_to(edge, (1, across))], axis=0)
                p = xp.minimum(xp.minimum(xp.minimum(p0, down + xp.float32(P1)), up + xp.float32(P1)), least + xp.float32(P2))
                l = xp.where(came[None, :], c + p - least, c)
            if dx:
                total[:, :, a] += l
            else:
                total[:, a, :] += l
            prev = l
    return total


def _card_aggregated(laid, cp):
    """(H, W, D) the sums along PATHS of the cost `laid` (H, W, D) on the card (`_KERNEL`), a launch a path, a block
    each of its chains."""
    kernel = _SOLVER.get('kernel')
    if kernel is None:
        kernel = _SOLVER['kernel'] = cp.RawKernel(_KERNEL, 'sgm_chains')
    D = laid.shape[2]
    total = cp.zeros_like(laid)
    for dx, dy in PATHS:
        chains = (H if dx else 0) + ((W - 1 if dx else W) if dy else 0)
        kernel((chains,), (D,), (laid, total, cp.int32(H), cp.int32(W), cp.int32(D), cp.int32(dx), cp.int32(dy),
                                 cp.float32(P1), cp.float32(P2)), shared_mem=2 * D * 4)
    return total


def _fused(cp, name):
    """The card's fused launch `name` (`_FUSED`), compiled at its first ask."""
    module = _SOLVER.get('fused')
    if module is None:
        module = _SOLVER['fused'] = cp.RawModule(code=_FUSED)
    return module.get_function(name)


def _card_costed(gl, gr, cp):
    """((H, W, D) the census costs as the paths read them, (D, H, W) the windows' sums) of greys `gl`, `gr` on the
    card in three launches (`_FUSED`): `_census`'s and the cost loop's in `_array_disparity`, the same bits."""
    n, cells = H * W, DISPARITIES * H * W
    gl, gr = cp.ascontiguousarray(gl, dtype=cp.int64), cp.ascontiguousarray(gr, dtype=cp.int64)
    census = [cp.empty((H, W), dtype=cp.uint64) for _ in range(4)]
    for g, (dark, bright) in ((gl, census[:2]), (gr, census[2:])):
        _fused(cp, 'censused')(((n + _BLOCK - 1) // _BLOCK,), (_BLOCK,),
                               (g, dark, bright, cp.int32(H), cp.int32(W), cp.int32(CENSUS[0] // 2),
                                cp.int32(CENSUS[1] // 2), cp.int64(QUANTUM)))
    cost = cp.empty((H, W, DISPARITIES), dtype=cp.float32)
    block = cp.empty((DISPARITIES, H, W), dtype=cp.float32)
    _fused(cp, 'costed')(((cells + _BLOCK - 1) // _BLOCK,), (_BLOCK,),
                         (gl, gr, census[0], census[2], census[1], census[3], cost, block, cp.int32(H), cp.int32(W),
                          cp.int32(DISPARITIES), cp.int32(WINDOW // 2)))
    return cost, block


def _card_backed(total, cp):
    """(H, W) each pixel's best as the right eye sees it - numpy's argmin over `back` in `_array_disparity` - of the
    paths' sums `total` (H, W, D), one launch."""
    best = cp.empty((H, W), dtype=cp.int64)
    _fused(cp, 'backed')((((H * W) + _BLOCK - 1) // _BLOCK,), (_BLOCK,),
                         (total, best, cp.int32(H), cp.int32(W), cp.int32(DISPARITIES)))
    return best


def _array_disparity(left, right, xp, card=False):
    """Semi-global matching on array module `xp` (numpy or cupy); its paths on the card's kernel where `card`."""
    gl, gr = _grey(left, xp), _grey(right, xp)
    r = WINDOW // 2
    if card:
        laid, block = _card_costed(gl, gr, xp)
        paths = _card_aggregated(laid, xp)
        total = paths.transpose(2, 0, 1)
    else:
        (dl, bl), (dr, br) = _census(gl, xp), _census(gr, xp)
        cost = xp.full((DISPARITIES, H, W), xp.inf, dtype=xp.float32)
        block = xp.full((DISPARITIES, H, W), xp.inf, dtype=xp.float32)
        for d in range(DISPARITIES):
            cost[d, :, d:] = (xp.bitwise_count(dl[:, d:] ^ dr[:, :W - d])
                              + xp.bitwise_count(bl[:, d:] ^ br[:, :W - d])).astype(xp.float32)
            block[d, :, d:] = _boxed(xp.abs(gl[:, d:] - gr[:, :W - d]), r, xp).astype(xp.float32)
        total = _aggregated(cost, xp)
    best = xp.argmin(total, axis=0)
    rows, cols = indices(xp)
    # the paths' sum chooses the disparity, the window's own costs about it place it within a pixel: the paths'
    # sum read so spread it 0.197 px, the window's 0.122 (against the renderer's depth)
    c0 = block[best, rows, cols]
    lo = block[xp.maximum(best - 1, 0), rows, cols]
    hi = block[xp.minimum(best + 1, DISPARITIES - 1), rows, cols]
    denom = lo - 2.0 * c0 + hi
    curved = xp.isfinite(denom) & (denom > 1e-6)
    sub = xp.zeros_like(c0)
    sub[curved] = 0.5 * (lo[curved] - hi[curved]) / denom[curved]
    disp = best + xp.clip(sub, -0.5, 0.5)
    apart = xp.abs(xp.arange(DISPARITIES)[:, None, None] - best[None, :, :]) > 2
    second = xp.where(apart, total, xp.inf).min(axis=0)
    mean = _boxed(gl, r, xp) / WINDOW ** 2
    spread = xp.sqrt(xp.maximum(_boxed(gl * gl, r, xp) / WINDOW ** 2 - mean * mean, 0.0))
    if card:
        seen = _card_backed(paths, xp)
    else:
        back = xp.full_like(total, xp.inf)
        for d in range(DISPARITIES):
            back[d, :, :W - d] = total[d, :, d:]
        seen = xp.argmin(back, axis=0)
    agreed = xp.abs(seen[rows, xp.clip(cols - best, 0, W - 1)] - best) <= 1
    # a depth's edge: its disparity a step over one off a neighbour's - a slope's steps one at most -; within half a
    # window of it the windows took both sides, the nearer smeared over the further: 23.7 % of such pixels
    # matched a pixel or more off, 0.98 % of the rest
    jump = xp.zeros((H, W), dtype=xp.int64)
    jump[:, 1:] |= xp.abs(best[:, 1:] - best[:, :-1]) > 1
    jump[1:, :] |= xp.abs(best[1:, :] - best[:-1, :]) > 1
    rim = _boxed(jump, r, xp) > 0
    kept = ((spread > TEXTURE) & (second > UNIQUE * xp.maximum(total[best, rows, cols], 1.0)) & (best > 0)
            & (best < DISPARITIES - 1) & xp.isfinite(c0) & agreed & ~rim)
    kept[:, :DISPARITIES] = False
    return disp, kept


def cpu_disparity(left, right):
    """(disparity px, kept) computed on the CPU."""
    return _array_disparity(left, right, _np())


_SOLVER = {}


def _gpu_solver():
    """The card's stereo solver (CuPy and its kernel), or None when COAXIAL_GPU=0 or no card answers."""
    if os.environ.get(ENV, '1') == '0':
        return None
    if 'solver' in _SOLVER:
        return _SOLVER['solver']
    solver = None
    try:
        import importlib
        cp = importlib.import_module('cupy')
        if cp.cuda.runtime.getDeviceCount() > 0:
            solver = lambda left, right: tuple(cp.asnumpy(a) for a in _array_disparity(left, right, cp, card=True))
            _SOLVER['cp'] = cp
    except Exception:
        solver = None
    _SOLVER['solver'] = solver
    return solver


def card():
    """CuPy where her stereo's card answers (`_gpu_solver`), None on the CPU: COAXIAL_GPU=0 or no card, CI's."""
    return _SOLVER.get('cp') if _gpu_solver() is not None else None


def disparity(left, right):
    """(disparity px, kept) of the left image against the right (`_array_disparity`), on the card where one
    answers."""
    solver = _gpu_solver() if left.shape[:2] == (H, W) else None
    if solver is not None:
        return solver(left, right)
    return cpu_disparity(left, right)
