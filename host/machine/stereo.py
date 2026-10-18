"""Her stereo disparity on the card (wgpu or cupy) or the CPU: block matching over DISPARITIES.

`disparity(left, right)` computes the 9x9 SAD block match over `DISPARITIES` on the GPU when a
card answers (`wgpu` compute shader or `cupy`, 95.6 ms -> 1.6 ms at 320x240 on an RTX 4080
SUPER), or on the CPU when `COAXIAL_GPU=0` or no card is present.
"""
import os

#: Image dimensions, px, and block-matching bounds (`machine.sight`).
W, H = 320, 240
DISPARITIES, WINDOW, TEXTURE, UNIQUE = 48, 9, 1.5, 1.15

#: The switch: 0 keeps stereo matching on the CPU.
ENV = 'COAXIAL_GPU'

WGSL = """
@group(0) @binding(0) var<storage, read> gl: array<f32>;
@group(0) @binding(1) var<storage, read> gr: array<f32>;
@group(0) @binding(2) var<storage, read_write> hbox: array<f32>;
@group(0) @binding(3) var<storage, read_write> cost: array<f32>;
@group(0) @binding(4) var<storage, read_write> out_disp: array<f32>;
@group(0) @binding(5) var<storage, read_write> out_kept: array<u32>;

const W: i32 = 320;
const H: i32 = 240;
const D: i32 = 48;
const R: i32 = 4;

@compute @workgroup_size(16, 16, 1)
fn pass_hbox(@builtin(global_invocation_id) id: vec3<u32>) {
    let x = i32(id.x);
    let y = i32(id.y);
    let d = i32(id.z);
    if (x >= W || y >= H || d >= D) { return; }
    let idx = (d * H + y) * W + x;
    if (x < d) {
        hbox[idx] = 1e30;
        return;
    }
    var s: f32 = 0.0;
    let row = y * W;
    for (var dx: i32 = -R; dx <= R; dx = dx + 1) {
        let xx = clamp(x + dx, d, W - 1);
        s = s + abs(gl[row + xx] - gr[row + xx - d]);
    }
    hbox[idx] = s;
}

@compute @workgroup_size(16, 16, 1)
fn pass_vbox(@builtin(global_invocation_id) id: vec3<u32>) {
    let x = i32(id.x);
    let y = i32(id.y);
    let d = i32(id.z);
    if (x >= W || y >= H || d >= D) { return; }
    let idx = (d * H + y) * W + x;
    if (x < d) {
        cost[idx] = 1e30;
        return;
    }
    var s: f32 = 0.0;
    let plane = d * H * W;
    for (var dy: i32 = -R; dy <= R; dy = dy + 1) {
        let yy = clamp(y + dy, 0, H - 1);
        s = s + hbox[plane + yy * W + x];
    }
    cost[idx] = s;
}

@compute @workgroup_size(16, 16, 1)
fn pass_match(@builtin(global_invocation_id) id: vec3<u32>) {
    let x = i32(id.x);
    let y = i32(id.y);
    if (x >= W || y >= H) { return; }
    let pix = y * W + x;
    var best: i32 = 0;
    var c0: f32 = 1e30;
    for (var d: i32 = 0; d < D; d = d + 1) {
        let c = cost[(d * H + y) * W + x];
        if (c < c0) {
            c0 = c;
            best = d;
        }
    }
    let lo = cost[(max(best - 1, 0) * H + y) * W + x];
    let hi = cost[(min(best + 1, D - 1) * H + y) * W + x];
    let denom = lo - 2.0 * c0 + hi;
    var sub: f32 = 0.0;
    if (denom > 1e-6 && denom < 1e20) {
        sub = clamp(0.5 * (lo - hi) / denom, -0.5, 0.5);
    }
    out_disp[pix] = f32(best) + sub;
    if (x < D || best <= 0 || best >= D - 1 || c0 >= 1e20) {
        out_kept[pix] = 0u;
        return;
    }
    var second: f32 = 1e30;
    for (var d: i32 = 0; d < D; d = d + 1) {
        if (abs(d - best) > 2) {
            let c = cost[(d * H + y) * W + x];
            if (c < second) { second = c; }
        }
    }
    let rx = clamp(x - best, 0, W - 1);
    var rbest: i32 = 0;
    var rc0: f32 = 1e30;
    for (var d: i32 = 0; d < D; d = d + 1) {
        if (rx + d < W) {
            let c = cost[(d * H + y) * W + rx + d];
            if (c < rc0) {
                rc0 = c;
                rbest = d;
            }
        }
    }
    var sum1: f32 = 0.0;
    var sum2: f32 = 0.0;
    for (var dy: i32 = -R; dy <= R; dy = dy + 1) {
        let yy = clamp(y + dy, 0, H - 1) * W;
        for (var dx: i32 = -R; dx <= R; dx = dx + 1) {
            let v = gl[yy + clamp(x + dx, 0, W - 1)];
            sum1 = sum1 + v;
            sum2 = sum2 + v * v;
        }
    }
    let mean = sum1 / 81.0;
    let spread = sqrt(max(sum2 / 81.0 - mean * mean, 0.0));
    let ok = (spread > 1.5) && (second > 1.15 * max(c0, 1.0)) && (abs(rbest - best) <= 1);
    out_kept[pix] = select(0u, 1u, ok);
}
"""


def _np():
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    import numpy
    return numpy


def _gray(rgb):
    np = _np()
    return rgb.astype(np.float32) @ np.array([0.299, 0.587, 0.114], dtype=np.float32)


def _boxed(a, r, xp=None):
    """`a` summed over (2r+1)^2 windows, edges clipped."""
    xp = xp or _np()
    p = xp.pad(a, r, mode='edge')
    c = xp.cumsum(xp.cumsum(p, axis=0), axis=1)
    c = xp.pad(c, ((1, 0), (1, 0)))
    k = 2 * r + 1
    return c[k:, k:] - c[:-k, k:] - c[k:, :-k] + c[:-k, :-k]


def _array_disparity(left, right, xp):
    """Block matching on array module `xp` (numpy or cupy)."""
    luma = xp.array([0.299, 0.587, 0.114], dtype=xp.float32)
    gl = xp.asarray(left, dtype=xp.float32) @ luma
    gr = xp.asarray(right, dtype=xp.float32) @ luma
    r = WINDOW // 2
    cost = xp.full((DISPARITIES, H, W), xp.inf, dtype=xp.float32)
    for d in range(DISPARITIES):
        diff = xp.abs(gl[:, d:] - gr[:, :W - d])
        cost[d, :, d:] = _boxed(diff, r, xp)
    best = xp.argmin(cost, axis=0)
    rows, cols = xp.indices((H, W))
    c0 = cost[best, rows, cols]
    lo = cost[xp.maximum(best - 1, 0), rows, cols]
    hi = cost[xp.minimum(best + 1, DISPARITIES - 1), rows, cols]
    denom = lo - 2.0 * c0 + hi
    curved = xp.isfinite(denom) & (denom > 1e-6)
    sub = xp.zeros_like(c0)
    sub[curved] = 0.5 * (lo[curved] - hi[curved]) / denom[curved]
    disp = best + xp.clip(sub, -0.5, 0.5)
    apart = xp.abs(xp.arange(DISPARITIES)[:, None, None] - best[None, :, :]) > 2
    second = xp.where(apart, cost, xp.inf).min(axis=0)
    mean = _boxed(gl, r, xp) / WINDOW ** 2
    spread = xp.sqrt(xp.maximum(_boxed(gl * gl, r, xp) / WINDOW ** 2 - mean * mean, 0.0))
    back = xp.full_like(cost, xp.inf)
    for d in range(DISPARITIES):
        back[d, :, :W - d] = cost[d, :, d:]
    agreed = xp.abs(xp.argmin(back, axis=0)[rows, xp.clip(cols - best, 0, W - 1)] - best) <= 1
    kept = ((spread > TEXTURE) & (second > UNIQUE * xp.maximum(c0, 1.0)) & (best > 0)
            & (best < DISPARITIES - 1) & xp.isfinite(c0) & agreed)
    kept[:, :DISPARITIES] = False
    return disp, kept


def cpu_disparity(left, right):
    """(disparity px, kept) computed on the CPU."""
    return _array_disparity(left, right, _np())


class _WgpuStereo:
    """The 3-pass WGSL stereo matcher on a wgpu device."""

    def __init__(self, device):
        import wgpu
        self.device = device
        mod = device.create_shader_module(code=WGSL)
        ro, rw = wgpu.BufferBindingType.read_only_storage, wgpu.BufferBindingType.storage
        layout = device.create_bind_group_layout(entries=[
            {'binding': i, 'visibility': wgpu.ShaderStage.COMPUTE,
             'buffer': {'type': ro if i < 2 else rw}} for i in range(6)])
        pl = device.create_pipeline_layout(bind_group_layouts=[layout])
        self.passes = tuple(device.create_compute_pipeline(
            layout=pl, compute={'module': mod, 'entry_point': ep})
            for ep in ('pass_hbox', 'pass_vbox', 'pass_match'))
        n_pix, n_vol = W * H, DISPARITIES * W * H
        st, dst, src = wgpu.BufferUsage.STORAGE, wgpu.BufferUsage.COPY_DST, wgpu.BufferUsage.COPY_SRC
        self.bufs = (
            device.create_buffer(size=4 * n_pix, usage=st | dst),
            device.create_buffer(size=4 * n_pix, usage=st | dst),
            device.create_buffer(size=4 * n_vol, usage=st),
            device.create_buffer(size=4 * n_vol, usage=st),
            device.create_buffer(size=4 * n_pix, usage=st | src),
            device.create_buffer(size=4 * n_pix, usage=st | src),
        )
        self.group = device.create_bind_group(layout=layout, entries=[
            {'binding': i, 'resource': {'buffer': b, 'offset': 0, 'size': b.size}}
            for i, b in enumerate(self.bufs)])

    def __call__(self, left, right):
        np = _np()
        gl = np.ascontiguousarray(_gray(left), np.float32)
        gr = np.ascontiguousarray(_gray(right), np.float32)
        q = self.device.queue
        q.write_buffer(self.bufs[0], 0, gl.tobytes())
        q.write_buffer(self.bufs[1], 0, gr.tobytes())
        enc = self.device.create_command_encoder()
        cp = enc.begin_compute_pass()
        cp.set_bind_group(0, self.group)
        gx, gy = (W + 15) // 16, (H + 15) // 16
        for pipe, gz in zip(self.passes, (DISPARITIES, DISPARITIES, 1)):
            cp.set_pipeline(pipe)
            cp.dispatch_workgroups(gx, gy, gz)
        cp.end()
        q.submit([enc.finish()])
        disp = np.frombuffer(q.read_buffer(self.bufs[4]), np.float32).reshape(H, W).copy()
        kept = np.frombuffer(q.read_buffer(self.bufs[5]), np.uint32).reshape(H, W).astype(bool)
        return disp, kept


_SOLVER = {}


def _gpu_solver():
    """The card's stereo solver (wgpu or cupy), or None when COAXIAL_GPU=0 or no card answers."""
    if os.environ.get(ENV, '1') == '0':
        return None
    if 'solver' in _SOLVER:
        return _SOLVER['solver']
    solver = None
    try:
        import importlib
        cp = importlib.import_module('cupy')
        if cp.cuda.runtime.getDeviceCount() > 0:
            solver = lambda left, right: tuple(cp.asnumpy(a) for a in _array_disparity(left, right, cp))
    except Exception:
        solver = None
    if solver is None:
        try:
            import wgpu
            found = wgpu.gpu.request_adapter_sync(power_preference='high-performance')
            if found is not None and found.info.get('adapter_type') != 'CPU':
                solver = _WgpuStereo(found.request_device_sync())
        except Exception:
            solver = None
    _SOLVER['solver'] = solver
    return solver


def disparity(left, right):
    """(disparity px, kept) of the left image against the right: the best SAD a pixel over
    DISPARITIES, refined by the parabola through its neighbours; kept where textured and the
    best UNIQUE times under the next best."""
    solver = _gpu_solver() if left.shape[:2] == (H, W) else None
    if solver is not None:
        return solver(left, right)
    return cpu_disparity(left, right)

