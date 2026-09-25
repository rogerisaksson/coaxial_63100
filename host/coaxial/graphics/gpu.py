"""The staged engine's raster on a GPU (wgpu, any backend), and a crew that draws its frames there.

`crew_for` is what a page asks for: a `GpuCrew` where a card answers, else `crew.Crew`'s
processes. The GPU draws the dot raster - `engine.raster`'s contract, 100 ms -> 2.1 ms at
392x224 on a GTX 1080 Ti (2026-09-25) - and this process folds and shades it
(`crew.cells`). Against the CPU raster: 0-2 of 11 000 cells' dots differ, where coplanar
faces tie and f32 rounds another way than f64. COAXIAL_GPU=0 keeps every frame on the CPU.
"""
import os
from typing import Any

from coaxial.errors import RigError
from coaxial.graphics import crew as crewmod
from coaxial.graphics import engine

#: The switch: 0 keeps the pages on the CPU crew.
ENV = 'COAXIAL_GPU'

#: A sample is engine.raster's integer point (px, py) and a pixel's centre is at +0.5, so every
#: screen coordinate moves half a pixel. Depth is `ooz`, linear in screen space as the engine
#: interpolates it (clip w is 1), the nearest winning; the face's flags ride on its corners.
WGSL = """
struct U {
    m0: vec4<f32>, m1: vec4<f32>, m2: vec4<f32>,    // the attitude, row by row
    cam: vec4<f32>,                                  // cx, cy, scale, aspect
    cam2: vec4<f32>,                                 // distance, width, height, -
    near: vec4<f32>,                                 // w at the nearest, 1 / its span, -, -
};
@group(0) @binding(0) var<uniform> u: U;

struct VOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) @interpolate(linear) ooz: f32,
    @location(1) @interpolate(flat) flags: u32,
};

@vertex
fn vs(@location(0) p: vec3<f32>, @location(1) flags: u32) -> VOut {
    let tx = dot(u.m0.xyz, p);
    let ty = dot(u.m1.xyz, p);
    let tz = dot(u.m2.xyz, p);
    let w = 1.0 / (u.cam2.x - tz);
    let sx = u.cam.x + u.cam.z * w * tx;
    let sy = u.cam.y - u.cam.z * u.cam.w * w * ty;
    var o: VOut;
    o.pos = vec4<f32>((sx + 0.5) / u.cam2.y * 2.0 - 1.0,
                      1.0 - (sy + 0.5) / u.cam2.z * 2.0,
                      clamp((u.near.x - w) * u.near.y, 0.0, 1.0), 1.0);
    o.ooz = w;
    o.flags = flags;
    return o;
}

struct FOut {
    @location(0) ooz: f32,
    @location(1) flags: u32,
};

@fragment
fn fs(i: VOut) -> FOut {
    var f: FOut;
    f.ooz = i.ooz;
    f.flags = i.flags;
    return f;
}
"""


def adapter():
    """The card's adapter, or None: wgpu absent, no GPU but a software one, or
    COAXIAL_GPU=0."""
    if os.environ.get(ENV, '1') == '0':
        return None
    try:
        import wgpu
    except ImportError:
        return None
    try:
        found = wgpu.gpu.request_adapter_sync(power_preference='high-performance')
    except (RuntimeError, OSError, wgpu.GPUError):
        return None
    if found is None or found.info.get('adapter_type') == 'CPU':
        return None
    return found


#: id(device) -> (device, bind group layout, pipeline): the shader compiled once a device.
_PIPELINES = {}


def _pipeline(device):
    """The raster's bind group layout and render pipeline on `device`."""
    import wgpu
    got = _PIPELINES.get(id(device))
    if got is not None and got[0] is device:
        return got[1], got[2]
    shader = device.create_shader_module(code=WGSL)
    layout = device.create_bind_group_layout(entries=[{
        'binding': 0, 'visibility': wgpu.ShaderStage.VERTEX | wgpu.ShaderStage.FRAGMENT,
        'buffer': {'type': wgpu.BufferBindingType.uniform}}])
    pipeline = device.create_render_pipeline(
        layout=device.create_pipeline_layout(bind_group_layouts=[layout]),
        vertex={'module': shader, 'entry_point': 'vs', 'buffers': [
            {'array_stride': 12, 'step_mode': wgpu.VertexStepMode.vertex, 'attributes': [
                {'format': wgpu.VertexFormat.float32x3, 'offset': 0, 'shader_location': 0}]},
            {'array_stride': 4, 'step_mode': wgpu.VertexStepMode.vertex, 'attributes': [
                {'format': wgpu.VertexFormat.uint32, 'offset': 0, 'shader_location': 1}]}]},
        primitive={'topology': wgpu.PrimitiveTopology.triangle_list,
                   'cull_mode': wgpu.CullMode.none},
        depth_stencil={'format': wgpu.TextureFormat.depth32float,
                       'depth_write_enabled': True,
                       'depth_compare': wgpu.CompareFunction.less},
        fragment={'module': shader, 'entry_point': 'fs', 'targets': [
            {'format': wgpu.TextureFormat.r32float},
            {'format': wgpu.TextureFormat.r32uint}]})
    _PIPELINES[id(device)] = (device, layout, pipeline)
    return layout, pipeline


class GpuRaster:
    """One solid uploaded once; `raster()` as engine.raster for the whole frame, as arrays."""

    def __init__(self, solid, device):
        import wgpu

        from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
        self.device = device
        pos, idx, nrm = solid
        p = np.asarray(pos, float).reshape(-1, 3)
        t = np.asarray(idx, int).reshape(-1, 3)
        #: The body-frame normals in f64: a face's flags are the engine's own arithmetic.
        self.normals = np.asarray(nrm, float).reshape(-1, 3)
        #: How far out the model reaches: the depth buffer's range.
        self.reach = float(np.sqrt((p * p).sum(axis=1)).max())
        # Non-indexed, every corner its own vertex, so a face's flags ride on all three.
        corners = p[t].reshape(-1, 3)
        self.count = len(corners)
        self.corners = device.create_buffer_with_data(
            data=corners.astype(np.float32).tobytes(), usage=wgpu.BufferUsage.VERTEX)
        self.flags = device.create_buffer(
            size=4 * self.count, usage=wgpu.BufferUsage.VERTEX | wgpu.BufferUsage.COPY_DST)
        self.uniform = device.create_buffer(
            size=96, usage=wgpu.BufferUsage.UNIFORM | wgpu.BufferUsage.COPY_DST)
        layout, self.pipeline = _pipeline(device)
        self.group = device.create_bind_group(layout=layout, entries=[{
            'binding': 0, 'resource': {'buffer': self.uniform, 'offset': 0, 'size': 96}}])
        self._targets = {}

    def _targets_for(self, width, height):
        got = self._targets.get((width, height))
        if got is None:
            import wgpu
            out = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.COPY_SRC
            made = [self.device.create_texture(size=(width, height, 1), format=fmt, usage=use)
                    for fmt, use in ((wgpu.TextureFormat.r32float, out),
                                     (wgpu.TextureFormat.r32uint, out),
                                     (wgpu.TextureFormat.depth32float,
                                      wgpu.TextureUsage.RENDER_ATTACHMENT))]
            self._targets.clear()                  # one size at a time: the window's
            got = self._targets[(width, height)] = made
        return got

    def _face_flags(self, m, beam, sun_min):
        """Each face's (flat, lit) as engine.raster decides them, in f64."""
        from coaxial.model.blocks import numpy as np
        lx, ly, lz = beam if beam else (0.0, 0.0, 1.0)
        bx, by, bz = self.normals[:, 0], self.normals[:, 1], self.normals[:, 2]
        nx = m[0] * bx + m[1] * by + m[2] * bz
        ny = m[3] * bx + m[4] * by + m[5] * bz
        nz = m[6] * bx + m[7] * by + m[8] * bz
        flip = nz < 0.0
        nx, ny, nz = np.where(flip, -nx, nx), np.where(flip, -ny, ny), np.where(flip, -nz, nz)
        lit = (bz > 0.0) & (nx * lx + ny * ly + nz * lz > sun_min)
        flat = (bz > 0.9) | (bz < -0.9)
        return np.repeat(flat.astype(np.uint32) | (lit.astype(np.uint32) << 1), 3)

    def raster(self, m, cam, beam=None, sun_min=0.0):
        """(depth, top, sun) at `cam`'s samples - the fine camera - as (height, width) arrays."""
        import wgpu

        from coaxial.model.blocks import numpy as np
        width, height, distance = cam['width'], cam['height'], cam['distance']
        near, far = 1.0 / (distance - self.reach), 1.0 / (distance + self.reach)
        queue = self.device.queue
        queue.write_buffer(self.uniform, 0, np.array(
            [m[0], m[1], m[2], 0.0, m[3], m[4], m[5], 0.0, m[6], m[7], m[8], 0.0,
             cam['cx'], cam['cy'], cam['scale'], cam.get('aspect', 0.5),
             distance, width, height, 0.0, near, 1.0 / (near - far), 0.0, 0.0],
            np.float32).tobytes())
        queue.write_buffer(self.flags, 0, self._face_flags(m, beam, sun_min).tobytes())
        depth_t, flag_t, z_t = self._targets_for(width, height)
        encoder = self.device.create_command_encoder()
        colour = {'load_op': wgpu.LoadOp.clear, 'store_op': wgpu.StoreOp.store,
                  'clear_value': (0, 0, 0, 0)}
        rp = encoder.begin_render_pass(
            color_attachments=[dict(colour, view=depth_t.create_view()),
                               dict(colour, view=flag_t.create_view())],
            depth_stencil_attachment={'view': z_t.create_view(), 'depth_clear_value': 1.0,
                                      'depth_load_op': wgpu.LoadOp.clear,
                                      'depth_store_op': wgpu.StoreOp.store})
        rp.set_pipeline(self.pipeline)
        rp.set_bind_group(0, self.group)
        rp.set_vertex_buffer(0, self.corners)
        rp.set_vertex_buffer(1, self.flags)
        rp.draw(self.count)
        rp.end()
        queue.submit([encoder.finish()])
        depth = _read(queue, depth_t, width, height, np.float32)
        flags = _read(queue, flag_t, width, height, np.uint32)
        return depth, (flags & 1).astype(np.uint8), ((flags >> 1) & 1).astype(np.uint8)


#: A lit mesh: world positions, smooth normals, a (u, v) and a material a corner, seen through the
#: engine's own projection about `centre`. A pixel's colour is its material's, lit by a key and a
#: fill, a rim where the surface turns away and a highlight; MESH wears a lattice in (u, v).
LIT_WGSL = """
struct U {
    m0: vec4<f32>, m1: vec4<f32>, m2: vec4<f32>,    // the view, row by row
    cam: vec4<f32>,                                  // cx, cy, scale, aspect
    cam2: vec4<f32>,                                 // distance, width, height, -
    near: vec4<f32>,                                 // w at the nearest, 1 / its span, -, -
    centre: vec4<f32>,                               // the model point the view turns about
    key: vec4<f32>,                                  // the key light, view space
    fill: vec4<f32>,                                 // the fill light, view space
};
@group(0) @binding(0) var<uniform> u: U;

struct VOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) @interpolate(linear) ooz: f32,
    @location(1) @interpolate(linear) n: vec3<f32>,
    @location(2) @interpolate(linear) uv: vec2<f32>,
    @location(3) @interpolate(flat) material: u32,
};

@vertex
fn vs(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>, @location(2) uv: vec2<f32>,
      @location(3) material: u32) -> VOut {
    let q = p - u.centre.xyz;
    let tx = dot(u.m0.xyz, q);
    let ty = dot(u.m1.xyz, q);
    let tz = dot(u.m2.xyz, q);
    let w = 1.0 / (u.cam2.x - tz);
    let sx = u.cam.x + u.cam.z * w * tx;
    let sy = u.cam.y - u.cam.z * u.cam.w * w * ty;
    var o: VOut;
    o.pos = vec4<f32>((sx + 0.5) / u.cam2.y * 2.0 - 1.0,
                      1.0 - (sy + 0.5) / u.cam2.z * 2.0,
                      clamp((u.near.x - w) * u.near.y, 0.0, 1.0), 1.0);
    o.ooz = w;
    o.n = vec3<f32>(dot(u.m0.xyz, n), dot(u.m1.xyz, n), dot(u.m2.xyz, n));
    o.uv = uv;
    o.material = material;
    return o;
}

struct FOut {
    @location(0) ooz: f32,
    @location(1) colour: vec4<f32>,
};

@fragment
fn fs(i: VOut) -> FOut {
    var palette = array<vec3<f32>, 4>(vec3<f32>(0.58, 0.64, 0.72), vec3<f32>(0.96, 0.79, 0.69),
                                      vec3<f32>(0.80, 0.83, 0.88), vec3<f32>(0.35, 0.85, 1.0));
    var n = normalize(i.n);
    if (n.z < 0.0) { n = -n; }
    let key = max(dot(n, u.key.xyz), 0.0);
    let fill = 0.35 * max(dot(n, u.fill.xyz), 0.0);
    let rim = pow(1.0 - clamp(n.z, 0.0, 1.0), 3.0);
    let spec = pow(max(dot(n, normalize(u.key.xyz + vec3<f32>(0.0, 0.0, 1.0))), 0.0), 40.0);
    var base = palette[min(i.material, 3u)];
    var shine = 0.55;
    if (i.material == 0u) {
        let g = abs(fract(i.uv * vec2<f32>(22.0, 30.0)) - 0.5);
        base = base * (1.0 - 0.5 * smoothstep(0.36, 0.46, max(g.x, g.y)));
    }
    if (i.material == 1u) { shine = 0.12; }
    var c = base * (0.10 + 0.85 * key + fill) + vec3<f32>(spec * shine)
            + vec3<f32>(0.55, 0.75, 1.0) * rim * 0.8;
    if (i.material == 3u) { c = base * (0.75 + 0.25 * key); }
    var f: FOut;
    f.ooz = i.ooz;
    f.colour = vec4<f32>(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), 1.0);
    return f;
}
"""


class LitRaster:
    """A mesh posed anew each frame, lit on the GPU: `raster()` -> (depth, colour) arrays at the
    fine camera's samples, colour 0..255 RGB."""

    def __init__(self, device=None, found=None):
        import wgpu
        self.name = 'GPU'
        if device is None:
            found = found or adapter()
            if found is None:
                raise RigError('no GPU to draw on')
            device = found.request_device_sync()
            self.name = found.info.get('device', 'GPU')
        self.device = device
        shader = device.create_shader_module(code=LIT_WGSL)
        layout = device.create_bind_group_layout(entries=[{
            'binding': 0, 'visibility': wgpu.ShaderStage.VERTEX | wgpu.ShaderStage.FRAGMENT,
            'buffer': {'type': wgpu.BufferBindingType.uniform}}])
        self.uniform = device.create_buffer(
            size=144, usage=wgpu.BufferUsage.UNIFORM | wgpu.BufferUsage.COPY_DST)
        self.group = device.create_bind_group(layout=layout, entries=[{
            'binding': 0, 'resource': {'buffer': self.uniform, 'offset': 0, 'size': 144}}])
        attrs = ((wgpu.VertexFormat.float32x3, 12), (wgpu.VertexFormat.float32x3, 12),
                 (wgpu.VertexFormat.float32x2, 8), (wgpu.VertexFormat.uint32, 4))
        self.pipeline = device.create_render_pipeline(
            layout=device.create_pipeline_layout(bind_group_layouts=[layout]),
            vertex={'module': shader, 'entry_point': 'vs', 'buffers': [
                {'array_stride': size, 'step_mode': wgpu.VertexStepMode.vertex,
                 'attributes': [{'format': fmt, 'offset': 0, 'shader_location': k}]}
                for k, (fmt, size) in enumerate(attrs)]},
            primitive={'topology': wgpu.PrimitiveTopology.triangle_list,
                       'cull_mode': wgpu.CullMode.none},
            depth_stencil={'format': wgpu.TextureFormat.depth32float,
                           'depth_write_enabled': True,
                           'depth_compare': wgpu.CompareFunction.less},
            fragment={'module': shader, 'entry_point': 'fs', 'targets': [
                {'format': wgpu.TextureFormat.r32float},
                {'format': wgpu.TextureFormat.rgba8unorm}]})
        #: (vertex buffers, index buffer, index count) for the mesh's size, made on first sight.
        self._mesh: tuple = ((), None, 0)
        self._targets = {}

    def _upload(self, arrays, index):
        import wgpu
        buffers, indices, count = self._mesh
        if not buffers or buffers[0].size != arrays[0].nbytes:
            buffers = [self.device.create_buffer(
                size=a.nbytes, usage=wgpu.BufferUsage.VERTEX | wgpu.BufferUsage.COPY_DST)
                for a in arrays]
            indices = self.device.create_buffer_with_data(
                data=index.tobytes(), usage=wgpu.BufferUsage.INDEX)
            count = len(index)
            self._mesh = (buffers, indices, count)
        for buffer, a in zip(buffers, arrays):
            self.device.queue.write_buffer(buffer, 0, a.tobytes())
        return buffers, indices, count

    def raster(self, positions, normals, uv, material, index, m, cam, centre, reach,
               key=(-0.45, 0.62, 0.64), fill=(0.7, 0.1, 0.7)):
        """(depth, colour): (height, width) f32 `ooz` (0 uncovered) and (height, width, 3) u8, the
        mesh `index` triangles over per-corner arrays, `m` the view about `centre`. The index is
        fixed for a mesh; the rest may change every frame."""
        import wgpu

        from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
        width, height, distance = cam['width'], cam['height'], cam['distance']
        near, far = 1.0 / (distance - reach), 1.0 / (distance + reach)

        def unit(v):
            v = np.asarray(v, float)
            return v / np.linalg.norm(v)

        self.device.queue.write_buffer(self.uniform, 0, np.concatenate([
            np.asarray(m[0:3] + (0.0,) + m[3:6] + (0.0,) + m[6:9] + (0.0,)),
            [cam['cx'], cam['cy'], cam['scale'], cam.get('aspect', 0.5),
             distance, width, height, 0.0, near, 1.0 / (near - far), 0.0, 0.0],
            list(centre) + [0.0], list(unit(key)) + [0.0], list(unit(fill)) + [0.0]])
            .astype(np.float32).tobytes())
        buffers, indices, count = self._upload(
            [np.ascontiguousarray(positions, np.float32),
             np.ascontiguousarray(normals, np.float32),
             np.ascontiguousarray(uv, np.float32),
             np.ascontiguousarray(material, np.uint32)],
            np.ascontiguousarray(index, np.uint32).ravel())
        got = self._targets.get((width, height))
        if got is None:
            out = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.COPY_SRC
            self._targets.clear()
            got = self._targets[(width, height)] = [
                self.device.create_texture(size=(width, height, 1), format=fmt, usage=use)
                for fmt, use in ((wgpu.TextureFormat.r32float, out),
                                 (wgpu.TextureFormat.rgba8unorm, out),
                                 (wgpu.TextureFormat.depth32float,
                                  wgpu.TextureUsage.RENDER_ATTACHMENT))]
        depth_t, colour_t, z_t = got
        encoder = self.device.create_command_encoder()
        clear = {'load_op': wgpu.LoadOp.clear, 'store_op': wgpu.StoreOp.store,
                 'clear_value': (0, 0, 0, 0)}
        rp = encoder.begin_render_pass(
            color_attachments=[dict(clear, view=depth_t.create_view()),
                               dict(clear, view=colour_t.create_view())],
            depth_stencil_attachment={'view': z_t.create_view(), 'depth_clear_value': 1.0,
                                      'depth_load_op': wgpu.LoadOp.clear,
                                      'depth_store_op': wgpu.StoreOp.store})
        rp.set_pipeline(self.pipeline)
        rp.set_bind_group(0, self.group)
        for k, buffer in enumerate(buffers):
            rp.set_vertex_buffer(k, buffer)
        rp.set_index_buffer(indices, wgpu.IndexFormat.uint32)
        rp.draw_indexed(count)
        rp.end()
        self.device.queue.submit([encoder.finish()])
        depth = _read(self.device.queue, depth_t, width, height, np.float32)
        colour = _read(self.device.queue, colour_t, width, height, np.uint8)
        return depth, colour.reshape(height, width, 4)[..., :3]


def _read(queue, texture, width, height, dtype) -> Any:
    """A 4-byte-a-pixel texture back as a (height, width) array of `dtype`, raw."""
    from coaxial.model.blocks import numpy as np
    raw = queue.read_texture({'texture': texture},
                             {'offset': 0, 'bytes_per_row': 4 * width, 'rows_per_image': height},
                             (width, height, 1))
    got = np.frombuffer(raw, dtype)
    return got.reshape(height, -1) if got.size else got


#: Solids a crew keeps on the card: the board's levels of detail and a page's own.
SOLIDS_KEPT = 12


class GpuCrew:
    """`crew.Crew`'s calls, the raster on the GPU and the cells in this process: nothing to
    spawn, and any solid drawn - uploaded the first time it is seen, kept by identity."""

    def __init__(self, solids=(), art=None, found=None):
        found = found or adapter()
        if found is None:
            raise RigError('no GPU to draw on')
        self.device = found.request_device_sync()
        self.art = art
        self.name = found.info.get('device', 'GPU')
        #: id -> (solid, its GpuRaster): the solid held so its id stays its own.
        self.rasters: dict | None = {}
        for solid in solids:
            self._raster_of(solid)
        #: A frame drawn and not yet collected - `Crew`'s pipelining, drawn at submit.
        self.pending = 0
        self._frame = None

    def _raster_of(self, solid):
        if self.rasters is None:
            raise RigError('the crew is closed')
        got = self.rasters.get(id(solid))
        if got is None or got[0] is not solid:
            if len(self.rasters) >= SOLIDS_KEPT:
                self.rasters.clear()
            got = self.rasters[id(solid)] = (solid, GpuRaster(solid, self.device))
        return got[1]

    def holds(self, solid):
        """Always: a solid not on the card yet is uploaded when it is drawn."""
        return self.rasters is not None

    def close(self):
        self.rasters = None

    def __enter__(self):
        return self

    def __exit__(self, *exc_info):
        self.close()
        return False

    def submit(self, solid, m, cam, beam, sun_min, shading):
        """This frame, drawn: the GPU's raster, folded and shaded here."""
        if self.pending:
            raise RigError('a frame is in flight; collect it first')
        depth, top, sun = self._raster_of(solid).raster(m, engine.fine(cam), beam, sun_min)
        self._frame = crewmod.cells(depth, top, sun, m, cam, (0, cam['height']), shading,
                                    self.art)
        self.pending = 1

    def collect(self):
        """The frame `submit` drew, as `Crew.collect` hands it over."""
        if not self.pending:
            raise RigError('nothing is in flight')
        self.pending = 0
        frame, self._frame = self._frame, None
        return frame

    def raster(self, solid, m, cam, beam=None, sun_min=0.0):
        """(depth, top, sun, coverage, reached) at cell resolution for the whole frame."""
        self.submit(solid, m, cam, beam, sun_min, None)
        return self.collect()

    def frame(self, solid, m, cam, beam, sun_min, shading):
        """(depth, coverage, reached, classes, levels, bare, seed) for the whole frame."""
        self.submit(solid, m, cam, beam, sun_min, shading)
        return self.collect()


def card_crew(solids=(), art=None):
    """A `GpuCrew` where a card answers, else None: for a page that otherwise draws in its
    own process."""
    found = adapter()
    if found is None:
        return None
    try:
        return GpuCrew(solids, art, found)
    except (RigError, RuntimeError, OSError):
        return None


def crew_for(solids, art=None, workers=None):
    """A `GpuCrew` where a card answers, else `crew.Crew`'s processes."""
    return card_crew(solids, art) or crewmod.Crew(solids, art, workers)
