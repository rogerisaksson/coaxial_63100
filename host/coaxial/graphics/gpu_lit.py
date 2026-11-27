"""Lit 3D Braille mesh rasterizer on a GPU (wgpu) with 4x RGSS (4-rooks) sub-dot FSAA."""
from typing import Any

from coaxial.errors import RigError
from coaxial.graphics.card import adapter, device_of
from coaxial.graphics.lit import FILL, GLOSS, KEY, LAYER_M

LIT_WGSL = """
struct U {
    m0: vec4<f32>, m1: vec4<f32>, m2: vec4<f32>,
    cam: vec4<f32>,
    cam2: vec4<f32>,
    near: vec4<f32>,
    centre: vec4<f32>,
    key: vec4<f32>,
    fill: vec4<f32>,
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
      @location(3) material: u32, @builtin(instance_index) s: u32) -> VOut {
    var rooks = array<vec2<f32>, 4>(
        vec2<f32>(-0.375, -0.125), vec2<f32>(0.125, -0.375),
        vec2<f32>(-0.125, 0.375), vec2<f32>(0.375, 0.125));
    let off = select(vec2<f32>(0.0, 0.0), rooks[s & 3u], u.cam2.w > 0.5);
    let q = p - u.centre.xyz;
    let tx = dot(u.m0.xyz, q);
    let ty = dot(u.m1.xyz, q);
    let tz = dot(u.m2.xyz, q);
    let w = 1.0 / (u.cam2.x - tz);
    let under = 1.0 / (u.cam2.x - tz + select(0.0, u.near.z, (material & 67108864u) != 0u));
    let sx = u.cam.x + u.cam.z * w * tx;
    let sy = u.cam.y - u.cam.z * u.cam.w * w * ty;
    var o: VOut;
    o.pos = vec4<f32>((sx - off.x + 0.5) / u.cam2.y * 2.0 - 1.0,
                      1.0 - (sy - off.y + 0.5) / u.cam2.z * 2.0,
                      clamp((u.near.x - under) * u.near.y, 0.0, 1.0), 1.0);
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
    let lobe = max(dot(n, normalize(u.key.xyz + vec3<f32>(0.0, 0.0, 1.0))), 0.0);
    var spec = pow(lobe, 40.0);
    var base = palette[min(i.material, 3u)];
    var shine = 0.55;
    if (i.material >= 16777216u) {
        let c = i.material & 16777215u;
        base = vec3<f32>(f32((c >> 16u) & 255u), f32((c >> 8u) & 255u), f32(c & 255u)) / 255.0;
        shine = 0.3;
    }
    if (i.material == 0u) {
        let g = abs(fract(i.uv * vec2<f32>(22.0, 30.0)) - 0.5);
        base = base * (1.0 - 0.5 * smoothstep(0.36, 0.46, max(g.x, g.y)));
    }
    if (i.material == 1u) { shine = 0.12; }
    var edge = 0.8;
    var screen = 1.0;
    if ((i.material & 33554432u) != 0u) { shine = 0.0; edge = 0.15; }
    if ((i.material & 134217728u) != 0u) {
        shine = u.fill.w;
        spec = pow(lobe, 160.0) + pow(lobe, 28.0) / 1.5;
        screen = select(128.0 / 255.0, 0.0, n.x > 0.0);
    }
    if ((i.material & 268435456u) != 0u) {
        screen = select(224.0 / 255.0, screen + 64.0 / 255.0, screen < 1.0);
        edge = 0.3;
    }
    var c = base * (0.10 + 0.85 * key + fill) + vec3<f32>(spec * shine)
            + vec3<f32>(0.55, 0.75, 1.0) * rim * edge;
    if (i.material == 3u) { c = base * (0.75 + 0.25 * key); }
    var f: FOut;
    f.ooz = i.ooz;
    f.colour = vec4<f32>(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), screen);
    return f;
}
"""

RESOLVE_WGSL = """
@group(0) @binding(0) var depth_hi: texture_2d<f32>;
@group(0) @binding(1) var colour_hi: texture_2d<f32>;

struct FOut {
    @location(0) ooz: f32,
    @location(1) colour: vec4<f32>,
};

@vertex
fn vs(@builtin(vertex_index) vi: u32) -> @builtin(position) vec4<f32> {
    let x = f32((vi << 1u) & 2u) * 2.0 - 1.0;
    let y = 1.0 - f32(vi & 2u) * 2.0;
    return vec4<f32>(x, y, 0.0, 1.0);
}

@fragment
fn fs(@builtin(position) pos: vec4<f32>) -> FOut {
    let dims = vec2<i32>(textureDimensions(depth_hi)) / 2;
    let p = vec2<i32>(i32(pos.x), i32(pos.y));
    var offs = array<vec2<i32>, 4>(
        vec2<i32>(0, 0), vec2<i32>(dims.x, 0),
        vec2<i32>(0, dims.y), vec2<i32>(dims.x, dims.y));
    var d_arr = array<f32, 4>(0.0, 0.0, 0.0, 0.0);
    var a_arr = array<u32, 4>(255u, 255u, 255u, 255u);
    var sum_rgb = vec3<f32>(0.0);
    var any_hits = 0;
    var best_o = 0.0;
    var sec_o = 0.0;
    var best_s = 0u;
    var sec_s = 0u;
    for (var s = 0u; s < 4u; s = s + 1u) {
        let q = p + offs[s];
        let d = textureLoad(depth_hi, q, 0).x;
        d_arr[s] = d;
        if (d > 0.0) {
            any_hits = any_hits + 1;
            let c = textureLoad(colour_hi, q, 0);
            sum_rgb = sum_rgb + c.rgb;
            a_arr[s] = u32(round(c.a * 255.0));
            if (d > best_o) {
                sec_o = best_o;
                sec_s = best_s;
                best_o = d;
                best_s = s;
            } else if (d > sec_o) {
                sec_o = d;
                sec_s = s;
            }
        }
    }
    var out: FOut;
    if (any_hits == 0) {
        out.ooz = 0.0;
        out.colour = vec4<f32>(0.0);
        return out;
    }
    var fg_hits = 0;
    for (var s = 0u; s < 4u; s = s + 1u) {
        if (d_arr[s] >= best_o * 0.96) {
            fg_hits = fg_hits + 1;
        }
    }
    var rep_s = best_s;
    if (fg_hits == 1 && any_hits >= 2) {
        rep_s = sec_s;
    }
    let h_rep = a_arr[rep_s];
    let miss = u32(4 - any_hits);
    let out_h = select(h_rep + miss, 255u - miss, h_rep >= 248u);
    out.ooz = d_arr[rep_s];
    out.colour = vec4<f32>(sum_rgb / f32(any_hits), f32(out_h) / 255.0);
    return out;
}
"""


def _read(queue, texture, width, height, dtype) -> Any:
    """A 4-byte-a-pixel texture back as a `(height, width)` array of `dtype`, raw."""
    from coaxial.model.blocks import numpy as np
    raw = queue.read_texture({'texture': texture},
                             {'offset': 0, 'bytes_per_row': 4 * width, 'rows_per_image': height},
                             (width, height, 1))
    got = np.frombuffer(raw, dtype)
    return got.reshape(height, -1) if got.size else got


class LitRaster:
    """A mesh posed anew each frame, lit on the GPU with 4x RGSS (4-rooks) sub-dot FSAA:
    `raster()` -> `(depth, colour)` arrays at the fine camera's dot samples."""

    def __init__(self, device=None, found=None, fsaa=True):
        import wgpu
        self.name = 'GPU'
        self.fsaa = bool(fsaa)
        if device is None:
            found = found or adapter()
            if found is None:
                raise RigError('no GPU to draw on')
            device = device_of(found)
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
        res_shader = device.create_shader_module(code=RESOLVE_WGSL)
        self.res_layout = device.create_bind_group_layout(entries=[
            {'binding': 0, 'visibility': wgpu.ShaderStage.FRAGMENT,
             'texture': {'sample_type': wgpu.TextureSampleType.unfilterable_float,
                         'view_dimension': wgpu.TextureViewDimension.d2, 'multisampled': False}},
            {'binding': 1, 'visibility': wgpu.ShaderStage.FRAGMENT,
             'texture': {'sample_type': wgpu.TextureSampleType.float,
                         'view_dimension': wgpu.TextureViewDimension.d2, 'multisampled': False}}])
        self.res_pipeline = device.create_render_pipeline(
            layout=device.create_pipeline_layout(bind_group_layouts=[self.res_layout]),
            vertex={'module': res_shader, 'entry_point': 'vs', 'buffers': []},
            primitive={'topology': wgpu.PrimitiveTopology.triangle_list,
                       'cull_mode': wgpu.CullMode.none},
            fragment={'module': res_shader, 'entry_point': 'fs', 'targets': [
                {'format': wgpu.TextureFormat.r32float},
                {'format': wgpu.TextureFormat.rgba8unorm}]})
        self._mesh: tuple = ((), None, 0)
        self._targets: dict = {}

    def _upload(self, arrays, index):
        import wgpu
        buffers, indices, count = self._mesh
        if (not buffers or len(buffers) != len(arrays)
                or any(b.size != a.nbytes for b, a in zip(buffers, arrays))
                or len(index) != count):
            buffers = [self.device.create_buffer(
                size=a.nbytes, usage=wgpu.BufferUsage.VERTEX | wgpu.BufferUsage.COPY_DST)
                for a in arrays]
            indices = self.device.create_buffer(
                size=index.nbytes, usage=wgpu.BufferUsage.INDEX | wgpu.BufferUsage.COPY_DST)
            count = len(index)
            self._mesh = (buffers, indices, count)
        for buffer, a in zip(buffers, arrays):
            self.device.queue.write_buffer(buffer, 0, a.tobytes())
        self.device.queue.write_buffer(indices, 0, index.tobytes())
        return buffers, indices, count

    def _targets_for(self, width, height, fsaa):
        key = (width, height, bool(fsaa))
        got = self._targets.get(key)
        if got is not None:
            return got
        import wgpu
        out = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.COPY_SRC
        hi = wgpu.TextureUsage.RENDER_ATTACHMENT | wgpu.TextureUsage.TEXTURE_BINDING
        self._targets.clear()
        depth_t = self.device.create_texture(
            size=(width, height, 1), format=wgpu.TextureFormat.r32float, usage=out)
        colour_t = self.device.create_texture(
            size=(width, height, 1), format=wgpu.TextureFormat.rgba8unorm, usage=out)
        if not fsaa:
            z_t = self.device.create_texture(
                size=(width, height, 1), format=wgpu.TextureFormat.depth32float,
                usage=wgpu.TextureUsage.RENDER_ATTACHMENT)
            got = (depth_t, colour_t, z_t, None, None, None)
        else:
            depth_hi = self.device.create_texture(
                size=(width * 2, height * 2, 1), format=wgpu.TextureFormat.r32float, usage=hi)
            colour_hi = self.device.create_texture(
                size=(width * 2, height * 2, 1), format=wgpu.TextureFormat.rgba8unorm, usage=hi)
            z_hi = self.device.create_texture(
                size=(width * 2, height * 2, 1), format=wgpu.TextureFormat.depth32float,
                usage=wgpu.TextureUsage.RENDER_ATTACHMENT)
            res_group = self.device.create_bind_group(layout=self.res_layout, entries=[
                {'binding': 0, 'resource': depth_hi.create_view()},
                {'binding': 1, 'resource': colour_hi.create_view()}])
            got = (depth_t, colour_t, z_hi, depth_hi, colour_hi, res_group)
        self._targets[key] = got
        return got

    def raster(self, positions, normals, uv, material, index, m, cam, centre, reach,
               key=KEY, fill=FILL, fsaa=None):
        """`(depth, colour)`: `(height, width)` f32 `ooz` (0 uncovered) and `(height, width, 4)` u8
        RGB plus sub-dot coverage/screen byte, rendered at 4-rooks FSAA when `fsaa` is True."""
        import wgpu
        from coaxial.model.blocks import numpy as np
        use_fsaa = self.fsaa if fsaa is None else bool(fsaa)
        width, height, distance = int(cam['width']), int(cam['height']), float(cam['distance'])
        pos_c = np.ascontiguousarray(positions, np.float32).reshape(-1, 3)
        nrm_c = np.ascontiguousarray(normals, np.float32).reshape(-1, 3)
        uv_c = np.ascontiguousarray(uv, np.float32).reshape(-1, 2)
        mat_c = np.ascontiguousarray(material, np.uint32).ravel()
        idx_c = np.ascontiguousarray(index, np.uint32).ravel()
        if width <= 0 or height <= 0 or len(pos_c) == 0 or len(idx_c) == 0:
            h0, w0 = max(0, height), max(0, width)
            return np.zeros((h0, w0), np.float32), np.zeros((h0, w0, 4), np.uint8)
        near, far = 1.0 / max(1e-3, distance - reach), 1.0 / (distance + reach)

        def unit(v):
            v = np.asarray(v, float)
            return v / np.linalg.norm(v)

        mf = tuple(np.asarray(m, float).ravel())
        self.device.queue.write_buffer(self.uniform, 0, np.concatenate([
            np.asarray(mf[0:3] + (0.0,) + mf[3:6] + (0.0,) + mf[6:9] + (0.0,)),
            [cam['cx'], cam['cy'], cam['scale'], cam.get('aspect', 0.5),
             distance, width, height, 1.0 if use_fsaa else 0.0,
             near, 1.0 / max(1e-6, near - far), LAYER_M, 0.0],
            list(centre) + [0.0], list(unit(key)) + [0.0], list(unit(fill)) + [GLOSS]])
            .astype(np.float32).tobytes())
        buffers, indices, count = self._upload([pos_c, nrm_c, uv_c, mat_c], idx_c)
        depth_t, colour_t, z_t, depth_hi, colour_hi, res_group = self._targets_for(
            width, height, use_fsaa)
        encoder = self.device.create_command_encoder()
        clear = {'load_op': wgpu.LoadOp.clear, 'store_op': wgpu.StoreOp.store,
                 'clear_value': (0, 0, 0, 0)}
        if use_fsaa:
            assert depth_hi is not None and colour_hi is not None and res_group is not None
            tgt_d, tgt_c = depth_hi, colour_hi
        else:
            tgt_d, tgt_c = depth_t, colour_t
        rp = encoder.begin_render_pass(
            color_attachments=[dict(clear, view=tgt_d.create_view()),
                               dict(clear, view=tgt_c.create_view())],
            depth_stencil_attachment={'view': z_t.create_view(), 'depth_clear_value': 1.0,
                                      'depth_load_op': wgpu.LoadOp.clear,
                                      'depth_store_op': wgpu.StoreOp.store})
        rp.set_pipeline(self.pipeline)
        rp.set_bind_group(0, self.group)
        for k, buffer in enumerate(buffers):
            rp.set_vertex_buffer(k, buffer)
        rp.set_index_buffer(indices, wgpu.IndexFormat.uint32)
        if use_fsaa:
            for s in range(4):
                ox, oy = (s & 1) * width, (s >> 1) * height
                rp.set_viewport(ox, oy, width, height, 0.0, 1.0)
                rp.set_scissor_rect(ox, oy, width, height)
                rp.draw_indexed(count, 1, 0, 0, s)
        else:
            rp.draw_indexed(count)
        rp.end()
        if use_fsaa:
            assert res_group is not None
            rp_res = encoder.begin_render_pass(
                color_attachments=[dict(clear, view=depth_t.create_view()),
                                   dict(clear, view=colour_t.create_view())])
            rp_res.set_pipeline(self.res_pipeline)
            rp_res.set_bind_group(0, res_group)
            rp_res.draw(3)
            rp_res.end()
        self.device.queue.submit([encoder.finish()])
        depth = _read(self.device.queue, depth_t, width, height, np.float32)
        colour = _read(self.device.queue, colour_t, width, height, np.uint8)
        return depth, colour.reshape(height, width, 4)
