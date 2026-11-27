"""4x RGSS (4-rooks) sub-dot FSAA lit triangle mesh rasterizer on GPU (`LitRaster`) or SIMD C11."""
import math

from coaxial.graphics.lit import LAYER_M, UNDER, _const, _light
from coaxial.graphics.raster import SUBDOT


def raster_mesh_py(positions, normals, uv, materials, index, m, cam, centre, fsaa=True):
    """Pure-NumPy 4-rooks FSAA indexed triangle rasterizer matching `rs_lit_mesh_fsaa`."""
    np = _const()[0]
    height, width = int(cam['height']), int(cam['width'])
    pos = np.asarray(positions, float)
    nrm = np.asarray(normals, float)
    uvs = np.asarray(uv, float)
    mats = np.asarray(materials, np.uint32)
    tris = np.asarray(index, np.int32).reshape(-1, 3)
    m3 = np.asarray(m, float).reshape(3, 3)
    q = (pos - np.asarray(centre, float)) @ m3.T
    vn = nrm @ m3.T
    dz = float(cam['distance']) - q[:, 2]
    ok = dz > 1e-6
    w = np.where(ok, 1.0 / np.where(ok, dz, 1.0), 0.0)
    uw = np.where(ok, 1.0 / np.where(ok, dz + np.where((mats & UNDER) > 0, LAYER_M, 0.0), 1.0), 0.0)
    sx = float(cam['cx']) + float(cam['scale']) * w * q[:, 0]
    sy = float(cam['cy']) - float(cam['scale']) * float(cam.get('aspect', 0.5)) * w * q[:, 1]
    rooks = SUBDOT if fsaa else ((0.0, 0.0),)
    ns = len(rooks)
    zbuf = np.zeros((ns, height, width), np.float32)
    obuf = np.zeros((ns, height, width), np.float32)
    cbuf = np.zeros((ns, height, width, 4), np.uint8)
    pad = 0.375 if fsaa else 0.0
    for ia, ib, ic in tris.tolist():
        if not (ok[ia] and ok[ib] and ok[ic]):
            continue
        x0, y0, x1, y1, x2, y2 = sx[ia], sy[ia], sx[ib], sy[ib], sx[ic], sy[ic]
        area = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        if area == 0.0:
            continue
        jb, jc = (ic, ib) if area < 0.0 else (ib, ic)
        if area < 0.0:
            x1, y1, x2, y2, area = x2, y2, x1, y1, -area
        lox = max(0, math.ceil(min(x0, x1, x2) - pad))
        hix = min(width - 1, math.floor(max(x0, x1, x2) + pad))
        loy = max(0, math.ceil(min(y0, y1, y2) - pad))
        hiy = min(height - 1, math.floor(max(y0, y1, y2) + pad))
        if hix < lox or hiy < loy:
            continue
        yy, xx = np.mgrid[loy:hiy + 1, lox:hix + 1]
        inv = 1.0 / area
        for s, (ox, oy) in enumerate(rooks):
            qx, qy = xx + ox, yy + oy
            w0 = ((x2 - x1) * (qy - y1) - (y2 - y1) * (qx - x1)) * inv
            w1 = ((x0 - x2) * (qy - y2) - (y0 - y2) * (qx - x2)) * inv
            w2 = 1.0 - w0 - w1
            hit = (w0 >= 0.0) & (w1 >= 0.0) & (w2 >= 0.0)
            if not hit.any():
                continue
            zt = (w0 * uw[ia] + w1 * uw[jb] + w2 * uw[jc]).astype(np.float32)
            win = hit & (zt > zbuf[s, loy:hiy + 1, lox:hix + 1])
            if not win.any():
                continue
            wy, wx = np.nonzero(win)
            bw0, bw1, bw2 = w0[wy, wx], w1[wy, wx], w2[wy, wx]
            n_i = bw0[:, None] * vn[ia] + bw1[:, None] * vn[jb] + bw2[:, None] * vn[jc]
            uv_i = bw0[:, None] * uvs[ia] + bw1[:, None] * uvs[jb] + bw2[:, None] * uvs[jc]
            zbuf[s, loy + wy, lox + wx] = zt[wy, wx]
            obuf[s, loy + wy, lox + wx] = (bw0 * w[ia] + bw1 * w[jb] + bw2 * w[jc]).astype(np.float32)
            cbuf[s, loy + wy, lox + wx] = _light(n_i, np.full(len(wy), mats[ia], np.uint32), uv_i)
    cov_s = obuf > 0.0
    any_h = cov_s.sum(axis=0)
    order = np.argsort(obuf, axis=0)
    best_s, sec_s = order[-1], order[-2] if ns > 1 else order[-1]
    best_o = np.take_along_axis(obuf, best_s[None], 0)[0]
    fg_h = (cov_s & (obuf >= best_o[None] * np.float32(0.96))).sum(axis=0)
    rep_s = np.where((ns == 4) & (fg_h == 1) & (any_h >= 2), sec_s, best_s)
    out_d = np.where(any_h > 0, np.take_along_axis(obuf, rep_s[None], 0)[0], 0.0).astype(np.float64)
    out_c = np.zeros((height, width, 4), np.uint8)
    safe_h = np.maximum(any_h, 1)
    rgb_s = (cbuf[..., :3].astype(int) * cov_s[..., None]).sum(axis=0)
    out_c[..., :3] = np.where((any_h > 0)[..., None],
                              (rgb_s * 2 + safe_h[..., None]) // (2 * safe_h[..., None]), 0)
    h_rep = np.take_along_axis(cbuf[..., 3], rep_s[None], 0)[0].astype(int)
    miss = (4 - any_h) if ns == 4 else np.zeros_like(any_h)
    out_c[..., 3] = np.where(any_h > 0, np.where(h_rep >= 248, 255 - miss, h_rep + miss), 0)
    return out_d, out_c


def raster_mesh(positions, normals, uv, materials, index, m, cam, centre, reach=1.0, card=None,
                fsaa=True):
    """`(depth, colour)` for an indexed lit mesh on `card` (`LitRaster`) or SIMD C11 4x RGSS FSAA."""
    if card is not None:
        return (card.raster(positions, normals, uv, materials, index, m, cam, centre, reach)
                if fsaa else card.raster(positions, normals, uv, materials, index, m, cam, centre,
                                         reach, fsaa=False))
    from tools.cores import raster_draw
    got = raster_draw.lit_mesh_fsaa(positions, normals, uv, materials, index, m, cam, centre,
                                    UNDER, LAYER_M, fsaa)
    if got is not None:
        return got
    return raster_mesh_py(positions, normals, uv, materials, index, m, cam, centre, fsaa)
