"""Her looks' objects: a segmenter's masks on the card, each boxed from her stereo's points under it.

    seg = instances.segmenter()                     # None unless SEGMENT, a card and torch: her map's things alone
    masks = seg(left)                               # (n, H, W) bool, each object her picture shows
    objects = instances.boxes(masks, look, chart)   # [(kind, centre, half, R)] each mask's, `sight.Look`'s points
    things = instances.merged(things, objects, chart)   # her map's things, each object in place of those it is

The segmenter is a Mask R-CNN (torchvision's ResNet-50 FPN v2 on COCO, BSD): its masks, not its classes - this
room's chairs and its table were COCO's umbrellas, its stool a chair, its bed alone a bed -; what a thing is her
classifier tells from its box (`classes`). Its backbone a TensorRT engine at half precision, built once a card and
kept beside torch's weights (2.2 ms against torch's 10.0 at 480 x 640, its five maps 1-2 % off), its heads torch's;
without TensorRT torch's own. A mask is one the model holds SCORE likely or more, the same object's lesser masks
under any class dropped by the model's own overlap (`nms_thresh`), a pixel its where the mask's is over half, eroded
a pixel round: on an object's rim her stereo's window mixes it with what is behind. Its box is its points' PERCENT
along the text's axes of their footprint (`voxels.fitted`) and up: on the renderer's depth a seat's top 0.40 and
0.48 m for 0.397 and 0.47, her map's 0.48 and 0.58.
"""
import collections
import importlib
import os
import sys

from machine.eyes import voxels
from machine.eyes.stereo import ENV, H, W, _np

#: The switch: 1 boxes her looks' objects (`segmenter`). Off, her map's things alone: on her stereo's points
#: (320 x 240, SAD) a low chair's mask took its seat and back as one box 0.87 m high, a table's, its stool's
#: 0.8 m long with the floor behind it, a bench's - her map told it a stool -, the chair none; the table alone a table.
SEGMENT = 'COAXIAL_SEGMENT'
#: A mask kept where the segmenter holds it more likely than not.
SCORE = 0.5
#: Her picture at the segmenter's least size (480 rows): the backbone's one input, px (rows, columns).
IN = (max(H, 480), max(H, 480) * W // H)
#: A box: its points' percentiles along each axis, the outlying 2 % each end cut.
PERCENT = (2.0, 98.0)
#: The backbone's maps as the model names them.
MAPS = ('0', '1', '2', '3', 'pool')

_SEG = {}


def segmenter():
    """Her segmenter (`Segmenter`) on the card, laid at its first ask; None unless SEGMENT is 1, where `COAXIAL_GPU`
    is 0, no card answers or torch is not there - her map's things alone then."""
    if 'seg' not in _SEG:
        _SEG['seg'] = None
        if os.environ.get(SEGMENT, '0') == '1' and os.environ.get(ENV, '1') != '0':
            try:
                torch = importlib.import_module('torch')
                if torch.cuda.is_available():
                    _SEG['seg'] = Segmenter()
            except ImportError:
                pass
            except Exception as e:      # its weights not to be had, a card that refuses it: her map's things alone
                print('her segmenter: none (%s: %s), her map alone' % (type(e).__name__, e), file=sys.stderr)
    return _SEG['seg']


class Segmenter:

    """The Mask R-CNN on the card, its backbone a TensorRT engine where one builds."""

    def __init__(self):
        self.torch = importlib.import_module('torch')
        detection = importlib.import_module('torchvision.models.detection')
        self.nms = importlib.import_module('torchvision.ops').nms
        self.model = detection.maskrcnn_resnet50_fpn_v2(
            weights=detection.MaskRCNN_ResNet50_FPN_V2_Weights.COCO_V1, min_size=IN[0], max_size=IN[1],
            box_score_thresh=SCORE).eval().cuda()
        blob = _engine(self.torch, self.model.backbone)
        if blob is not None:
            self.model.backbone = _backbone(self.torch, blob)

    def __call__(self, left):
        """(n, H, W) bool: each object's pixels in `left` (H, W, 3 uint8), eroded a pixel round."""
        torch, np = self.torch, _np()
        with torch.inference_mode():
            image = torch.from_numpy(np.ascontiguousarray(left)).cuda().permute(2, 0, 1).float().div_(255.0)
            out = self.model([image])[0]
            one = self.nms(out['boxes'], out['scores'], self.model.roi_heads.nms_thresh)
            masks = out['masks'][one, 0] > 0.5
            if not len(masks):
                return np.zeros((0, H, W), bool)
            rim = torch.nn.functional.max_pool2d((~masks).float()[:, None], 3, stride=1, padding=1)[:, 0] > 0.0
            return (masks & ~rim).cpu().numpy()


def _engine(torch, backbone):
    """The backbone's TensorRT engine at half precision, serialized: built once a card and TensorRT from its ONNX graph
    and kept in torch's cache beside its weights; None without TensorRT or where it does not build."""
    try:
        trt = importlib.import_module('tensorrt')
    except ImportError:
        return None
    import copy
    import tempfile
    path = os.path.join(torch.hub.get_dir(), 'engines', 'maskrcnn_backbone-%s-%s-%dx%d.engine' % (
        trt.__version__, torch.cuda.get_device_name(0).replace(' ', '_'), *IN))
    if os.path.exists(path):
        with open(path, 'rb') as f:
            return f.read()
    log = trt.Logger(trt.Logger.ERROR)
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        graph = os.path.join(tmp, 'backbone.onnx')
        try:
            torch.onnx.export(copy.deepcopy(backbone).half(), (torch.zeros((1, 3) + IN, dtype=torch.float16, device='cuda'),),
                              graph, input_names=['image'], output_names=list(MAPS), dynamo=True)
            builder = trt.Builder(log)
            network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.STRONGLY_TYPED))
            parser = trt.OnnxParser(network, log)
            blob = builder.build_serialized_network(network, builder.create_builder_config()) if parser.parse_from_file(
                graph) else None
            del parser, network, builder
        except Exception as e:      # an export or a build the engine's versions refuse: torch's backbone instead
            print('her segmenter: no TensorRT engine (%s: %s), torch\'s backbone' % (type(e).__name__, e), file=sys.stderr)
            return None
    if blob is None:
        return None
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(bytes(blob))
    return bytes(blob)


def _backbone(torch, blob):
    """The engine `blob` as the model calls its backbone: (1, 3, IN) in, its maps out, as floats, by name."""
    trt = importlib.import_module('tensorrt')

    class Backbone(torch.nn.Module):
        out_channels = 256

        def __init__(self):
            super().__init__()
            self.engine = trt.Runtime(trt.Logger(trt.Logger.ERROR)).deserialize_cuda_engine(blob)
            self.context = self.engine.create_execution_context()
            self.maps = {n: torch.empty(tuple(self.engine.get_tensor_shape(n)), dtype=torch.float16, device='cuda')
                         for n in MAPS}
            for n, t in self.maps.items():
                self.context.set_tensor_address(n, t.data_ptr())

        def forward(self, x):
            image = x.half().contiguous()
            self.context.set_tensor_address('image', image.data_ptr())
            self.context.execute_async_v3(torch.cuda.current_stream().cuda_stream)
            return collections.OrderedDict((n, self.maps[n].float()) for n in MAPS)
    return Backbone()


def boxes(masks, look, chart):
    """[(kind, centre, half, R)] of each mask's object among a look's points (`sight.Look`: its pixels, places and
    colours) her map would take (`voxels.Map.add`'s, NEAR_M to FAR_M off her eye, FLOOR_M to CEIL_M up): its kind the
    palette's whose lit line its pooled colour lies nearest (`voxels.lit`), as her map tells a thing's; its box `box`'s;
    a mask with fewer than HITS of them none."""
    np = _np()
    xyz, away = look.xyz, np.linalg.norm(look.xyz - look.seen.at, axis=1)
    ok = (xyz[:, 1] > voxels.FLOOR_M) & (xyz[:, 1] < voxels.CEIL_M) & (away > voxels.NEAR_M) & (away < voxels.FAR_M)
    out = []
    for mask in masks:
        on = ok & mask[look.rows, look.cols]
        if on.sum() >= voxels.HITS:
            kind = int(np.argmin(voxels.lit(voxels.chroma(look.rgb[on].sum(axis=0)), chart.means)))
            out.append((chart.names[kind],) + box(xyz[on]))
    return out


def box(p):
    """(centre, half sizes, turn 3x3) of points `p` (n, 3), upright as her map's: PERCENT of them within it along the
    text's axes of their footprint (`voxels.fitted`) and up."""
    np = _np()
    m = p[:, ::2].mean(axis=0)
    xz = p[:, ::2] - m
    a, b = voxels.fitted(xz, 0.0)
    if a[0] * b[1] - a[1] * b[0] < 0.0:
        b = -b
    (a0, a1), (b0, b1), (y0, y1) = (np.percentile(q, PERCENT) for q in (xz @ a, xz @ b, p[:, 1]))
    c = m + 0.5 * (a0 + a1) * a + 0.5 * (b0 + b1) * b
    return ((float(c[0]), 0.5 * float(y0 + y1), float(c[1])), (0.5 * float(a1 - a0), 0.5 * float(y1 - y0), 0.5 * float(b1 - b0)),
            ((float(a[0]), 0.0, float(b[0])), (0.0, 1.0, 0.0), (float(a[1]), 0.0, float(b[1]))))


def _top(box_):
    """A box's top, m: its centre and its turned half sizes' height."""
    np = _np()
    return float(box_[1][1] + (np.abs(np.asarray(box_[3], float)) @ np.asarray(box_[2], float))[1])


def _within(box_, x, z, slack):
    """Whether points (x, z) - arrays broadcast - lie in a box's footprint, `slack` m round."""
    np = _np()
    (cx, _cy, cz), half, R = box_[1:4]
    dx, dz = x - cx, z - cz
    return (np.abs(dx * R[0][0] + dz * R[2][0]) <= half[0] + slack) & (np.abs(dx * R[0][2] + dz * R[2][2]) <= half[2] + slack)


def merged(things, objects, chart):
    """[(kind, centre, half, R, id, clear)] her map's `things` (`voxels.Map.things`) with each of `objects` (`boxes`)
    in place of those of them it is - a thing whose footprint's middle lies in the object's, a cell round, its top
    within a cell of the object's -: the object the id of the nearest it replaces, once, else none (0); clear where
    her map saw through a voxel of the layer over its top within its footprint, as her map's surfaces are."""
    np = _np()
    out, used = list(things), set()
    for ob in objects:
        top, c = _top(ob), ob[1]
        same = [k for k, t in enumerate(out)
                if _within(ob, t[1][0], t[1][2], voxels.CELL_M) and abs(_top(t) - top) <= voxels.CELL_M]
        nearest = sorted(same, key=lambda k: np.hypot(out[k][1][0] - c[0], out[k][1][2] - c[2]))
        ident = next((out[k][4] for k in nearest if out[k][4] not in used), 0)
        used.add(ident)
        out = [t for k, t in enumerate(out) if k not in same] + [tuple(ob) + (ident, _clear(ob, top, chart))]
    return out


def _clear(ob, top, chart):
    """1.0 where her map saw through a voxel of the layer over `top` within object `ob`'s footprint, 0.0 else."""
    np = _np()
    j = int(np.floor((top - voxels.FLOOR_M) / chart.voxel)) + 1
    if not 0 <= j < chart.ny:
        return 0.0
    x = voxels.X_M[0] + (np.arange(chart.nx) + 0.5) * chart.voxel
    z = voxels.Z_M[0] + (np.arange(chart.nz) + 0.5) * chart.voxel
    i, k = np.nonzero(_within(ob, x[:, None], z[None, :], 0.5 * chart.voxel))
    return float((chart.free[i, j, k] > 0).any())
