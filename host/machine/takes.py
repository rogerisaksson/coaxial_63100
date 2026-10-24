"""The mocap takes as rows for her loop: a take's modes at its beats, laid once, said as rows.

    wire, end_ms = takes.wire('stand_to_sit')  # the take's rows (ortho.ROW), its voices at its beats
    row = takes.pattern('jogging_forward')     # a gait take as its amounts on her standard gait (machine.gait)

FILE holds each take's times (s, at her size) and her first eleven modes at them, read off the take as
`modal.reading` reads hers (`tools/sim/stunts.py --reference`; the takes lie beside the repo).
A take is said at its beats (`beats`): the frames where one of its voices turns - its rate, read
over one natural time (TICK_S), changes sign -, its first and its last; her loop draws its own
curve through them (`latent.CONTIG`, the rows contiguous). Said at every frame, 31 ms apart,
the frames' noise bent her 118 sagittal/s^2 and 56 pitch/s^2 (stunts.md). STREAMED are the
voices a take says, her posture's: the legs' shortening, the centre of mass over the soles
along, the trunk's pitch; her stance's and her arms' stay her balance's (the human's feet,
pelvis list, heels and arm swing each felled her when said), and VOICES names a take's own. A
take enters over ENTER_S from wherever she is, its first row's blend. A gait take is its amounts
on her standard gait (`amounts`, `pattern`: `gait.modified`): its stride's Hz and what its voices
sing at it, less the walk's own, laid over the gait she is on. The
bench tells a take (`scenarios._told`); her texts stay the lisp's: told the bed's takes she sat
at its edge as the human did and collapsed back from there (stunts.md).
"""
import math
import os

from machine import latent, modal, ortho

#: Each take's '<key>_t' and '<key>_z' (frames, modes), at her size.
FILE = os.path.join(os.path.dirname(__file__), 'takes.npz')
#: The modes a take says.
STREAMED = (modal.AXIAL, modal.SAGITTAL, modal.PITCH)
#: A take's own voices, where STREAMED's are not all hers to say: a squat's hips go past his
#: toes on heels that rise, a dance's where its steps are, and told them her parry stepped
#: her round the squat till she fell, over at 4.1 s in the dance (sequencer.md).
VOICES = {key: (modal.AXIAL, modal.PITCH)
          for key in ('crouching_down', 'crouch_up', 'kneeling_down', 'kneeling_up', 'hip_hop')}
#: A take's rows' sequence id, after the texts' streams (200-205).
TAKE_ID = 300
#: A take's first row's blend, s: the half period of her leg's swing, the window its measures
#: are smoothed over too.
ENTER_S = math.pi * math.sqrt(modal.LEG_M / modal.G)
#: A beat's resolution, s: her natural time, the window a take's measures are read over.
TICK_S = math.sqrt(modal.LEG_M / modal.G)
#: A pattern's modifiers (`gait.MODIFIERS`): its pace, these sung at the stride's Hz (their
#: amplitudes) and these held (their means). A climb's is not hers yet (sequencer.md): its crouch
#: said over the floor's fold she squatted twice as deep on the second tread, and its pace, stride
#: and lean alone hopped her onto the first tread and toppled her backward, where the walk climbs.
SUNG, HELD = {'stride': modal.DRIVE, 'swing': modal.ARMS}, {'crouch': modal.AXIAL, 'lean': modal.PITCH}


def frames(key, since_s=0.0, until_s=math.inf):
    """(s, modes (frames, N)) of a take's frames from `since_s` to `until_s`, at her size."""
    import numpy as np
    with np.load(FILE) as kept:
        t, z = kept[key + '_t'], kept[key + '_z'].astype(float)
    kept = (t >= since_s - 1e-9) & (t <= until_s + 1e-9)
    return t[kept], z[kept]


def jump(key):
    """(its crouch's s, its take-off's s, its modes at the crouch, its modes at the take-off, its
    legs' lengthening at take-off, axial/s) of a jump's take: the crouch where its legs are
    shortest, the take-off where they are longest again within ENTER_S after."""
    import numpy as np
    t, z = frames(key)
    crouch = int(np.argmax(z[:, modal.AXIAL]))
    after = (t > t[crouch]) & (t <= t[crouch] + ENTER_S)
    off = int(np.argmin(np.where(after, z[:, modal.AXIAL], np.inf)))
    rate = (z[off - 1, modal.AXIAL] - z[off, modal.AXIAL]) / (t[off] - t[off - 1])
    return float(t[crouch]), float(t[off]), z[crouch], z[off], float(rate)


def _mean(a, n):
    """`a`'s running mean over `n` samples along its first axis, the ends held."""
    import numpy as np
    pad = np.pad(a, [(n // 2, n - 1 - n // 2), (0, 0)], mode='edge')
    return np.stack([np.convolve(pad[:, k], np.ones(n) / n, 'valid') for k in range(a.shape[1])], 1)


def beats(key, voices, since_s=0.0):
    """(s, modes (beats, N)) of a take's beats from `since_s`: its frames where one of its
    `voices` turns or changes pace - its rate or its acceleration, each read over one natural
    time (TICK_S), changes sign -, its first and its last; every second or third where more
    than her table holds (`latent.ROWS`). By its turns alone a rise from a crouch was one
    straight note, 10 to 90 % of the way in 5.95 natural times for the take's 2.33."""
    import numpy as np
    t, z = frames(key, since_s)
    n = max(1, int(round(TICK_S / (t[1] - t[0])))) | 1
    rate = np.gradient(_mean(z[:, list(voices)], n), t, axis=0)
    acc = np.gradient(_mean(rate, n), t, axis=0)
    on = np.zeros(len(t), bool)
    on[0] = on[-1] = True
    on[1:] |= np.any((np.sign(rate[1:]) * np.sign(rate[:-1]) < 0) | (np.sign(acc[1:]) * np.sign(acc[:-1]) < 0), axis=1)
    thin = -(-int(on.sum()) // latent.ROWS)
    return t[on][::thin], z[on][::thin]


def amounts(key):
    """{modifier: amount} of a gait take on her standard gait (`gait.MODIFIERS`): its stride's Hz
    - the drive voice's strongest line -, SUNG's amplitudes and HELD's means at that Hz by least
    squares, each less the walk's own (`gait.standard`). The lift's amplitude is the swing's
    height in the take, not her clearance's; her stride renders it."""
    import numpy as np
    from machine import gait
    t, z = frames(key)
    x = z[:, modal.DRIVE] - z[:, modal.DRIVE].mean()
    n = 16 * len(x)
    f = np.fft.rfftfreq(n, t[1] - t[0])
    line = np.abs(np.fft.rfft(x * np.hanning(len(x)), n)) * (f >= 1.0 / t[-1])
    hz = float(f[int(np.argmax(line))])
    w = math.tau * hz * t
    c, *_ = np.linalg.lstsq(np.stack([np.ones_like(t), np.sin(w), np.cos(w)], 1), z, rcond=None)
    std = gait.standard()
    got = {'pace': hz, **{name: float(np.hypot(c[1, i], c[2, i])) for name, i in SUNG.items()},
           **{name: float(c[0, i]) for name, i in HELD.items()}}
    return {name: value - float(std[gait.MODIFIERS[name]]) for name, value in got.items()}


def pattern(key, blend_s=ENTER_S):
    """A gait take as one row (`ortho.ROW`) over the gait she is on: her standard gait moved by
    the take's amounts (`gait.modified`), the rest left the gait's, blended over `blend_s`."""
    from machine import gait
    return gait.modified(blend_s, **amounts(key))


def wire(key, at_ms=0, since_s=0.0, enter_s=ENTER_S):
    """A take's rows for her loop (`ortho.ROW`) from `at_ms` on, its beats from `since_s` of it:
    its first row blended over `enter_s`, each after laid as the beat before arrives and blended
    to its own over their gap, so her loop draws one curve through them (`ortho.CURVE`); its
    voices said (VOICES, else STREAMED), the rest left as they are. And the ms its last beat
    arrives at, where a next take's rows join."""
    voices = VOICES.get(key, STREAMED)
    t, z = beats(key, voices, since_s)
    gaps = [int(round(1e3 * enter_s))] + [int(round(1e3 * (t[k] - t[k - 1]))) for k in range(1, len(t))]
    rows, at = [], at_ms
    for k in range(len(t)):
        p = [math.nan] * ortho.P
        for i in voices:
            p[ortho.Z + i] = float(z[k, i])
        rows.append(ortho.ROW.pack(TAKE_ID, at, gaps[k], ortho.CURVE, *p))
        at += gaps[k]
    return b''.join(rows), at
