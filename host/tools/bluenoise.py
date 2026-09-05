"""Generate `coaxial/bluenoise64.bin`, the threshold mask the board's
halftone dithers with.

    python tools/bluenoise.py            # writes host/coaxial/bluenoise64.bin

Void-and-cluster, Ulichney's method, on a 64 x 64 torus: start from a
sparse random pattern, relax it by moving the tightest cluster to the
largest void until that would undo itself, then rank every position -
clusters removed downward from the initial count, voids filled upward
from it - so that thresholding the ranks at ANY level gives an even,
structure-free pattern. The energy is a Gaussian (sigma 1.9) on the
torus, taken through the FFT, which is why this wants numpy and the
renderer, which only reads the file, does not.

WHY A MASK AND NOT A MATRIX. The face was an 8 x 8 Bayer dither, and at
a real window (150 x 44) its hierarchy showed: two-by-two clusters of
dots that read as small square blocks across the board - "blocky as
hell", the bench said, after the Bayer had replaced a sparser lattice
that was blocky in its own way. Interleaved gradient noise and the R2
sequence were rastered beside it: a regular diagonal screen, and a
half-structured one. Blue noise has no structure at any density, and
fixed in screen space it does not crawl when the board turns - the one
thing error diffusion, the other structure-free dither, cannot offer.

Deterministic: seeded, so the file is reproducible from this script.
The renderer reads 4096 little-endian uint16 ranks, row-major.
"""
import os
import struct
import sys

SIZE = 64
SIGMA = 1.9
SEED = 7
START = SIZE * SIZE // 10
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   'coaxial', 'bluenoise64.bin')


def ranks(n=SIZE, sigma=SIGMA, seed=SEED):
    import numpy as np

    yy, xx = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
    dx = np.minimum(xx, n - xx)
    dy = np.minimum(yy, n - yy)
    kernel_f = np.fft.fft2(np.exp(-(dx ** 2 + dy ** 2) / (2.0 * sigma ** 2)))

    def energy(mask):
        return np.real(np.fft.ifft2(np.fft.fft2(mask.astype(float)) * kernel_f))

    rng = np.random.default_rng(seed)
    mask = np.zeros((n, n), dtype=bool)
    mask.flat[rng.choice(n * n, START, replace=False)] = True
    for _ in range(4000):
        cluster = np.argmax(np.where(mask, energy(mask), -np.inf))
        mask.flat[cluster] = False
        void = np.argmin(np.where(mask, np.inf, energy(mask)))
        if void == cluster:
            mask.flat[cluster] = True
            break
        mask.flat[void] = True

    rank = np.zeros((n, n), dtype=int)
    work = mask.copy()
    for r in range(int(work.sum()) - 1, -1, -1):
        cluster = np.argmax(np.where(work, energy(work), -np.inf))
        work.flat[cluster] = False
        rank.flat[cluster] = r
    work = mask.copy()
    for r in range(int(mask.sum()), n * n):
        void = np.argmin(np.where(work, np.inf, energy(work)))
        work.flat[void] = True
        rank.flat[void] = r
    return rank


def main():
    rank = ranks()
    flat = [int(v) for v in rank.flatten()]
    assert sorted(flat) == list(range(SIZE * SIZE))
    with open(OUT, 'wb') as f:
        f.write(struct.pack('<%dH' % len(flat), *flat))
    print('wrote %s: %d x %d ranks' % (OUT, SIZE, SIZE))
    return 0


if __name__ == '__main__':
    sys.exit(main())
