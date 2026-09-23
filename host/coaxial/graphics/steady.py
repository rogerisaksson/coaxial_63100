"""Three frames vote per cell, so nothing blinks for one frame."""


def _vote(held, now, grid, tone, width, height):
    """The three frames' vote, cell by cell, written into `grid` and
    `tone`: the previous frame's glyph and tone, unless the frames before
    and after it agree against it. A still row is left as it is."""
    (g2, t2), (g1, t1) = held
    g0, t0 = now
    for py in range(height):
        r2, r1, r0 = g2[py], g1[py], g0[py]
        if r2 == r1 == r0:
            continue                          # a still row: as it is
        out_g, out_t = grid[py], tone[py]
        c1, c0 = t1[py], t0[py]
        for px in range(width):
            agreed = r2[px] == r0[px] != r1[px]
            out_g[px] = r0[px] if agreed else r1[px]
            out_t[px] = c0[px] if agreed else c1[px]


def _steady(grid, tone, width, height, persist):
    """THREE FRAMES VOTE, per cell: what shows is the PREVIOUS frame's
    glyph and tone, unless the frames before and after it agree on
    something else - then that. So nothing exists for a single frame
    and nothing vanishes for one: on-off-on and off-on-off both go,
    and so do a face cell's `.`-`:`-`.` and an outline cell's dot
    pattern flipping and flipping back. The price is one frame of
    latency, 50 ms at 20 fps - under an IMU's own tens of milliseconds
    and a hand's turn, and nothing else: a cell that stays changed
    shows changed one frame late and stays; a turning board leaves no
    trail.

    The last of Schroedinger's pixels. With the outline's edges merged
    and its dots on the matrix's lines, what still blinked was the
    hidden-line test at cell resolution (2.7 outline cells a frame in
    a 0.6-degree-a-frame tumble at 150x44), the face's own dither at
    the art's blanks and the shadows' edges (2.1 a frame), and glyphs
    that changed and changed back inside a frame - 16.2 events a frame
    in all, measured over every cell. A one-sided phosphor was built
    first, holding a cell that lost ink for one frame: it took the
    count to 13.8, because it could only fill an off frame, never
    remove an on one, and left every blip a frame longer. The vote
    takes both. At rest under the deadband the frames are identical
    and the vote changes nothing. State is the caller's dict, the two
    frames before this one, keyed by size: a resized window starts
    clean, and the first two frames of a run show as they are."""
    frames = persist.get('steady')
    if frames is None or frames['size'] != (width, height):
        frames = {'size': (width, height), 'held': []}
    held = frames['held']
    now = ([row[:] for row in grid], [row[:] for row in tone])
    if len(held) == 2:
        _vote(held, now, grid, tone, width, height)
    held.append(now)
    del held[:-2]
    persist['steady'] = frames
