"""Three frames vote per cell, so nothing blinks for one frame."""


def _vote(held, now, grid, tone, width, height):
    """The three frames' vote, cell by cell, written into `grid` and `tone`:
    the previous frame's glyph and tone, unless the frames before and
    after it agree against it.
    """
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
    """THREE FRAMES VOTE, per cell: what shows is the PREVIOUS frame's glyph
    and tone, unless the frames before and after it agree on something
    else - then that.
    """
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
