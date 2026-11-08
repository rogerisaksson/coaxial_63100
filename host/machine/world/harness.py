"""Her harness: her pelvis hung where it was by springs on its place and its turn, and its load cell.

    w = harness.wrench(d, np, hung, pelvis)    # the harness's wrench on her pelvis, world (6,)
    f = harness.pull(d, np, hung, pelvis)      # its load cell: force N and moment N m about her pelvis

`hung` (place (3,), turn (4,) w, x, y, z) where `World.hang` caught her; `pelvis` her pelvis's body
in `d`, MuJoCo's data; `np` the world's numpy.
"""

#: Her harness (`World.hang`): N/m and N s/m on her pelvis's place, N m/rad and N m s/rad on its
#: turn - 0.7 of critical on her 31 kg and 3 kg m^2.
HANG = (4000.0, 500.0, 400.0, 50.0)


def wrench(d, np, hung, pelvis):
    """The harness's wrench on her pelvis, world: twice the vector part of turn * conj(q) is the
    small turn from q to turn."""
    at, turn = hung
    q, w = d.qpos[3:7], d.xmat[pelvis].reshape(3, 3) @ d.qvel[3:6]
    off = 2.0 * (q[0] * turn[1:] - turn[0] * q[1:] - np.cross(turn[1:], q[1:]))
    return np.r_[HANG[0] * (at - d.qpos[0:3]) - HANG[1] * d.qvel[0:3], HANG[2] * off - HANG[3] * w]


def pull(d, np, hung, pelvis):
    """What a load cell in the harness reads: its force on her pelvis, world, N, and its moment
    about her pelvis's origin, world, N m."""
    w = wrench(d, np, hung, pelvis)
    return np.r_[w[:3], w[3:] + np.cross(d.xipos[pelvis] - d.xpos[pelvis], w[:3])]
