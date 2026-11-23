/** wbc_hull.h - the convex hull of the robot's support's floor points (machine.law.barrier.hull) in C
    (wbc_hull.c): Andrew's monotone chain on doubles as Python's floats take them. Portable C11, the host's
    alone. */
#ifndef WBC_HULL_H
#define WBC_HULL_H

/** The most floor points a hull takes. */
#define WBC_HULL_MOST 64

/** barrier.hull of `n` floor points `pts` (n, 2), counterclockwise, into `ring` (n, 2): sorted as Python's
    tuples sort, the lower chain then the upper, each a turn of more than nothing, each chain's last left off. The
    hull's points, or -1 past WBC_HULL_MOST. */
int wbc_hull(const double *pts, int n, double *ring);

#endif
