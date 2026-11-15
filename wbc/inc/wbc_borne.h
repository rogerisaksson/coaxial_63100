/** wbc_borne.h - what bears the robot off its soles as its skin feels it: machine.senses.borne.read in C (wbc_borne.c).
    The robot's support and its step's contacts laid on its law's model once: its constants, the
    robot's model's posed arrays, MuJoCo's own Jacobians by their pointers and numpy's OpenBLAS (wbc_blas.h).
    Portable C11, the host's alone. */
#ifndef WBC_BORNE_H
#define WBC_BORNE_H

#include "wbc_blas.h"

/** The robot's skin's segments (`edge.SKIN`), a segment's capsules (`borne.CAPSULES`), a read's contacts and its
    law's or its model's degrees of freedom: the most laid. */
#define WBC_BORNE_SKIN 24
#define WBC_BORNE_CAPS 8
#define WBC_BORNE_CONTACTS 32
#define WBC_BORNE_DOF 64

/** MuJoCo's mj_jac and mj_jacDot (mujoco.h): the robot's model and data, the point's translational and
    rotational rows, the point in the world and its body. */
typedef void (*wbc_borne_jac_t)(const void *m, const void *d, double *jacp, double *jacr, const double *point,
                                int body);

/** machine.senses.borne.read laid on the robot's law's model, and a read's support and contacts
 * beside it. */
typedef struct
{
  int                    skin, above, n, nv;        /* the robot's skin's segments, those above its soles; its dofs, its model's */
  int                    touch_at, at_at, give_at, nrm_at;  /* the robot's edge's array: TOUCH's, AT's, GIVE's and NORMAL's first */
  int                    pelvis, carpal, let, role_spare; /* borne.PELVIS, CARPAL, LET; explicit: its ints a whole number of doubles */
  int                    sole[2];                   /* each sole's segment (`touch.SOLES`) */
  int                    ids[WBC_BORNE_SKIN];       /* each segment's body in the robot's model */
  int                    order[WBC_BORNE_SKIN];     /* borne.ABOVE, in its order */
  int                    hand[WBC_BORNE_SKIN];      /* 1 of borne.HANDS */
  int                    right[WBC_BORNE_SKIN];     /* 1 of the robot's right arm's (HANDS past touch.ARMS[0]) */
  int                    caps[WBC_BORNE_SKIN];      /* each one's capsules */
  double                 cap_r[WBC_BORNE_SKIN][WBC_BORNE_CAPS];
  double                 cap_a[WBC_BORNE_SKIN][WBC_BORNE_CAPS][3], cap_b[WBC_BORNE_SKIN][WBC_BORNE_CAPS][3];
  double                 bears_n, seat_x;          /* BEARS_N, figure.SEAT_X */
  const double          *xmat, *xpos;               /* the robot's model's data as its sense posed it */
  const void            *m, *d;                     /* the robot's model and its data, MuJoCo's */
  wbc_borne_jac_t        jac, jac_dot;              /* mj_jac, mj_jacDot */
  const wbc_blas_t      *blas;                      /* numpy's OpenBLAS */
  int                    nc;                        /* a read's contacts */
  int                    spare;                     /* explicit: its ints a whole number of doubles */
  double                 up, low[3], high[3], palms[2];
  double                 p[WBC_BORNE_CONTACTS][3], f[WBC_BORNE_CONTACTS][3], dJv[WBC_BORNE_CONTACTS][3];
  double                 J[WBC_BORNE_CONTACTS][3][WBC_BORNE_DOF];
  double                 carpal_of[WBC_BORNE_CONTACTS], room[WBC_BORNE_CONTACTS], give[WBC_BORNE_CONTACTS];
  int                    side[WBC_BORNE_CONTACTS], patch[WBC_BORNE_CONTACTS], body[WBC_BORNE_CONTACTS];
  double                 jp[3 * WBC_BORNE_DOF], jr[3 * WBC_BORNE_DOF];
  int                    seat_skin, seat_body, pelvis_body, seat_spare;  /* the seat's segment (emancipation.SEAT_SEG), its body;
                                                               the robot's pelvis's body */
  double                 seat_dx[2], seat_dz[2];    /* its CoP box's corners along the robot's way and across (SEAT_BOX) */
  double                 touch_m, reach_m, shed0;   /* leap.TOUCH_M, observer.REACH_M, cascade.SHED[0] */
  int                    corner[WBC_BORNE_CONTACTS];  /* a contact's corner of the seat's cone, -1 none */
  double                 seat_on, seat_near;        /* emancipation.seated's g['seat_on'], g['seat_near'] */
  double                 nrm[WBC_BORNE_CONTACTS][3]; /* a contact's normal of what bears it, world (borne._normal) */
} wbc_borne_t;

/** A read (machine.senses.borne.read) of the robot's edge's array `y` at its law's velocity `v`, its hands
    told to bear (`hands`) and its legs let go (`let_go`, 2): its support's lift `up` and box
    (`low`, `high`, infinite where nothing bears it), its palms and its contacts. 0, or -1 past WBC_BORNE_CONTACTS. */
int wbc_borne_read(wbc_borne_t *b, const double *y, const double *v, double hands, const double *let_go);

/** machine.moves.emancipation.seated on a read's contacts: the seat the robot's text tells bears it (`told`
    its row's share, the observer's `points` of its `row` among `rows`, `npoints`) lying within reach of the
    `floor` - `seat_on` and `seat_near` -, and while the robot sits (`sat` past a half) its contacts out and its CoP
    box's four corners in, each bearing what the robot's step put on it the pass before (`seat_f`) or a quarter of
    what the robot's skin felt of it, its Jacobian and drift at the robot's law's velocity `v`; `emancipated` the
    robot's legs' release, the larger. 0, or -1 past WBC_BORNE_CONTACTS. */
int wbc_borne_seated(wbc_borne_t *b, const double *points, const int64_t *rows, int npoints, int row, double told,
                     double floor, double emancipated, double sat, const double *seat_f, const double *v);

/** machine.moves.emancipation.supports on a read's contacts: what bears the robot besides its soles - each
    contact but a let-go foot's (`patch` under `let`), its point's x and z and its capsule's room; the robot's
    observer's `points` (np, 3) that `said` keeps, their `rooms`; each sole's corners (`corners0`, `corners1`: nc0,
    nc1 of 3), room 0 - into `pts` (n, 2) and `room` (n), and each leg's own in `soles` (2, n): its sole's corners 1,
    the other sole's 1 less whether that leg stands (`loads` past `touch_n`, its `legs` release under 1). The
    supports' n, or -1 past `cap`. */
int wbc_borne_supports(const wbc_borne_t *b, int np_, const unsigned char *said, const double *points,
                       const double *rooms, const double *corners0, int nc0, const double *corners1, int nc1,
                       const double *loads, const double *legs, double touch_n, int cap, double *pts, double *room,
                       double *soles);

#endif
