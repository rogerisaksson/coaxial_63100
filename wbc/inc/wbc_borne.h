/** wbc_borne.h - what bears her off her soles as her skin feels it: machine.senses.borne.read in C (wbc_borne.c).
    Her support and her step's contacts laid on her law's model once: its constants, her model's posed arrays,
    MuJoCo's own Jacobians by their pointers and numpy's OpenBLAS (wbc_blas.h). Portable C11, the host's alone. */
#ifndef WBC_BORNE_H
#define WBC_BORNE_H

#include "wbc_blas.h"

/** Her skin's segments (`edge.SKIN`), a segment's capsules (`borne.CAPSULES`), a read's contacts and her law's or
    her model's degrees of freedom: the most laid. */
#define WBC_BORNE_SKIN 24
#define WBC_BORNE_CAPS 8
#define WBC_BORNE_CONTACTS 32
#define WBC_BORNE_DOF 64

/** MuJoCo's mj_jac and mj_jacDot (mujoco.h): her model and data, the point's translational and rotational rows,
    the point in the world and its body. */
typedef void (*wbc_borne_jac_t)(const void *m, const void *d, double *jacp, double *jacr, const double *point,
                                int body);

/** machine.senses.borne.read laid on her law's model, and a read's support and contacts beside it. */
typedef struct
{
  int                    skin, above, n, nv;        /* her skin's segments, those above her soles; her dofs, her model's */
  int                    touch_at, at_at, give_at, nrm_at;  /* her edge's array: TOUCH's, AT's, GIVE's and NORMAL's first */
  int                    pelvis, carpal, let, role_spare; /* borne.PELVIS, CARPAL, LET; explicit: its ints a whole number of doubles */
  int                    sole[2];                   /* each sole's segment (`touch.SOLES`) */
  int                    ids[WBC_BORNE_SKIN];       /* each segment's body in her model */
  int                    order[WBC_BORNE_SKIN];     /* borne.ABOVE, in its order */
  int                    hand[WBC_BORNE_SKIN];      /* 1 of borne.HANDS */
  int                    right[WBC_BORNE_SKIN];     /* 1 of her right arm's (HANDS past touch.ARMS[0]) */
  int                    caps[WBC_BORNE_SKIN];      /* each one's capsules */
  double                 cap_r[WBC_BORNE_SKIN][WBC_BORNE_CAPS];
  double                 cap_a[WBC_BORNE_SKIN][WBC_BORNE_CAPS][3], cap_b[WBC_BORNE_SKIN][WBC_BORNE_CAPS][3];
  double                 bears_n, seat_x, g_down;   /* BEARS_N, figure.SEAT_X, -edge.G[1] */
  const double          *xmat, *xpos;               /* her model's data as her sense posed it */
  const void            *m, *d;                     /* her model and its data, MuJoCo's */
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
                                                               her pelvis's body */
  double                 seat_dx[2], seat_dz[2];    /* its CoP box's corners along her way and across (SEAT_BOX) */
  double                 touch_m, reach_m, shed0;   /* leap.TOUCH_M, observer.REACH_M, cascade.SHED[0] */
  int                    corner[WBC_BORNE_CONTACTS];  /* a contact's corner of the seat's cone, -1 none */
  double                 seat_on, seat_near;        /* emancipation.seated's g['seat_on'], g['seat_near'] */
  double                 nrm[WBC_BORNE_CONTACTS][3]; /* a contact's normal of what bears it, world (borne._normal) */
} wbc_borne_t;

/** A read (machine.senses.borne.read) of her edge's array `y` at her law's velocity `v`, her `weight`, her hands
    told to bear (`hands`), her legs let go (`let_go`, 2) and her law's pass `dt`: her support's lift `up` and box
    (`low`, `high`, infinite where nothing bears her), her palms and her contacts. 0, or -1 past WBC_BORNE_CONTACTS. */
int wbc_borne_read(wbc_borne_t *b, const double *y, const double *v, double weight, double hands, const double *let_go,
                   double dt);

/** machine.moves.emancipation.seated on a read's contacts: the seat her text tells bears her (`told` its row's share,
    the observer's `points` of its `row` among `rows`, `npoints`) lying within reach of the `floor` - `seat_on` and
    `seat_near` -, and while she sits (`sat` past a half) its contacts out and its CoP box's four corners in, each
    bearing what her step put on it the pass before (`seat_f`) or a quarter of what her skin felt of it, its
    Jacobian and drift at her law's velocity `v`; `emancipated` her legs' release, the larger. 0, or -1 past
    WBC_BORNE_CONTACTS. */
int wbc_borne_seated(wbc_borne_t *b, const double *points, const int64_t *rows, int npoints, int row, double told,
                     double floor, double emancipated, double sat, const double *seat_f, const double *v);

/** machine.moves.emancipation.supports on a read's contacts: what bears her besides her soles - each contact but a
    let-go foot's (`patch` under `let`), its point's x and z and its capsule's room; her observer's `points` (np, 3)
    that `said` keeps, their `rooms`; each sole's corners (`corners0`, `corners1`: nc0, nc1 of 3), room 0 - into `pts`
    (n, 2) and `room` (n), and each leg's own in `soles` (2, n): its sole's corners 1, the other sole's 1 less
    whether that leg stands (`loads` past `touch_n`, its `legs` release under 1). Its n, or -1 past `cap`. */
int wbc_borne_supports(const wbc_borne_t *b, int np_, const unsigned char *said, const double *points,
                       const double *rooms, const double *corners0, int nc0, const double *corners1, int nc1,
                       const double *loads, const double *legs, double touch_n, int cap, double *pts, double *room,
                       double *soles);

#endif
