/** wbc_regressor.h - her Newton-Euler regressor on the host: machine/regressor.py's `regressor` in C
    (wbc_regressor.c), numpy's arithmetic operation for operation. Portable C11, the host's alone. */
#ifndef WBC_REGRESSOR_H
#define WBC_REGRESSOR_H

/** machine.regressor's layout (sysid.laid): its bodies, hinges, parameters, MuJoCo's dofs, its
    driven hinges, the drives' first column; each body's parent and hinge span (nb, 2), each
    hinge's dof, each driven hinge and its dof, which hinges each body is under (nb, nh). */
typedef struct
{
  int                  nb, nh, p, nv, nd, drive;
  const int           *parent, *span, *hinge, *driven, *dof;
  const unsigned char *above;
} wbc_regressor_t;

/** regressor: Y (n, nh + 6, p) at `n` states - her frames (xpos (n, nb, 3), xmat (n, nb, 9), each
    hinge's axis and anchor (n, nh, 3), regressor.posed), qvel and qacc (n, nv), a0 (n, 3), wacc
    (n, nd) or NULL; `work` 12 nb + 3 nh doubles. */
void wbc_regressor(const wbc_regressor_t *r, int n, const double *xpos, const double *xmat, const double *axis,
                   const double *anchor, const double *qvel, const double *qacc, const double *a0,
                   const double *wacc, double *Y, double *work);

#endif
