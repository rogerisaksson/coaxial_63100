/** wbc_inekf.h - machine.senses.inekf.stepped's arithmetic around its points and its step in C (wbc_inekf.c): the
    robot's bearing points' frames and weights off its edge's array, each contact's share, and the drift its
    rolling soles add - numpy's sums and maximum, Python's floats, min and sum where the robot's python takes them.
    Portable C11, the host's alone. */
#ifndef WBC_INEKF_H
#define WBC_INEKF_H

/** The robot's bearing points and its edge's array, laid once: each offset into its edge's `y`, its insoles and its
    core's two skin segments (pelvis, torso), the touch felt to count its core's share and a sole's release. */
typedef struct
{
  int    touch, cop, press, at;    /* edge.TOUCH, COP, PRESS, AT's starts */
  int    core[2], insoles, spare;  /* inekf.CORE's segments, edge.INSOLES' count */
  double touched_n, release;       /* edge.TOUCHED_N, observer.RELEASE */
} wbc_inekf_t;

/** A law pass's bearing points laid off the robot's edge's `y`: each insole's and core segment's point in its own
    frame (`local`, insoles then core, 3 each), its weight (`weights`: an insole's press, a core segment's push, each
    plus 1e-9), and each contact's share (`sh`, 3) of `shares` (`n`, zero past them), the core's held to its touch. */
void wbc_inekf_laid(const wbc_inekf_t *k, const double *y, const double *shares, int n, double *local, double *weights,
                    double *sh);

/** The robot's drift `d` (4 by 4) a law pass on: its gyro's (`gyro2dt`) and each contact's roll since its last pass -
    its point's travel squared, no more than `cap`, where its share `sh` holds it and the filter has it `on` - summed in
    Python's floats; the roll summed is returned. */
double wbc_inekf_rolled(const wbc_inekf_t *k, const double *soles, const double *anchors, const double *sh,
                        const double *on, double cap, double gyro2dt, double *d);

#endif
