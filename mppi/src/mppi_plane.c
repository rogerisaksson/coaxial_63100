/** mppi_plane.c - a plane's rollouts and their costs (mppi.h): no branch the data decides */
#ifndef __CUDACC_RTC__
#include <math.h>
#include "mppi.h"
#endif

#define SQ(a) ((a) * (a))
/* 1 over 0, 0 at or under it, a ramp below 1e-12: a step the data takes, as arithmetic. */
#define POS(x) fmin(1.0, fmax(0.0, (x) * 1e12))

MPPI_FN double mppi_plane_rollout(const double *x0, const double *u, const double *ref,
                                  const double *k, const double *c, int nc, int ns,
                                  const double *r, int nr, const double *weight, const double *u0,
                                  int h, double dt, double w_p, double w_v, const double *wind)
{
    double p[3], v[3], R[9], w[3], last[4], cost = 0.0;
    for (int i = 0; i < 3; i++) {
        p[i] = x0[i];
        v[i] = x0[3 + i];
        w[i] = x0[15 + i];
    }
    for (int i = 0; i < 9; i++) R[i] = x0[6 + i];
    for (int i = 0; i < 4; i++) last[i] = u0[i];
    const double mass = k[MP_MASS], wp2 = w_p * w_p, I[3] = {k[MP_IX], k[MP_IY], k[MP_IZ]};
    for (int t = 0; t < h; t++) {
        const double *ut = u + 4 * t, *rf = ref + MR_REF * t;
        const double el = ut[1], ai = ut[2], ru = ut[3];
        const double ax = v[0] - wind[0], ay = v[1] - wind[1], az = v[2] - wind[2];
        const double sx = R[0], sy = R[3], sz = R[6], nx = R[1], ny = R[4], nz = R[7];
        const double bx = R[2], by = R[5], bz = R[8];
        const double along = ax * nx + ay * ny + az * nz, vh = hypot(ax, az);
        const double air = sqrt(ax * ax + ay * ay + az * az);
        /* the propeller, its pitch fixed, through its inflow (aerobat._aero_inflow) */
        const double jj = fmax(0.0, 1.0 - along / (k[MP_J0] * fmax(1.0, k[MP_PITCH] * fabs(w_p) / 6.283185307179586)));
        const double share = jj * (1.0 + 0.04 * k[MP_CL_ALPHA_A] * fmin(0.35, vh / fmax(w_p * k[MP_PROP_R], 8.0)));
        const double lift = k[MP_K_THRUST] * wp2 * share;
        /* the wing nose first, to and past its stall (wingborne.Wing.lift) */
        const double uy = fmax(0.0, along), uz = ax * bx + ay * by + az * bz, v2 = uy * uy + uz * uz;
        const double on = POS(uy) * POS(v2 - 1e-6);
        const double alpha = atan2(uz, on * uy + (1.0 - on));
        const double flat = fmin(1.0, fmax(0.0, (fabs(alpha) - k[MP_STALL]) / k[MP_BLEND]));
        const double sa = sin(alpha), ca = cos(alpha), cln = k[MP_CL_ALPHA] * alpha;
        const double cl = (1.0 - flat) * cln + flat * 2.0 * sa * ca;
        const double cd = (1.0 - flat) * k[MP_CD_LIFT] * cln * cln + flat * 2.0 * sa * sa;
        const double q = on * 0.5 * k[MP_RHO] * k[MP_AREA] * v2, sp = sqrt(fmax(v2, 1e-12));
        const double dx = (uy * nx + uz * bx) / sp, dy = (uy * ny + uz * by) / sp, dz = (uy * nz + uz * bz) / sp;
        const double lx = (uz * nx - uy * bx) / sp, ly = (uz * ny - uy * by) / sp, lz = (uz * nz - uy * bz) / sp;
        /* the side, a plate across the span, against the slip */
        const double slip = -k[MP_RHO] * k[MP_SIDE] * air * (ax * sx + ay * sy + az * sz);
        /* the airframe's drag (aerobat._aero_cda) */
        const double cle = 0.12 * k[MP_CL_ALPHA_A] * k[MP_SHARE] * hypot(el, ai);
        const double dn = 0.5 * k[MP_RHO] * (k[MP_CDA] + k[MP_AREA_A] * (k[MP_CD0] + k[MP_CDI] * cle * cle)) * air;
        double F[3] = {lift * nx + q * cl * lx - q * cd * dx + slip * sx - dn * ax,
                       lift * ny + q * cl * ly - q * cd * dy + slip * sy - dn * ay - mass * k[MP_GRAVITY],
                       lift * nz + q * cl * lz - q * cd * dz + slip * sz - dn * az};
        /* the surfaces in the slipstream (aerobat.slipstream), the prop's torque, the wing's own
           moments (Wing.moments) */
        const double uxb = ax * sx + ay * sy + az * sz, qw = k[MP_WQ] * along, qd = k[MP_WQ] * fabs(along);
        const double qs = 0.5 * k[MP_RHO] * uy * uy + fmax(0.0, lift) / k[MP_A_DISC];
        const double arm = k[MP_ARM], prop = (k[MP_PROP_BASE] + k[MP_PROP_LOAD] * share
                                              + k[MP_PROP_SURF] * (el * el + ai * ai)) * k[MP_K_DRAG] * wp2;
        double T[3] = {k[MP_K_PITCH] * qs * el + arm * k[MP_STAB] * (qw * uz - qd * arm * w[0]),
                       k[MP_K_YAW] * w_v * w_v * ru - prop - qd * k[MP_AREA] * SQ(0.25 * k[MP_SPAN]) * w[1],
                       k[MP_K_ROLL] * qs * ai - arm * k[MP_FIN] * (qw * uxb + qd * arm * w[2])};
        /* the floor under its contact points: a spring and a damper up, friction against the slip */
        const double ww[3] = {R[0] * w[0] + R[1] * w[1] + R[2] * w[2], R[3] * w[0] + R[4] * w[1] + R[5] * w[2],
                              R[6] * w[0] + R[7] * w[1] + R[8] * w[2]};
        double tw[3] = {0.0, 0.0, 0.0};
        for (int n = 0; n < nc; n++) {
            const double *b = c + 5 * n;
            const double a[3] = {R[0] * b[0] + R[1] * b[1] + R[2] * b[2], R[3] * b[0] + R[4] * b[1] + R[5] * b[2],
                                 R[6] * b[0] + R[7] * b[1] + R[8] * b[2]};
            const double pen = fmax(0.0, b[3] - p[1] - a[1]);
            const double vp[3] = {v[0] + ww[1] * a[2] - ww[2] * a[1], v[1] + ww[2] * a[0] - ww[0] * a[2],
                                  v[2] + ww[0] * a[1] - ww[1] * a[0]};
            const double nf = POS(pen) * fmax(0.0, k[MP_K_FLOOR] * pen - k[MP_C_FLOOR] * vp[1]);
            const double sl = sqrt(vp[0] * vp[0] + vp[2] * vp[2] + 0.01);
            const double f[3] = {-b[4] * nf * vp[0] / sl, nf, -b[4] * nf * vp[2] / sl};
            for (int i = 0; i < 3; i++) F[i] += f[i];
            tw[0] += a[1] * f[2] - a[2] * f[1];
            tw[1] += a[2] * f[0] - a[0] * f[2];
            tw[2] += a[0] * f[1] - a[1] * f[0];
        }
        for (int i = 0; i < 3; i++) T[i] += R[i] * tw[0] + R[3 + i] * tw[1] + R[6 + i] * tw[2];
        /* semi-implicit Euler, the attitude by the exponential map */
        const double Iw[3] = {I[0] * w[0], I[1] * w[1], I[2] * w[2]};
        const double cr[3] = {w[1] * Iw[2] - w[2] * Iw[1], w[2] * Iw[0] - w[0] * Iw[2], w[0] * Iw[1] - w[1] * Iw[0]};
        for (int i = 0; i < 3; i++) {
            v[i] += dt * F[i] / mass;
            w[i] += dt * (T[i] - cr[i]) / I[i];
            p[i] += dt * v[i];
        }
        const double tt = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * dt, s1 = sin(tt), c1 = 1.0 - cos(tt);
        double e[3], kx[9], inc[9], Rn[9];
        for (int i = 0; i < 3; i++) e[i] = w[i] * dt / fmax(tt, 1e-12);
        kx[0] = 0.0; kx[1] = -e[2]; kx[2] = e[1]; kx[3] = e[2]; kx[4] = 0.0; kx[5] = -e[0];
        kx[6] = -e[1]; kx[7] = e[0]; kx[8] = 0.0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                inc[3 * i + j] = s1 * kx[3 * i + j]
                                 + c1 * (kx[3 * i] * kx[j] + kx[3 * i + 1] * kx[3 + j] + kx[3 * i + 2] * kx[6 + j]);
        for (int i = 0; i < 3; i++) inc[4 * i] += 1.0;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Rn[3 * i + j] = R[3 * i] * inc[j] + R[3 * i + 1] * inc[3 + j] + R[3 * i + 2] * inc[6 + j];
        for (int i = 0; i < 9; i++) R[i] = Rn[i];
        /* the cost of this step */
        cost += weight[MW_X] * SQ(p[0] - rf[MR_PLACE]) + weight[MW_Z] * SQ(p[2] - rf[MR_PLACE + 2])
                + weight[MW_Y] * rf[MR_W_HEIGHT] * SQ(p[1] - rf[MR_PLACE + 1]);
        cost += weight[MW_SPEED] * (SQ(v[0] - rf[MR_SPEED]) + SQ(v[1] - rf[MR_SPEED + 1]) + SQ(v[2] - rf[MR_SPEED + 2]));
        cost += weight[MW_BELLY] * rf[MR_W_BELLY]
                * (1.0 - (R[2] * rf[MR_BELLY] + R[5] * rf[MR_BELLY + 1] + R[8] * rf[MR_BELLY + 2]));
        cost += weight[MW_NOSE] * rf[MR_W_NOSE]
                * (1.0 - (R[1] * rf[MR_NOSE] + R[4] * rf[MR_NOSE + 1] + R[7] * rf[MR_NOSE + 2]));
        cost += weight[MW_SPIN] * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        for (int j = 1; j < 4; j++) cost += weight[MW_RESERVE] * SQ(fmax(0.0, fabs(ut[j]) - 0.5));
        for (int n = 0; n < nc; n++) {
            const double *b = c + 5 * n;
            cost += weight[MW_TOUCH] * SQ(fmax(0.0, -0.02 - (p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2] - b[3])));
        }
        double low = 1e3;
        for (int n = 0; n < ns; n++) {
            const double *b = c + 5 * n;
            low = fmin(low, p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2] - b[3]);
        }
        cost += weight[MW_SKIM] * rf[MR_W_SKIM] * SQ(low - rf[MR_SKIM]);
        for (int n = 0; n < nr; n++) {
            const double *b = r + 3 * n;
            cost += weight[MW_CRASH] * SQ(fmax(0.0, 0.03 - (p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2])));
        }
        for (int j = 0; j < 4; j++) {
            cost += weight[MW_EASE] * SQ(ut[j] - last[j]);
            last[j] = ut[j];
        }
        cost += weight[MW_THRUST] * ut[0] * ut[0];
    }
    return fmin(cost, 1e18);
}

#ifndef __CUDACC_RTC__
void mppi_plane_costs(const double *x0, const double *u, const double *ref, const double *k,
                      const double *c, int nc, int ns, const double *r, int nr,
                      const double *weight, const double *u0, int n, int h, double dt, double w_p,
                      double w_v, const double *wind, double *cost)
{
    for (int i = 0; i < n; i++)
        cost[i] = mppi_plane_rollout(x0, u + (long)i * h * 4, ref, k, c, nc, ns, r, nr, weight, u0, h,
                                     dt, w_p, w_v, wind);
}
#endif
