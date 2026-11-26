/** mppi_plane.c - a plane's rollouts and their costs (mppi.h): no branch the data decides */
#ifndef __CUDACC_RTC__
#include <math.h>
#include "board_units.h"
#include "mppi.h"
#define MPPI_RESTRICT restrict
#endif
#ifdef __CUDACC_RTC__
#define MPPI_RESTRICT
#endif

#define SQ(a) ((a) * (a))
/* 1 over 0, 0 at or under it, a ramp below 1e-12: a step the data takes, as arithmetic. */
#define POS(x) fmin(1.0, fmax(0.0, (x) * 1e12))

MPPI_FN double mppi_plane_rollout(const double *MPPI_RESTRICT x0, const double *MPPI_RESTRICT u,
                                  const double *MPPI_RESTRICT ref, const double *MPPI_RESTRICT k,
                                  const double *MPPI_RESTRICT c, int nc, int ns,
                                  const double *MPPI_RESTRICT r, int nr,
                                  const double *MPPI_RESTRICT weight, const double *MPPI_RESTRICT u0,
                                  int h, double dt, double w_p, double w_v,
                                  const double *MPPI_RESTRICT wind)
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
    const double wx = wind[0], wy = wind[1], wz = wind[2], mg = mass * k[MP_GRAVITY];
    const double j_den = k[MP_J0] * fmax(1.0, k[MP_PITCH] * fabs(w_p) / TAU_D);
    const double wake_den = fmax(w_p * k[MP_PROP_R], k[MP_WAKE_FLOOR]);
    const double c_inf = 0.04 * k[MP_CL_ALPHA_A], inf_fl = k[MP_INFLOW_FLOOR];
    const double k_thr_wp2 = k[MP_K_THRUST] * wp2, stall = k[MP_STALL], blend = k[MP_BLEND];
    const double cl_a = k[MP_CL_ALPHA], cd_l = k[MP_CD_LIFT], rho = k[MP_RHO], area = k[MP_AREA];
    const double neg_rho_side = -rho * k[MP_SIDE], k_cle = 0.12 * k[MP_CL_ALPHA_A] * k[MP_SHARE];
    const double cda = k[MP_CDA], area_a = k[MP_AREA_A], cd0 = k[MP_CD0], cdi = k[MP_CDI];
    const double wq = k[MP_WQ], a_disc = k[MP_A_DISC], arm = k[MP_ARM], span2 = SQ(0.25 * k[MP_SPAN]);
    const double p_base = k[MP_PROP_BASE], p_load = k[MP_PROP_LOAD], p_surf = k[MP_PROP_SURF];
    const double k_drag = k[MP_K_DRAG], k_pitch = k[MP_K_PITCH], k_roll = k[MP_K_ROLL];
    const double stab = k[MP_STAB], fin = k[MP_FIN], ky_wv2 = k[MP_K_YAW] * w_v * w_v;
    const double k_fl = k[MP_K_FLOOR], c_fl = k[MP_C_FLOOR], touch = k[MP_TOUCH], skim = k[MP_SKIM];
    const double wx_w = weight[MW_X], wy_w = weight[MW_Y], wz_w = weight[MW_Z], wsp_w = weight[MW_SPEED];
    const double wb_w = weight[MW_BELLY], wn_w = weight[MW_NOSE], wspin_w = weight[MW_SPIN];
    const double wres_w = weight[MW_RESERVE], wtouch_w = weight[MW_TOUCH], wskim_w = weight[MW_SKIM];
    const double wcrash_w = weight[MW_CRASH], wease_w = weight[MW_EASE], wthr_w = weight[MW_THRUST];
    for (int t = 0; t < h; t++) {
        const double *ut = u + 4 * t, *rf = ref + MR_REF * t;
        const double el = ut[1], ai = ut[2], ru = ut[3];
        const double ax = v[0] - wx, ay = v[1] - wy, az = v[2] - wz;
        const double sx = R[0], sy = R[3], sz = R[6], nx = R[1], ny = R[4], nz = R[7];
        const double bx = R[2], by = R[5], bz = R[8];
        const double along = ax * nx + ay * ny + az * nz, vh = hypot(ax, az);
        const double air = sqrt(ax * ax + ay * ay + az * az), uxb = ax * sx + ay * sy + az * sz;
        /* the propeller, its pitch fixed, through its inflow (aerobat._aero_inflow) */
        const double jj = fmax(0.0, 1.0 - along / j_den);
        const double share = jj * (1.0 + c_inf * fmin(inf_fl, vh / wake_den));
        const double lift = k_thr_wp2 * share;
        /* the wing nose first, to and past its stall (wingborne.Wing.lift) */
        const double uy = fmax(0.0, along), uz = ax * bx + ay * by + az * bz, v2 = uy * uy + uz * uz;
        const double on = POS(uy) * POS(v2 - 1e-6);
        const double alpha = atan2(uz, on * uy + (1.0 - on));
        const double flat = fmin(1.0, fmax(0.0, (fabs(alpha) - stall) / blend));
        const double sa = sin(alpha), ca = cos(alpha), cln = cl_a * alpha;
        const double cl = (1.0 - flat) * cln + flat * 2.0 * sa * ca;
        const double cd = (1.0 - flat) * cd_l * cln * cln + flat * 2.0 * sa * sa;
        const double q = on * 0.5 * rho * area * v2, sp = sqrt(fmax(v2, 1e-12));
        const double dx = (uy * nx + uz * bx) / sp, dy = (uy * ny + uz * by) / sp, dz = (uy * nz + uz * bz) / sp;
        const double lx = (uz * nx - uy * bx) / sp, ly = (uz * ny - uy * by) / sp, lz = (uz * nz - uy * bz) / sp;
        /* the side, a plate across the span, against the slip */
        const double slip = neg_rho_side * air * uxb;
        /* the airframe's drag (aerobat._aero_cda) */
        const double cle = k_cle * hypot(el, ai);
        const double dn = 0.5 * rho * (cda + area_a * (cd0 + cdi * cle * cle)) * air;
        double F[3] = {lift * nx + q * cl * lx - q * cd * dx + slip * sx - dn * ax,
                       lift * ny + q * cl * ly - q * cd * dy + slip * sy - dn * ay - mg,
                       lift * nz + q * cl * lz - q * cd * dz + slip * sz - dn * az};
        /* the surfaces in the slipstream (aerobat.slipstream), the prop's torque, the wing's own
           moments (Wing.moments) */
        const double qw = wq * along, qd = wq * fabs(along);
        const double qs = 0.5 * rho * uy * uy + fmax(0.0, lift) / a_disc;
        const double prop = (p_base + p_load * share + p_surf * (el * el + ai * ai)) * k_drag * wp2;
        double T[3] = {k_pitch * qs * el + arm * stab * (qw * uz - qd * arm * w[0]),
                       ky_wv2 * ru - prop - qd * area * span2 * w[1],
                       k_roll * qs * ai - arm * fin * (qw * uxb + qd * arm * w[2])};
        /* the floor under the plane's contact points: a spring and a damper up, friction
           against the slip */
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
            const double nf = POS(pen) * fmax(0.0, k_fl * pen - c_fl * vp[1]);
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
        cost += wx_w * SQ(p[0] - rf[MR_PLACE]) + wz_w * SQ(p[2] - rf[MR_PLACE + 2])
                + wy_w * rf[MR_W_HEIGHT] * SQ(p[1] - rf[MR_PLACE + 1]);
        cost += wsp_w * (SQ(v[0] - rf[MR_SPEED]) + SQ(v[1] - rf[MR_SPEED + 1]) + SQ(v[2] - rf[MR_SPEED + 2]));
        cost += wb_w * rf[MR_W_BELLY]
                * (1.0 - (R[2] * rf[MR_BELLY] + R[5] * rf[MR_BELLY + 1] + R[8] * rf[MR_BELLY + 2]));
        cost += wn_w * rf[MR_W_NOSE]
                * (1.0 - (R[1] * rf[MR_NOSE] + R[4] * rf[MR_NOSE + 1] + R[7] * rf[MR_NOSE + 2]));
        cost += wspin_w * (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
        for (int j = 1; j < 4; j++) cost += wres_w * SQ(fmax(0.0, fabs(ut[j]) - 0.5));
        for (int n = 0; n < nc; n++) {
            const double *b = c + 5 * n;
            cost += wtouch_w
                    * SQ(fmax(0.0, -touch - (p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2] - b[3])));
        }
        double low = 1e3;
        for (int n = 0; n < ns; n++) {
            const double *b = c + 5 * n;
            low = fmin(low, p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2] - b[3]);
        }
        cost += wskim_w * rf[MR_W_SKIM] * SQ(low - rf[MR_SKIM]);
        for (int n = 0; n < nr; n++) {
            const double *b = r + 3 * n;
            cost += wcrash_w * SQ(fmax(0.0, skim - (p[1] + R[3] * b[0] + R[4] * b[1] + R[5] * b[2])));
        }
        for (int j = 0; j < 4; j++) {
            cost += wease_w * SQ(ut[j] - last[j]);
            last[j] = ut[j];
        }
        cost += wthr_w * ut[0] * ut[0];
    }
    return fmin(cost, 1e18);
}

#ifndef __CUDACC_RTC__
void mppi_plane_costs(const double *MPPI_RESTRICT x0, const double *MPPI_RESTRICT u,
                      const double *MPPI_RESTRICT ref, const double *MPPI_RESTRICT k,
                      const double *MPPI_RESTRICT c, int nc, int ns,
                      const double *MPPI_RESTRICT r, int nr, const double *MPPI_RESTRICT weight,
                      const double *MPPI_RESTRICT u0, int n, int h, double dt, double w_p,
                      double w_v, const double *MPPI_RESTRICT wind, double *MPPI_RESTRICT cost)
{
    for (int i = 0; i < n; i++)
        cost[i] = mppi_plane_rollout(x0, u + (long)i * h * 4, ref, k, c, nc, ns, r, nr, weight, u0, h,
                                     dt, w_p, w_v, wind);
}
#endif
