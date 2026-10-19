"""A portable C core built with the host's compiler into a shared library."""
import glob
import hashlib
import os
import subprocess
from shutil import which

from tools import REPO

OUT = os.path.join(REPO, 'build', 'hosttest')

# The warnings the firmware build puts on the portable cores.
FLAGS = ['-std=c11', '-O1', '-Wall', '-Wextra', '-Wconversion', '-Wshadow']

#: With COAXIAL_GCOV set (tools/dev/cover.py), built for gcov: unoptimised, so a
#: line is a line, the counts beside the library.
GCOV = ['--coverage', '-O0']


def find_cc():
    """A host C compiler, or None. PATH first, then where winget puts one."""
    found = which('gcc') or which('clang') or which('cc')
    if found:
        return found
    packages = os.path.join(os.environ.get('LOCALAPPDATA', ''),
                            'Microsoft', 'WinGet', 'Packages')
    if not os.path.isdir(packages):
        return None
    for root, _dirs, files in os.walk(packages):
        if 'gcc.exe' in files and root.endswith(os.path.join('mingw64', 'bin')):
            return os.path.join(root, 'gcc.exe')
    return None


def build(cc, sources, includes, name, extra=()):
    """(path, warnings) for a shared library, built fresh every run; `extra` flags after the
    firmware's."""
    os.makedirs(OUT, exist_ok=True)
    lib = os.path.join(OUT, name + ('.dll' if os.name == 'nt' else '.so'))
    flags = [] if os.name == 'nt' else ['-fPIC']    # a .so on Linux; mingw warns on it
    for path in includes:
        flags += ['-I', path]
    if os.environ.get('COAXIAL_GCOV'):
        flags += GCOV
    done = subprocess.run([cc, '-shared', '-o', lib] + FLAGS + list(extra) +
                          sources + flags,
                          capture_output=True, text=True, encoding='utf-8',
                          errors='replace')
    if done.returncode != 0:
        raise RuntimeError(done.stderr.strip()[-2000:])
    return lib, [l for l in (done.stderr or '').splitlines() if 'warning:' in l]


def cached(sources, includes, name, extra=()):
    """(path, warnings) of a library built once for these sources', their headers' and the
    flags' bytes, shared by every process that asks - a stand-in opened in each of a gate's
    suites built it once -, a compiler sought only where none is built yet."""
    digest = hashlib.sha1(' '.join(FLAGS + list(extra)).encode())
    digest.update(str(bool(os.environ.get('COAXIAL_GCOV'))).encode())
    headers = sorted(h for d in includes for h in glob.glob(os.path.join(d, '*.h')))
    for path in list(sources) + headers:
        with open(path, 'rb') as f:
            digest.update(f.read())
    lib = os.path.join(OUT, '%s_%s%s' % (name, digest.hexdigest()[:12],
                                          '.dll' if os.name == 'nt' else '.so'))
    if os.path.exists(lib):
        return lib, []
    cc = find_cc()
    if cc is None:
        raise RuntimeError('%s is C built on this host, and there is no C compiler here '
                           '(setup.ps1 installs one)' % name)
    made, warnings = build(cc, sources, includes, '%s_%d' % (name, os.getpid()), extra)
    try:
        os.replace(made, lib)
    except OSError:
        return made, warnings           # another process's is there, loaded
    return lib, warnings


def c_wrench_solve():
    """C11 linear solver and sole cone bounds for `tools.cores.transpile_law`."""
    return '''
static int c_gauss(int n, int nrhs, double *A, double *B) {
  for (int col = 0; col < n; col++) {
    int piv = col;
    double best = fabs(A[col * n + col]);
    for (int r = col + 1; r < n; r++) {
      const double v = fabs(A[r * n + col]);
      if (v > best) { best = v; piv = r; }
    }
    if (best < 1e-18) return 0;
    if (piv != col) {
      for (int c = col; c < n; c++) { double t = A[col * n + c]; A[col * n + c] = A[piv * n + c]; A[piv * n + c] = t; }
      for (int c = 0; c < nrhs; c++) { double t = B[col * nrhs + c]; B[col * nrhs + c] = B[piv * nrhs + c]; B[piv * nrhs + c] = t; }
    }
    const double inv = 1.0 / A[col * n + col];
    for (int r = col + 1; r < n; r++) {
      const double f = A[r * n + col] * inv;
      if (f != 0.0) {
        for (int c = col + 1; c < n; c++) A[r * n + c] -= f * A[col * n + c];
        for (int c = 0; c < nrhs; c++) B[r * nrhs + c] -= f * B[col * nrhs + c];
      }
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    const double inv = 1.0 / A[r * n + r];
    for (int c = 0; c < nrhs; c++) {
      double s = B[r * nrhs + c];
      for (int k = r + 1; k < n; k++) s -= A[r * n + k] * B[k * nrhs + c];
      B[r * nrhs + c] = s * inv;
    }
  }
  return 1;
}

static void c_solve21(const double K[441], const double rhs[42], int nrhs, const double free_u[21], double out[42]) {
  double A[441];
  memcpy(A, K, sizeof(A));
  memcpy(out, rhs, (size_t)(21 * nrhs) * sizeof(double));
  if (!c_gauss(21, nrhs, A, out)) {
    double mdiag = 0.0;
    for (int i = 0; i < 21; i++) mdiag = fmax(mdiag, K[i * 21 + i]);
    memcpy(A, K, sizeof(A));
    memcpy(out, rhs, (size_t)(21 * nrhs) * sizeof(double));
    for (int i = 0; i < 21; i++) A[i * 21 + i] += free_u[i] * RIDGE * mdiag;
    c_gauss(21, nrhs, A, out);
  }
}

static void c_bounded(const double u[21], const double R[2][9], const double stance[2],
                      const double edges[2], const double mu[2], double out[21], double moved[21]) {
  memcpy(out, u, 21 * sizeof(double));
  for (int k = 0; k < 2; k++) {
    const double *uk = u + 6 * k, *Rk = R[k];
    const double m0 = Rk[0] * uk[0] + Rk[3] * uk[1] + Rk[6] * uk[2];
    const double m1 = Rk[1] * uk[0] + Rk[4] * uk[1] + Rk[7] * uk[2];
    const double m2 = Rk[2] * uk[0] + Rk[5] * uk[1] + Rk[8] * uk[2];
    const double fx = uk[3], fy = fmax(uk[4], 0.0), fz = uk[5], ek = edges[k], muk = mu[k];
    const double c0 = ROOM_1 * fy, c1 = muk * YAW_M * fy, c2 = ROOM_0 * fy, mf = muk * fy;
    const double mc0 = c_clip(m0, -c0 * (1.0 - 2.0 * fmax(-ek, 0.0)), c0 * (1.0 - 2.0 * fmax(ek, 0.0)));
    const double mc1 = c_clip(m1, -c1, c1), mc2 = c_clip(m2, -c2, c2);
    const double in_w[6] = {
      Rk[0] * mc0 + Rk[1] * mc1 + Rk[2] * mc2, Rk[3] * mc0 + Rk[4] * mc1 + Rk[5] * mc2,
      Rk[6] * mc0 + Rk[7] * mc1 + Rk[8] * mc2, c_clip(fx, -mf, mf), fy, c_clip(fz, -mf, mf)
    };
    for (int i = 0; i < 6; i++) out[6 * k + i] += stance[k] * (in_w[i] - uk[i]);
  }
  for (int i = 0; i < 21; i++) moved[i] = (fabs(out[i] - u[i]) > 1e-9) ? 1.0 : 0.0;
}
'''


def c_wrench_taken():
    """C11 implementation of `wrench._taken` for `NU = 21`."""
    return '''
static void c_taken(const double u0[21], const double metric[441], const double up[21],
                    const double R[2][9], const double stance[2], const double edges[2],
                    double lo, double hi, double grounded, const double mu[2], double at[21]) {
  double A[441], Gi[441], F[13][21], N[13][21], met[13], share[13], level[13], u[21];
  memcpy(A, metric, sizeof(A));
  memset(Gi, 0, sizeof(Gi));
  for (int i = 0; i < 21; i++) Gi[i * 21 + i] = 1.0;
  c_gauss(21, 21, A, Gi);
  memset(F, 0, sizeof(F));
  memset(N, 0, sizeof(N));
  for (int k = 0; k < 2; k++) {
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) F[6 * k + r][6 * k + c] = R[k][3 * c + r];
      F[6 * k + 3 + r][6 * k + 3 + r] = 1.0;
    }
    for (int r = 0; r < 6; r++) N[6 * k + r][6 * k + 4] = 1.0;
  }
  memcpy(F[12], up, 21 * sizeof(double));
  memset(met, 0, sizeof(met));
  memset(share, 0, sizeof(share));
  memset(level, 0, sizeof(level));
  memcpy(u, u0, sizeof(u));
  memcpy(at, u0, sizeof(u));
  for (int rd = 0; rd < 4; rd++) {
    double ay = -G;
    for (int i = 0; i < 21; i++) ay += up[i] * u[i];
    const double past_y = c_clip(ay, lo, hi) + G;
    level[12] += (1.0 - met[12]) * (past_y - level[12]);
    met[12] = fmax(met[12], grounded * ((fabs(past_y - G - ay) > 1e-9) ? 1.0 : 0.0));
    double C[13][21], CGi[13][21], M13[169], rhs13[13];
    for (int r = 0; r < 13; r++) {
      double cu0 = 0.0;
      for (int c = 0; c < 21; c++) {
        C[r][c] = met[r] * (F[r][c] - share[r] * N[r][c]);
        cu0 += C[r][c] * u0[c];
      }
      rhs13[r] = cu0 - met[r] * level[r];
    }
    for (int r = 0; r < 13; r++) {
      for (int c = 0; c < 21; c++) {
        double s = 0.0;
        for (int k = 0; k < 21; k++) s += C[r][k] * Gi[k * 21 + c];
        CGi[r][c] = s;
      }
    }
    for (int r = 0; r < 13; r++) {
      for (int c = 0; c < 13; c++) {
        double s = (r == c) ? (1.0 - met[r] + ((r == 12) ? 1e-9 : 0.0)) : 0.0;
        for (int k = 0; k < 21; k++) s += CGi[r][k] * C[c][k];
        M13[r * 13 + c] = s;
      }
    }
    c_gauss(13, 1, M13, rhs13);
    for (int i = 0; i < 21; i++) {
      double s = 0.0;
      for (int r = 0; r < 13; r++) s += CGi[r][i] * rhs13[r];
      u[i] = u0[i] - s;
    }
    memcpy(at, u, sizeof(u));
    for (int k = 0; k < 2; k++) {
      const double *uk = u + 6 * k, *Rk = R[k];
      const double was[6] = {
        Rk[0] * uk[0] + Rk[3] * uk[1] + Rk[6] * uk[2], Rk[1] * uk[0] + Rk[4] * uk[1] + Rk[7] * uk[2],
        Rk[2] * uk[0] + Rk[5] * uk[1] + Rk[8] * uk[2], uk[3], uk[4], uk[5]
      };
      const double fy = fmax(was[4], 0.0), ek = edges[k], muk = mu[k];
      const double low[6] = {-ROOM_1 * (1.0 - 2.0 * fmax(-ek, 0.0)), -muk * YAW_M, -ROOM_0, -muk, 0.0, -muk};
      const double high[6] = {ROOM_1 * (1.0 - 2.0 * fmax(ek, 0.0)), muk * YAW_M, ROOM_0, muk, 1.0, muk};
      double in_w[6];
      for (int i = 0; i < 6; i++) {
        const double lfy = low[i] * fy, hfy = high[i] * fy;
        in_w[i] = c_clip(was[i], lfy, hfy);
        const double out_i = stance[k] * ((fabs(in_w[i] - was[i]) > 1e-9) ? 1.0 : 0.0);
        share[6 * k + i] += out_i * (((was[i] < lfy) ? low[i] : high[i]) - share[6 * k + i]);
        met[6 * k + i] = fmax(met[6 * k + i], out_i);
      }
      const double w_rot[3] = {
        Rk[0] * in_w[0] + Rk[1] * in_w[1] + Rk[2] * in_w[2],
        Rk[3] * in_w[0] + Rk[4] * in_w[1] + Rk[5] * in_w[2],
        Rk[6] * in_w[0] + Rk[7] * in_w[1] + Rk[8] * in_w[2]
      };
      for (int i = 0; i < 3; i++) {
        at[6 * k + i] += stance[k] * (w_rot[i] - uk[i]);
        at[6 * k + 3 + i] += stance[k] * (in_w[3 + i] - uk[3 + i]);
      }
    }
  }
}
'''


def c_wrench_rounds():
    """C11 implementation of `wrench._rounds` for `NU = 21`."""
    return c_wrench_solve() + c_wrench_taken() + '''
void law_wrench_rounds(const double *H0, const double *g0, double lo, double hi,
                       const double *part, const double *up, const double *lift,
                       double mass, double grounded, const double *stance,
                       const double *R0, const double *R1, const double *edges,
                       const double *mu, double strides, const double *kinetic, double *at) {
  const double R[2][9] = {
    {R0[0], R0[1], R0[2], R0[3], R0[4], R0[5], R0[6], R0[7], R0[8]},
    {R1[0], R1[1], R1[2], R1[3], R1[4], R1[5], R1[6], R1[7], R1[8]}
  };
  double pinned[21], moved[21], hard[21], soft[21], free_u[21], H[441], K[441], rhs[42], sol[42], u[21], way[21], u0[21];
  memset(pinned, 0, sizeof(pinned));
  memset(at, 0, 21 * sizeof(double));
  memset(u0, 0, sizeof(u0));
  double band = 0.0, edge = 0.0;
  int done = 0, rounds = 4, any_pin = 0;
  while (done < rounds) {
    memcpy(H, H0, sizeof(H));
    for (int i = 0; i < 21; i++) {
      const double is_mom = ((i < 12) && (i % 6 < 3)) ? 1.0 : 0.0;
      hard[i] = pinned[i] * is_mom;
      const double sp = W_PIN * pinned[i] * (1.0 - is_mom) * part[i];
      soft[i] = sp * sp;
      H[i * 21 + i] += soft[i];
      free_u[i] = 1.0 - hard[i];
    }
    for (int r = 0; r < 21; r++) {
      double h_ha = 0.0;
      for (int c = 0; c < 21; c++) {
        K[r * 21 + c] = H[r * 21 + c] * free_u[r] * free_u[c] + ((r == c) ? hard[r] : 0.0);
        h_ha += H[r * 21 + c] * hard[c] * at[c];
      }
      rhs[r * 2 + 0] = free_u[r] * (g0[r] + soft[r] * at[r] - h_ha) + hard[r] * at[r];
      rhs[r * 2 + 1] = up[r];
    }
    c_solve21(K, rhs, 2, free_u, sol);
    double up_u = 0.0, up_w = 0.0;
    for (int i = 0; i < 21; i++) {
      u[i] = sol[i * 2 + 0];
      way[i] = sol[i * 2 + 1];
      up_u += up[i] * u[i];
      up_w += up[i] * way[i];
    }
    if (done == 0) memcpy(u0, u, sizeof(u0));
    const double step = band * (edge - up_u) / (up_w + 1.0 - band);
    double lift_u = 0.0;
    for (int i = 0; i < 21; i++) {
      u[i] += step * way[i];
      lift_u += lift[i] * u[i];
    }
    double ay = lift_u / mass - G;
    const double past_y = c_clip(ay, lo, hi) + G;
    const double next_b = fmax(band, grounded * ((fabs(past_y - G - ay) > 1e-9) ? 1.0 : 0.0));
    edge = edge + (1.0 - band) * (past_y - edge);
    band = next_b;
    c_bounded(u, R, stance, edges, mu, at, moved);
    any_pin = 0;
    double lift_at = 0.0;
    for (int i = 0; i < 21; i++) {
      pinned[i] = fmax(pinned[i], moved[i]);
      if (pinned[i] > 0.0) any_pin = 1;
      lift_at += lift[i] * at[i];
    }
    done++;
    ay = lift_at / mass - G;
    if ((done == rounds) && (rounds < 6) && (strides == 0.0) && (grounded * fabs(c_clip(ay, lo, hi) - ay) > 1e-9)) {
      band = 1.0; edge = c_clip(ay, lo, hi) + G; rounds++;
    }
    if ((!any_pin && (band == 0.0)) || (strides >= 1.0)) break;
  }
  if ((strides > 0.0) && (any_pin || (band > 0.0))) {
    double tk[21];
    c_taken(u0, kinetic, up, R, stance, edges, lo, hi, grounded, mu, tk);
    for (int i = 0; i < 21; i++) at[i] += strides * (tk[i] - at[i]);
  }
}
'''


def c_swings_poses():
    """C11 helpers for `impedance._swings` and `impedance._poses`."""
    return '''
static void c_swings(const double *y, const double *fr, const double v[3], double om,
                     const double shod[2][3], const double floors[2], double rise,
                     const double ref[11], const double rate[11], const double acc[11],
                     const double kp[11], const double kd[11], const double clear[3][2],
                     const double stance[2], double cruise, double grounded,
                     double free_acc[2][3], double frame[3][3]) {
  const double stood = fmax(stance[0], stance[1]);
  const double q0 = y[20], q1 = y[21], q2 = y[22], q3 = y[23];
  const double hers[3][3] = {
    {2.0 * (q1 * q3 + q0 * q2), 2.0 * (q2 * q3 - q0 * q1), 1.0 - 2.0 * (q1 * q1 + q2 * q2)},
    {1.0 - 2.0 * (q2 * q2 + q3 * q3), 2.0 * (q1 * q2 + q0 * q3), 2.0 * (q1 * q3 - q0 * q2)},
    {2.0 * (q1 * q2 - q0 * q3), 1.0 - 2.0 * (q1 * q1 + q3 * q3), 2.0 * (q2 * q3 + q0 * q1)}
  };
  for (int r = 0; r < 3; r++) {
    for (int c = 0; c < 3; c++) frame[r][c] = grounded * fr[3 * r + c] + (1.0 - grounded) * hers[r][c];
  }
  double reach[11], off3[3][2][3];
  for (int n = 0; n < 11; n++) reach[n] = ref[n] - ((n == AXIAL) ? fmin(ref[AXIAL], 0.0) : 0.0);
  memset(off3, 0, sizeof(off3));
  for (int n = 0; n < 11; n++) {
    const double u_n = fr[23 + n], r_z0 = reach[n] - fr[34 + n];
    for (int i = 0; i < 2; i++) {
      for (int d = 0; d < 3; d++) {
        const double h = C_HANDS[(n * 2 + i) * 3 + d] * u_n;
        off3[0][i][d] += h * r_z0; off3[1][i][d] += h * rate[n]; off3[2][i][d] += h * acc[n];
      }
    }
  }
  const double *w = y + 28;
  for (int i = 0; i < 2; i++) {
    const double arm[3] = {y[6 + 3 * i] - y[0], y[7 + 3 * i] - y[1], y[8 + 3 * i] - y[2]};
    const double cr[3] = {w[1] * arm[2] - w[2] * arm[1], w[2] * arm[0] - w[0] * arm[2], w[0] * arm[1] - w[1] * arm[0]};
    const double dv[3] = {y[12 + 3 * i] - y[3] - (1.0 - grounded) * cr[0],
                          y[13 + 3 * i] - y[4] - (1.0 - grounded) * cr[1],
                          y[14 + 3 * i] - y[5] - (1.0 - grounded) * cr[2]};
    for (int d = 0; d < 3; d++) {
      off3[0][i][d] -= dot3(arm, frame[d]);
      off3[1][i][d] -= dot3(dv, frame[d]);
    }
    off3[0][i][2] += clear[0][i]; off3[1][i][2] += clear[1][i]; off3[2][i][2] += clear[2][i];
  }
  const double b0 = (stance[0] + TINY) / (stance[0] + stance[1] + 2.0 * TINY);
  const double b1 = (stance[1] + TINY) / (stance[0] + stance[1] + 2.0 * TINY);
  for (int p = 0; p < 3; p++) {
    const double bw = b0 * off3[p][0][2] + b1 * off3[p][1][2];
    off3[p][0][2] -= stood * bw; off3[p][1][2] -= stood * bw;
  }
  const double al0 = dot3(y + 6, fr + 0), al1 = dot3(y + 9, fr + 0), b_al = b0 * al0 + b1 * al1;
  const double b_sl = b0 * (y[7] - y[37]) + b1 * (y[10] - y[38]);
  const double b_su = b0 * shod[0][2] + b1 * shod[1][2], b_sa = b0 * shod[0][0] + b1 * shod[1][0];
  for (int i = 0; i < 2; i++) {
    const double aloft = stood * (1.0 - stance[i]), ali = (i == 0) ? al0 : al1;
    off3[0][i][2] -= aloft * (b_sl - rise * (ali - b_al));
    off3[1][i][2] -= aloft * (b_su - rise * (shod[i][0] - b_sa));
    const double fl0 = clear[0][i] - (y[37 + i] - floors[i]), fl1 = clear[1][i] - shod[i][2], fl2 = clear[2][i];
    off3[0][i][2] += (grounded - stood) * (fl0 - off3[0][i][2]);
    off3[1][i][2] += (grounded - stood) * (fl1 - off3[1][i][2]);
    off3[2][i][2] += (grounded - stood) * (fl2 - off3[2][i][2]);
  }
  const double lead0 = grounded * (v[0] - cruise) / om;
  const int fb[3] = {DRIVE, WIDTH, LIFT};
  for (int i = 0; i < 2; i++) {
    for (int d = 0; d < 3; d++) {
      free_acc[i][d] = off3[2][i][d] + kp[fb[d]] * (off3[0][i][d] + ((d == 0) ? lead0 : 0.0)) + kd[fb[d]] * off3[1][i][d];
    }
  }
}

static void c_poses(const double *out, const double *y, const double *fr, double *a,
                    const double z[11], const double ref[11], const double rate[11],
                    const double acc[11], const double kp[11], const double kd[11],
                    const double turn[3][2], const double spun_in[3][2],
                    const double up[2], const double folds[2], double still) {
  double spun[3][2];
  memcpy(spun, spun_in, sizeof(spun));
  const double wh0 = dot3(fr + 0, y + 67), wh1 = dot3(fr + 3, y + 67);
  spun[0][1] += 0.5 * still * (wh1 - spun[0][1]);
  spun[1][1] += 0.5 * still * (wh0 - spun[1][1]);
  const double turn_dc = ref[TURN] - out[66 + TURN] * sin(out[55 + TURN]);
  const int by[3] = {PITCH, LIST, TURN};
  for (int k = 0; k < 2; k++) {
    const double pk = C_PELVIS[k];
    const double aim[3] = {fr[15 + k] + PITCH_RAD * ref[PITCH], pk * LIST_RAD * ref[LIST],
                           TURN_RAD * (pk * ref[TURN] + (1.0 - pk) * 1.10 * turn_dc)};
    const double ar[3] = {PITCH_RAD * rate[PITCH], pk * LIST_RAD * rate[LIST], pk * TURN_RAD * rate[TURN]};
    const double aa[3] = {PITCH_RAD * acc[PITCH], pk * LIST_RAD * acc[LIST], pk * TURN_RAD * acc[TURN]};
    double res[3];
    for (int r = 0; r < 3; r++) {
      const double tr = aa[r] + kp[by[r]] * (aim[r] - turn[r][k]) + kd[by[r]] * (ar[r] - spun[r][k]);
      res[r] = turn[r][k] + (tr + TURN_KD * spun[r][k]) / TURN_KP;
    }
    a[11 + k] = res[0]; a[23 + k] = res[1]; a[25 + k] = res[2];
  }
  a[17] = fmax(fmax(a[0], a[1]), a[58]) * ((kp[PITCH] + kd[PITCH] > 0.0) ? 1.0 : 0.0);
  a[29] = ARM_RAD * (ref[ARMS] + (acc[ARMS] + POSTURE_KD * rate[ARMS]) / POSTURE_KP);
  const double bent = z[AXIAL] + c_clip(ref[AXIAL] - z[AXIAL], -AHEAD, AHEAD);
  a[18] = 2.0 * acos(c_clip(cos(0.5 * fr[14]) - AXIAL_M * bent * fr[20] / LEG_M, -1.0, 1.0)) - fr[14];
  for (int k = 0; k < 2; k++) {
    const double sh = c_clip(cos(0.5 * (fr[14] + a[18])) - CLEAR_M * fmax(up[k], 0.0) / LEG_M, -1.0, 1.0);
    a[13 + k] = 2.0 * acos(sh) - fr[14] - a[18];
    a[15 + k] = (2.0 * CLEAR_M * C_LIMBS[k] * rate[LIFT] * ((up[k] > 0.0) ? 1.0 : 0.0) / LEG_M
                 / sqrt(fmax(1.0 - sh * sh, TINY)));
    a[30 + k] = folds[k];
  }
  a[19] = -MOMENTUM_K * y[34];
  a[20] = -MOMENTUM_K * y[35] - YAW_K * (y[35] - c_clip(y[35], -out[79], out[79]));
  a[21] = -MOMENTUM_K * y[36];
}
'''


# setup.ps1's probe, before host/ is installed: winget's gcc is on PATH in new shells only.
if __name__ == '__main__':
    print(find_cc() or '')
