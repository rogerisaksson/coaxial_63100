/** harness_borne.c - A flat C API over wbc_borne.c, wbc_hull.c and wbc_upright.c for tools.cores.borne, through
    ctypes: what bears the robot off its soles, its seat's cone, its supports, its lean, its support's hull and
    control's weighing on them. */
#include "wbc_borne.h"
#include "wbc_hull.h"
#include "wbc_upright.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

API int wbh_borne_read(wbc_borne_t *b, const double *y, const double *v, double hands, const double *let_go)
{
  return wbc_borne_read(b, y, v, hands, let_go);
}

API int wbh_borne_size(void)
{
  return (int)sizeof(wbc_borne_t);
}

API int wbh_borne_seated(wbc_borne_t *b, const double *points, const int64_t *rows, int npoints, int row, double told,
                         double floor, double emancipated, double sat, const double *seat_f, const double *v)
{
  return wbc_borne_seated(b, points, rows, npoints, row, told, floor, emancipated, sat, seat_f, v);
}

API int wbh_borne_supports(const wbc_borne_t *b, int np_, const unsigned char *said, const double *points,
                           const double *rooms, const double *corners0, int nc0, const double *corners1, int nc1,
                           const double *loads, const double *legs, double touch_n, int cap, double *pts, double *room,
                           double *soles)
{
  return wbc_borne_supports(b, np_, said, points, rooms, corners0, nc0, corners1, nc1, loads, legs, touch_n, cap, pts,
                            room, soles);
}

API int wbh_borne_leaned(const wbc_borne_t *b, const double *way, const double *sole0, const double *sole1,
                         const double *com, const double *vcom, double weight, double behind, double tiny, double g,
                         double ankle_h, double *out)
{
  return wbc_borne_leaned(b, way, sole0, sole1, com, vcom, weight, behind, tiny, g, ankle_h, out);
}

API int wbh_hull(const double *pts, int n, double *ring)
{
  return wbc_hull(pts, n, ring);
}

API int wbh_upright_size(void)
{
  return (int)sizeof(wbc_upright_t);
}

API void wbh_upright(const wbc_upright_t *u, const wbc_borne_t *b, const double *st, const double *out, const double *a,
                     const double *rest, const double *told, const double *tab, int rows, double load0, double load1,
                     double weight, double reclined, double sw, double w_hop, double *got)
{
  wbc_upright(u, b, st, out, a, rest, told, tab, rows, load0, load1, weight, reclined, sw, w_hop, got);
}
