/** vision.h - Shared prototypes and persistent worker pool for host C11 vision kernels. */
#ifndef VISION_H
#define VISION_H

#include <stddef.h>
#include <stdint.h>

#define VS_MAX_D_STRIDE 128
#define VS_MAX_THREADS  8

typedef void (*vs_task_fn)(void *ctx, int32_t tid, int32_t nt);

static inline int32_t vs_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline int32_t vs_popcount64(uint64_t x)
{
#if defined(__GNUC__) || defined(__clang__)
  return __builtin_popcountll(x);
#else
  x = x - ((x >> 1U) & 0x5555555555555555ULL);
  x = (x & 0x3333333333333333ULL) + ((x >> 2U) & 0x3333333333333333ULL);
  return (int32_t)((((x + (x >> 4U)) & 0x0F0F0F0F0F0F0F0FULL) * 0x0101010101010101ULL) >> 56U);
#endif
}

int32_t vs_threads_acquire(int32_t max_want);
void    vs_threads_release(int32_t nt);
void    vs_parallel(int32_t nt, vs_task_fn fn, void *ctx);

void vs_grey_u8(const uint8_t *restrict rgb, int64_t *restrict out, int32_t npix);
void vs_box_i64(const int64_t *restrict src, int64_t *restrict dst, int64_t *restrict hsum, int64_t *restrict vsum,
                int32_t height, int32_t width, int32_t rad);
int32_t vs_stereo_core(const int64_t *restrict gl, const int64_t *restrict gr, int32_t height, int32_t width,
                       int32_t d_count, int32_t d_stride, int32_t rx, int32_t ry, int64_t quantum, int32_t rad,
                       float p1, float p2, uint64_t *restrict census_buf, uint8_t *restrict cost_u8,
                       float *restrict total_hwd, float *restrict wave_buf, int64_t *restrict ring_buf,
                       int64_t *restrict vsum_buf, int64_t *restrict best_out, float *restrict tbest_out,
                       float *restrict second_out, int64_t *restrict seen_out, float *restrict c0_out,
                       float *restrict lo_out, float *restrict hi_out);
void vs_stereo_post(const int64_t *restrict gl, const int64_t *restrict best, const float *restrict tbest,
                    const float *restrict second, const int64_t *restrict seen, const float *restrict c0,
                    const float *restrict lo, const float *restrict hi, int32_t height, int32_t width,
                    int32_t d_count, int32_t rad, double texture, float unique_f, double edge_px,
                    int64_t *restrict box_a, int64_t *restrict box_b, int64_t *restrict box_c,
                    int64_t *restrict vsum_buf, double *restrict disp_out, uint8_t *restrict kept_out);
void vs_voxel_add(const double *restrict xyz, const float *restrict rgb, int32_t n_pts, const double *restrict eye,
                  int32_t nx, int32_t ny, int32_t nz, double ox, double floor_m, double oz, double voxel,
                  double ceil_m, double near_m, double far_m, double step_vox, int32_t hits_min, int32_t told_min,
                  int32_t *restrict hits, int32_t *restrict told, int32_t *restrict free_arr, double *restrict top,
                  double *restrict moments, float *restrict colour, uint8_t *restrict occ_buf,
                  int32_t *restrict pass_cnt, int32_t *restrict counts_buf, int64_t *restrict out_flat);
void vs_warp_u8(const uint8_t *restrict src, const int32_t *restrict at4, const int32_t *restrict wt4,
                const uint8_t *restrict on_mask, int32_t height, int32_t width, int32_t bits, uint8_t *restrict dst);
void vs_epi_from_d(const double *restrict d_xyz, int32_t height, int32_t width, double f_px, int32_t bits, int32_t fine,
                   int64_t *restrict v_out, int64_t *restrict u_out, uint8_t *restrict on_out,
                   int32_t *restrict at4_out, int32_t *restrict wt4_out);
void vs_epi_kept(const uint8_t *restrict inner0, const uint8_t *restrict inner1, const double *restrict disp,
                 int32_t height, int32_t width, uint8_t *restrict out);
int32_t vs_convex_f64(const double *restrict pts_in, int32_t n_pts, double *restrict work_xy, double *restrict out_xy);
int32_t vs_grid_cc(const uint8_t *restrict on_u8, const int64_t *restrict codes, int32_t ndim, int32_t d0, int32_t d1,
                   int32_t d2, const int32_t *restrict offs3, int32_t n_off, int32_t *restrict pos_buf,
                   int32_t *restrict parent_buf, int32_t *restrict aux_buf, int64_t *restrict coords_out,
                   int32_t *restrict label_out, int32_t *restrict starts_out);

#endif
