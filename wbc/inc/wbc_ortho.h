/** wbc_ortho.h - an orthokinematic row on the wire (machine.ortho.ROW): 304 bytes. */
#ifndef WBC_ORTHO_H
#define WBC_ORTHO_H

#include <stdint.h>

/** Her latent modes (machine.ortho.N) and a row's parameters (machine.ortho.P): each mode's
    target, stiffness, damping, amplitude and phase (rad), then her rhythm's gain and Hz, her
    ZMP's polygon and her yaw momentum's limit, her sidechain's duck, attack and release, her
    contact manifolds' weights (machine.ortho.MANIFOLDS), her slide over what bears her, m/s
    (machine.ortho.SLIDE). */
#define ORTHO_MODES 12
#define ORTHO_MANIFOLDS 4
#define ORTHO_P (5 * 12 + 12)
#define ORTHO_Z 0
#define ORTHO_K 12
#define ORTHO_D 24
#define ORTHO_AMP 36
#define ORTHO_PHASE 48
#define ORTHO_GAIN 60
#define ORTHO_HZ 61
#define ORTHO_ZMP 62
#define ORTHO_YAW 63
#define ORTHO_DUCK 64
#define ORTHO_MANIFOLD 67
#define ORTHO_SLIDE 71

/** A row's flag: every parameter back to her rest, blended ORTHO_REST_MS (machine.ortho). */
#define ORTHO_REST 1U
#define ORTHO_REST_MS 200U

/** A row on the wire (`<4I72f`, 304 bytes, `machine.ortho.ROW` / `machine.latent.WIRE`):
    its sequence id, the ms from the behaviour's start it is injected at, how long her state
    takes to blend to it, its flags, and its parameters (NaN where unsaid). */
typedef struct
{
  uint32_t id;
  uint32_t at;
  uint32_t blend;
  uint32_t flags;
  float    p[ORTHO_P];
} ortho_row_t;

_Static_assert(sizeof(ortho_row_t) == 304U, "ortho_row_t is 304 bytes");

#endif
