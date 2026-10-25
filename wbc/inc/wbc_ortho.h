/** wbc_ortho.h - an orthokinematic row on the wire (machine.ortho.ROW): 328 bytes. */
#ifndef WBC_ORTHO_H
#define WBC_ORTHO_H

#include <stdint.h>

/** Her latent modes (machine.ortho.N) and a row's parameters (machine.ortho.P): each mode's
    target, stiffness, damping, amplitude and phase (rad), then her rhythm's gain and Hz, her
    ZMP's polygon and her yaw momentum's limit, her sidechain's duck, attack and release, her
    contact manifolds' weights (machine.ortho.MANIFOLDS), her slide over what bears her, m/s
    (machine.ortho.SLIDE). */
#define ORTHO_MODES 13
#define ORTHO_MANIFOLDS 5
#define ORTHO_P (5 * 13 + 13)
#define ORTHO_Z 0
#define ORTHO_K 13
#define ORTHO_D 26
#define ORTHO_AMP 39
#define ORTHO_PHASE 52
#define ORTHO_GAIN 65
#define ORTHO_HZ 66
#define ORTHO_ZMP 67
#define ORTHO_YAW 68
#define ORTHO_DUCK 69
#define ORTHO_MANIFOLD 72
#define ORTHO_SLIDE 77

/** A row's flag: every parameter back to her rest, blended ORTHO_REST_MS (machine.ortho). */
#define ORTHO_REST 1U
#define ORTHO_REST_MS 200U

/** A row on the wire (`<4I78f`, 328 bytes, `machine.ortho.ROW` / `machine.latent.WIRE`):
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

_Static_assert(sizeof(ortho_row_t) == 328U, "ortho_row_t is 328 bytes");

#endif
