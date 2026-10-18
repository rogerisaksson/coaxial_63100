/** wbc_ortho.h - an orthokinematic row on the wire (machine.ortho.ROW): 252 bytes. */
#ifndef WBC_ORTHO_H
#define WBC_ORTHO_H

#include <stdint.h>

/** Her latent modes (machine.ortho.N) and a row's parameters (machine.ortho.P): each mode's
    target, stiffness, damping, amplitude and phase (rad), then her rhythm's gain and Hz, her
    ZMP's polygon and her yaw momentum's limit. */
#define ORTHO_MODES 11
#define ORTHO_P (5 * 11 + 4)
#define ORTHO_Z 0
#define ORTHO_K 11
#define ORTHO_D 22
#define ORTHO_AMP 33
#define ORTHO_PHASE 44
#define ORTHO_GAIN 55
#define ORTHO_HZ 56
#define ORTHO_ZMP 57
#define ORTHO_YAW 58

/** A row's flag: every parameter back to her rest, blended ORTHO_REST_MS (machine.ortho). */
#define ORTHO_REST 1U
#define ORTHO_REST_MS 200U

/** A row on the wire (`<4I59f`, 252 bytes, `machine.ortho.ROW` / `machine.latent.WIRE`):
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

_Static_assert(sizeof(ortho_row_t) == 252U, "ortho_row_t is 252 bytes");

#endif
