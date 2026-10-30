/** wbc_soa.h - the kernel's SoA ABI (ast_to_soa_lowering.lisp, machine.texts.lowering): a bank of floats on
    64-byte lines a tuner lowers a gait into and her loop reads. The text's fields at its offsets, SI;
    a float a node where a field is each node's (her nominal gait's nodes, in its order); a biquad a
    node, a column a coefficient or state; the manifolds' weights; the body schema's capsules, a
    column a coordinate. A bank rides machine.process.rcu's table as its words. */
#ifndef WBC_SOA_H
#define WBC_SOA_H

#include <stddef.h>

#define WBC_SOA_LINE 64
/** Floats a line: a node column's room. */
#define WBC_SOA_NODES 16
/** The capsules' room: the schema's 18 and its 4 payload slots on whole lines. */
#define WBC_SOA_CAPSULES 32

typedef struct
{
  _Alignas(WBC_SOA_LINE) float hz; /**< her clock, Hz */
  float duty;                      /**< her stance's share of a stride */
  float clock_[14];
  float sole_stack; /**< m */
  float plantar_[15];
  float track, stride, clearance; /**< m */
  float tarsal_[13];
  float free_[16];
  float dc_offset[4 * WBC_SOA_NODES]; /**< rad, a node each */
  float gamma;                        /**< her capture point's barrier, 1/s */
  float capture_margin;               /**< m */
  float barrier_[14];
  float eta;            /**< the wayfinder's ECBF pullback */
  float shin_clearance; /**< m */
  float wayfinder_[14];
  float b0[WBC_SOA_NODES], b1[WBC_SOA_NODES], b2[WBC_SOA_NODES], a1[WBC_SOA_NODES], a2[WBC_SOA_NODES];
  float s1[WBC_SOA_NODES], s2[WBC_SOA_NODES];
  float weight[WBC_SOA_NODES];                               /**< each manifold's */
  float p0[3][WBC_SOA_CAPSULES], p1[3][WBC_SOA_CAPSULES];    /**< m, in its segment's frame */
  float r0[WBC_SOA_CAPSULES], r1[WBC_SOA_CAPSULES];          /**< m */
} wbc_soa_t;

/* The text's offsets (*abi-field-descriptors*). */
_Static_assert(offsetof(wbc_soa_t, hz) == 0x0000, "bpm");
_Static_assert(offsetof(wbc_soa_t, duty) == 0x0004, "stance-duty-pwm");
_Static_assert(offsetof(wbc_soa_t, sole_stack) == 0x0040, "sole-stack-z-mm");
_Static_assert(offsetof(wbc_soa_t, track) == 0x0080, "track-width-mm");
_Static_assert(offsetof(wbc_soa_t, stride) == 0x0084, "stride-length-mm");
_Static_assert(offsetof(wbc_soa_t, clearance) == 0x0088, "swing-clearance-z-mm");
_Static_assert(offsetof(wbc_soa_t, dc_offset) == 0x0100, "dc-offset-rad");
_Static_assert(offsetof(wbc_soa_t, gamma) == 0x0200, "barrier-gamma");
_Static_assert(offsetof(wbc_soa_t, capture_margin) == 0x0204, "min-capture-margin-mm");
_Static_assert(offsetof(wbc_soa_t, eta) == 0x0240, "ecbf-pullback-stiffness-eta");
_Static_assert(offsetof(wbc_soa_t, shin_clearance) == 0x0244, "min-shin-clearance-mm");
_Static_assert(offsetof(wbc_soa_t, b0) % WBC_SOA_LINE == 0 && offsetof(wbc_soa_t, weight) % WBC_SOA_LINE == 0
                   && offsetof(wbc_soa_t, p0) % WBC_SOA_LINE == 0 && sizeof(wbc_soa_t) % WBC_SOA_LINE == 0,
               "each column on its own lines");

/** Her loop's pass of node `k`'s biquad of bank `b` on `x`, direct form II transposed: its output,
    its states advanced. */
float wbc_soa_biquad(wbc_soa_t *b, int k, float x);

#endif
