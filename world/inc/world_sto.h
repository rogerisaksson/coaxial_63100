/** world_sto.h - The STO chain as the schematic draws it: the master's common-mode pilot on
    RS485 A1/B1 (UART5), U16C's pulse train pumped into Cinj, U16B's clamp, U4 on Cinj, the
    keepalive's pump into Clevel, Q10, U11, U9's +15V7 (RS485, STO and Regulators sheets);
    electronic_simulations/sto/sto.asc's circuit, its master included. */
#ifndef WORLD_STO_H
#define WORLD_STO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The substep, s. */
#define WORLD_STO_DT 1e-6

/** The gate drivers' UVLO, V: the 2EDL8034F5 model's (motor_inverters/half_bridge). */
#define WORLD_STO_UVLO 7.5f

/** What the chain is given over one step. */
typedef struct
{
  float pilot_volts;      /**< the master's amplifier, amplitude V (sto.asc's PAM8406: 1.5) */
  float pilot_hz;
  float noise_volts;      /**< the far end's common mode at 100 kHz, amplitude V (sto.asc: 1.8) */
  float link_volts;       /**< VBUS: the LM5069's window sets PGD, U11's B */
  bool  rail5;            /**< +5, AFE_ON's LDO: U16's supply and thresholds */
  bool  keepalive;        /**< PA10 at the step's start */
  uint32_t edges;         /**< PA10's edges within the step */
  const float *at;        /**< their times from the step's start, s, ascending; NULL spreads them */
} world_sto_in_t;

typedef struct
{
  float cinj;             /**< V, PC1 */
  float clevel;           /**< V, PB1 */
  float reset;            /**< V, U4's RESET */
  float faultout;         /**< V, U11's Y: PE15 (nFAULT, BKIN), U9's enable */
  float vgate;            /**< V, +15V7: the gate drivers' supply */
} world_sto_out_t;

/** What the chain runs in: while it holds and the outputs have settled in it, the chain is
    held, its substeps skipped. */
typedef struct
{
  float pilot_volts, pilot_hz, noise_volts;
  bool  pumping;            /**< the keepalive's every gap within 25 us */
  bool  rail5, pgood;
} world_sto_regime_t;

typedef struct
{
  double loop;              /**< V, the master's coupling caps */
  double pc, ps;            /**< the pilot's phasor */
  double rc, rs, hz;        /**< its turn a substep, at `hz` */
  double c75, n024, c70, n007, n010;
  double c101, n025;
  double cinj;
  double c71, n032, n_reset;
  double clevel;
  double gate_at, drain;    /**< Q10A's drain, at Clevel `gate_at` */
  double crowbar;           /**< S, Q10B through R92 */
  double fault;             /**< V, U11's Y */
  double gate;              /**< V, +15V7 */
  double ct;                /**< V, U4's timing capacitor */
  double falling;           /**< s, U4's VDD under VIT- */
  double owed;              /**< s, short of a substep at the last step's end */
  double noise;             /**< the disturbance's phase, turns */
  double cmp1, cmp2;        /**< U16C's and U16B's inner node, V */
  double i13, i14, i10;     /**< the diodes in series with a resistor: their last current, A */
  bool   u1, u2;            /**< U16C, U16B high */
  bool   sensed;            /**< U4's VDD seen over VIT+ */
  bool   c_high;            /**< U11's C through its Schmitt input */
  bool   pgood;
  bool   dcdc;              /**< U9 enabled */
  bool   pin;               /**< PA10 */
  world_sto_regime_t regime;
  double since;             /**< s since PA10's last edge */
  double window;            /**< s into the regime's current window */
  double sums[3], means[3]; /**< Cinj, Clevel, +15V7: the window's integrals, the last one's means */
  bool   settled;           /**< a window has closed in this regime */
  bool   held;
} world_sto_t;

/** A board powered with its bus: every node at rest, the master's coupling caps charged. */
void world_sto_init(world_sto_t *s);

/** `dt` s on `in`, in WORLD_STO_DT substeps; the pins at its end into `out`. */
void world_sto_step(world_sto_t *s, const world_sto_in_t *in, float dt, world_sto_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* WORLD_STO_H */
