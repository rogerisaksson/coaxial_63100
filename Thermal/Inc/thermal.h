/**
  ******************************************************************************
  * @file    thermal.h
  * @brief   Lumped-node thermal observer: what each region of the board is at.
  *
  * Not a finite element model. A mesh and a solver do not fit in a main loop
  * and are not needed to answer "how hot is the gate driver" - six nodes and
  * the resistances between them do. The nodes are the parts that dissipate,
  * not a grid over the copper.
  *
  *   drivers ---+
  *   phases  ---+
  *   mcu     ---+--- board ---- ambient
  *   regs    ---+
  *   afe     ---+
  *
  * A star: every source couples to one board node, the board couples to
  * ambient. It is the coarsest topology that can still tell a hot part from a
  * hot board, which is the whole question the NTC alone cannot answer.
  *
  * TWO SENSORS, AND WHY THAT IS WHAT MAKES IT WORK
  * The NTC sits centrally, beside a gate driver, so it reads that driver's
  * local rise on top of the board. The A1335's TSEN sits out at the SPI4
  * corner and reads close to the board node. Measured 2026-08-28 from a
  * settled baseline, three legs at 50 % for ten minutes:
  *
  *     gate off     NTC - TSEN = -0.74 C     (mounting offset, constant)
  *     three legs   NTC - TSEN = +10.94 C
  *     dNTC +19.56  dTSEN +7.88              NTC overstates by 2.48x
  *
  * So the NTC is not a board thermometer while anything switches, and every
  * board-average figure taken from it alone is high by that factor.
  *
  * The board has no ambient sensor. Two measurements at different points plus
  * the network is what lets ambient be estimated rather than assumed - that is
  * why both sensors are inputs and neither is optional.
  *
  * Portable C11: no HAL, no CMSIS, float only (the M7 has an FPU; the wire
  * still carries integers). Host-testable the same way Modbus/ and Shtp/ are.
  ******************************************************************************
  */
#ifndef THERMAL_H
#define THERMAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The regions that dissipate. Order is the wire order; append only. */
typedef enum
{
  THERMAL_DRIVERS = 0,   /**< 2EDL8034 x3 - the NTC's neighbour           */
  THERMAL_PHASES,        /**< the six IAUCN10S7N021                       */
  THERMAL_MCU,           /**< STM32H753 at 475 MHz, through a linear LDO  */
  THERMAL_REGULATORS,    /**< MP4541 x2 and the LDOs after them           */
  THERMAL_AFE,           /**< THS4551 x3 and the reference                */
  THERMAL_BOARD,         /**< the copper everything sinks into            */
  THERMAL_NODES
} thermal_node_t;

/** One node's thermal properties. Set from the calibration record. */
typedef struct
{
  float to_board;        /**< K/W from this node into the board node   */
  float capacity;        /**< J/K - what sets how fast it responds     */
} thermal_node_cfg_t;

/** What the observer needs to know about the board, once. */
typedef struct
{
  thermal_node_cfg_t node[THERMAL_NODES];
  float board_to_ambient;  /**< K/W, the only path off the board        */

  /** How much of the drivers' rise the NTC sees. Not capped at 1.
    *
    * 0.0 would be a pure board reading. Above 1 means the NTC rises faster
    * than the node's own surface, which happens when it sits closer to the
    * heat than the point the node stands for. Solved against both camera
    * states 2026-08-28 it is **1.055**, and a cap at 1.0 cost 5.6 K in the
    * switching state.
    */
  float ntc_sees_drivers;

  /** The NTC's constant offset over the board, in K. Mounting and the
    * channel's own calibration, not physics: 6.00 measured against a camera
    * in the passive state, where no driver was warming anything. */
  float ntc_offset;
} thermal_cfg_t;

/** Live state. Owned by the caller; `thermal_init` fills it. */
typedef struct
{
  thermal_cfg_t cfg;
  float t[THERMAL_NODES];   /**< degrees C per node          */
  float ambient;            /**< estimated, not measured     */
  bool  settled;            /**< true once the anchor has converged */
  uint32_t steps;
} thermal_t;

/** Dissipation per node, watts. Whoever knows the board's state fills it. */
typedef struct
{
  float watt[THERMAL_NODES];
} thermal_power_t;

/** What the board is doing now, as measured. Input to the estimator.
  *
  * The calibration behind `thermal_defaults` was taken DRY - nothing on the
  * phases, so every current below was zero and the only losses were switching
  * and the static rails. Under load the picture inverts: at the board's 100 A
  * rating the shunt alone makes 35 W against the whole dry budget's 1.2 W.
  */
typedef struct
{
  float phase_amps[3];   /**< per leg, signed, as the shunts measure it   */
  float duty[3];         /**< 0..1 per leg, for the conduction split      */
  float link_volts;      /**< DC link, for the switching terms            */
  float link_amps;       /**< into the board. <0 = estimate from phases   */
  bool  switching;       /**< TIM1 driving the gates                      */
  bool  afe_on;          /**< AFE_ON high: the AFE draws, drivers do not  */
} thermal_load_t;

/** Resistances the estimator needs. Ohms, from electronics/ and the models. */
typedef struct
{
  float rds_on;          /**< one FET, IAUCN10S7N021 VDMOS Ron = 1.8 mOhm */
  float r_shunt;         /**< phase shunt, RU1||RU2 = 3.5 mOhm            */
  float r_hotswap;       /**< LM5069 pass FET, in the link                */
  float switching_watt;  /**< the whole switching loss at `switch_volts`  */
  float switch_volts;    /**< the link it was measured at                 */
  float driver_share;    /**< how much of it lands in the driver zone     */
  float mcu_watt;        /**< static, 475 MHz through the linear LDO      */
  float ldo_watt;        /**< the drop, plus what else the reg zone makes */
  float afe_watt;        /**< the AFE chain when AFE_ON is high           */
} thermal_loss_t;

/** The loss constants as measured/traced on this board. */
void thermal_losses(thermal_loss_t *loss);

/**
  * @brief  Dissipation per node from what the board is doing.
  *
  * Conduction is I^2 through the FET and its shunt, split across the legs by
  * duty. Switching is the calibrated figure scaled by link voltage - C_oss
  * loss goes as Q(V)*V, so roughly with V, not V^2 - and by how many legs
  * are actually driven.
  *
  * `link_amps` below zero is estimated as sum(duty * phase_amps), which is
  * what the link has to supply when nothing is stored. That is the only way
  * to it: this board senses link VOLTS, not link amps.
  */
void thermal_power_estimate(thermal_power_t *out, const thermal_load_t *load,
                            const thermal_loss_t *loss);

/**
  * @brief  Start the observer with every node at one temperature.
  * @param  cfg      Network parameters. Copied.
  * @param  celsius  What to assume everything is at - a first reading.
  */
void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius);

/**
  * @brief  Advance the model and pull it toward what the sensors say.
  * @param  p        Dissipation now, per node.
  * @param  ntc_c    The NTC, degrees C. Required.
  * @param  tsen_c   The A1335's TSEN, or NAN when the part is not answering.
  * @param  dt_s     Seconds since the last step.
  *
  * The model runs open loop between sensor readings and is corrected by
  * them, so a sensor that stops answering degrades the estimate rather than
  * stopping it. With TSEN absent the board node is anchored on the NTC with
  * the drivers' share removed, which is worse but not blind.
  */
void thermal_step(thermal_t *th, const thermal_power_t *p,
                  float ntc_c, float tsen_c, float dt_s);

/**
  * @brief  What the NTC should read, given the model. For checking the fit.
  *
  * The difference between this and the real NTC is the model's error, and it
  * is the one number that says whether the parameters are any good.
  */
float thermal_expected_ntc(const thermal_t *th);

/**
  * @brief  Board temperature with the driver hot spot taken out.
  *
  * What the NTC would read if it were not sitting beside a gate driver.
  */
float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float driver_rise_k);

/** Defaults: the network as measured on this board, 2026-08-28. */
void thermal_defaults(thermal_cfg_t *cfg);

/**
  * @brief  Change one node's parameters while the observer runs.
  * @return False for an unknown node or a non-positive value.
  *
  * The calibration behind the defaults was taken with NOTHING connected to
  * the phases and nothing drawn through the hot swap. Both of those change
  * the moment current flows: the phase node gains conduction loss it has
  * never had, and the hot swap goes from 6 K over the board to whatever its
  * FET dissipates. So these are meant to be re-fitted, from a host, without
  * a reflash - which is why they are here and not `#define`s.
  *
  * Re-fitting is one division per node: `to_board = (T_zone - T_board) / P`,
  * with T from a camera against a dead patch of soldermask and P from the
  * supply. See python_examples/thermal_model.py.
  */
bool thermal_set_node(thermal_t *th, thermal_node_t node,
                      float to_board, float capacity);

/**
  * @brief  Change the board's own two numbers.
  * @return False for a non-positive value.
  *
  * `board_to_ambient` is the one figure with a clean measurement behind it -
  * 8.33 K/W from the passive state against the supply's 50 mA - and it is
  * also the one that moves if the board is ever mounted behind a stator
  * instead of lying on a bench. Still air is not the same as a rotor.
  */
bool thermal_set_board(thermal_t *th, float to_ambient, float capacity);

/**
  * @brief  Change how the NTC is read.
  *
  * `offset` is the constant part, `sees_drivers` the share of the drivers'
  * rise it picks up. Measured 6.0 K and 0.44 - the first with nothing
  * switching, so it is mounting, the second from the switching state.
  */
bool thermal_set_ntc(thermal_t *th, float offset, float sees_drivers);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_H */
