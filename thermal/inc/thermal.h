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
  * still carries integers). Host-testable the same way modbus/ and shtp/ are.
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
/** PER LEG, because a leg that is not switching does not get warm.
  *
  * `drivers` and `phases` were one node each, so the model scaled the loss
  * by how many legs were driven and then spread it over all three. Measured
  * with a thermal camera 2026-08-29: switching U alone heats U's half-bridge
  * and the estimate showed all three the same.
  *
  * The split preserves the bulk exactly - each leg carries a third of the
  * capacity and three times the resistance to board, which in parallel is
  * what the lumped node had. So the four-state camera calibration still
  * holds, and only the PLACEMENT changed.
  */
typedef enum
{
  THERMAL_DRIVER_U = 0,  /**< one 2EDL8034. V is the NTC's neighbour      */
  THERMAL_DRIVER_V,
  THERMAL_DRIVER_W,
  THERMAL_PHASE_U,       /**< two IAUCN10S7N021, high and low             */
  THERMAL_PHASE_V,
  THERMAL_PHASE_W,
  THERMAL_MCU,           /**< STM32H753 at 475 MHz, through a linear LDO  */
  THERMAL_REGULATORS,    /**< MP4541 x2 and the LDOs after them           */
  THERMAL_AFE,           /**< THS4551 x3 and the reference                */
  THERMAL_BOARD,         /**< the copper everything sinks into            */
  THERMAL_NODES
} thermal_node_t;

/** The leg a node belongs to, for code that walks them three at a time. */
#define THERMAL_DRIVER(leg) ((thermal_node_t)(THERMAL_DRIVER_U + (leg)))
#define THERMAL_PHASE(leg)  ((thermal_node_t)(THERMAL_PHASE_U + (leg)))

/** The NTC sits beside the middle driver, so that is the one it anchors.
  * It used to anchor the lumped node, which made an idle leg's estimate
  * follow a neighbour that was switching. */
#define THERMAL_NTC_NEIGHBOUR THERMAL_DRIVER_V

/** One node's thermal properties. Set from the calibration record. */
typedef struct
{
  /** Junction above the node, kelvin. A die reads the junction, the node is
    * the package. Without it the board came out 6.4 K ABOVE an NTC that sits
    * in the drivers' hot spot - measured 2026-08-28. Zero if no die. */
  float die_over_node;
  float to_board;        /**< K/W from this node into the board node   */
  float capacity;        /**< J/K - what sets how fast it responds     */
} thermal_node_cfg_t;

/** What the thermal observer needs to know about the board, once. */
typedef struct
{
  thermal_node_cfg_t node[THERMAL_NODES];
  /** K/W off the board, AT `board_cal_rise_k`. Not a constant.
    *
    * A BOARD LOSES HEAT TO AIR TWO WAYS AND NEITHER IS LINEAR. Free
    * convection carries `h = Nu k/L` with `Nu` a power of the Rayleigh
    * number, so `h` grows as roughly the fourth root of the rise
    * (Ziegenfelder 2022, USU thesis, Eq. 2.4-2.6: `q = h A dT`, with
    * `Gr = (g/nu^2) beta dT P^3`); radiation carries
    * `h_rad = eps sigma (T^2 + T0^2)(T + T0)` (Silva 2022, Eq. 5), which
    * grows faster still. A single K/W is both of them frozen at one
    * temperature.
    *
    * THAT ONE TEMPERATURE WAS 10 K. The figure came from the passive
    * state - 1.2 W over a 10 K rise - and the board was then asked about
    * loads that put sixty kelvin on it, where the same two mechanisms
    * carry far more per kelvin. Held flat it over-predicted: the copper
    * needed 6.00 W to reach 70 C on the old arithmetic and about 8 W on
    * this one.
    *
    * `board_to_ambient_at` is what everything asks now. This stays the
    * value AT the calibration rise, so the measurement is reproduced
    * exactly at its own point and only the shape away from it is the
    * correlations'.
    */
  float board_to_ambient;
  /** The rise `board_to_ambient` was measured at, K. */
  float board_cal_rise_k;
  /** How much of the loss at that rise is radiation, 0 to 1.
    *
    * The two mechanisms have DIFFERENT SHAPES, so the split at the
    * calibration point is what lets them be scaled apart. Not measured
    * here: it is the 30 to 40 % that a compendium of PCBA thermal work
    * gives for passive cooling, which is also why it cannot be dropped -
    * "stralning star for 30-40 % av den totala varmeavledningen vid
    * passiv kylning och kan inte forsummas" (docs/papers). Zero disables
    * the split and leaves convection carrying all of it.
    */
  float board_rad_share;

  /** How much of the drivers' rise the NTC sees. Not capped at 1.
    *
    * 0.0 would be a pure board reading. Above 1 means the NTC rises faster
    * than the node's own surface, which happens when it sits closer to the
    * heat than the point the node stands for. Solved against both camera
    * states 2026-08-28 it is **1.055**, and a cap at 1.0 cost 5.6 K in the
    * switching state.
    */
  /** How much of the leg node the thermistor's own node is tied to, 0 to
    * 1: the steady-state fraction `R_board / (R_leg + R_board)` of the
    * element it sits in.
    *
    * AN ELEMENT NOW, NOT A COEFFICIENT. Silva 2022 (Appl. Sci. 12,
    * 12555) is the form: every thermal object is a resistance and a heat
    * capacitor in parallel, and objects join into a network. The
    * thermistor is one such object, tied to the leg on one side and the
    * board on the other, so its temperature is a WEIGHTED AVERAGE of the
    * two and cannot leave the interval between them whatever this number
    * is. That is the property the old form could not have: it was
    * `board + c x rise + offset` with c fitted at 1.055 and an additive
    * offset on top, so the sensor read hotter than the node heating it
    * at every load - 6.0 K over at rest, 11.5 K at a 100 K rise.
    *
    * NOT MEASURED, AND THE CAMPAIGN CANNOT MEASURE IT. Its one switching
    * state implies 9.6 K of thermistor rise against 9.12 K of leg rise,
    * a fraction of 1.05, which no passive element can have - a body
    * between two others is not hotter than both. Something among the
    * three inputs is wrong: the leg's spreading resistance (itself three
    * times a lumped figure the camera saw once), the driver's share of
    * the switching loss, or the camera's board reference, which reads a
    * mixed copper and soldermask surface through an emissivity nobody
    * corrected. The model can no longer absorb that inconsistency in a
    * coupling, so it shows up as a residual instead, which is where an
    * inconsistency belongs.
    */
  float ntc_sees_drivers;

  /** How slowly the modelled thermistor follows, seconds.
    *
    * NOT A NEW MEASUREMENT - it is the leg node's own RC, `capacity x
    * to_board`, on the argument that a sensor sitting in a lump cannot be
    * quicker than the lump. A thermistor a centimetre off is slower
    * still, so this is a FLOOR on the lag and not a fit: it says the
    * modelled reading may not outrun the copper, which is the sanity the
    * algebra had none of.
    *
    * Zero means no lag and the reading is the algebra outright, which is
    * what it was before this existed.
    */
  float ntc_tau_s;

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
  /** The modelled thermistor reading, LAGGED. Not a node: a state.
    *
    * A THERMISTOR HAS MASS AND THE ALGEBRA DID NOT. `thermal_expected_ntc`
    * used to be a function of the driver node alone, so a modelled sensor
    * followed a small fast lump instantly - 18 W into 0.12 J/K is 150 K a
    * second, and the page showed an NTC doing exactly that. A thermistor a
    * centimetre from the nearest switch node cannot: the heat has to cross
    * copper that has its own mass, and what arrives is low passed.
    *
    * So the algebra is the TARGET and this relaxes toward it at
    * `ntc_tau_s`. Steady state is unchanged - the campaign is reproduced
    * exactly - and the rate is bounded by something physical rather than
    * by a clamp: the fastest it can move is the distance to the target
    * over the constant.
    */
  float ntc;      /**< the element's own temperature, integrated */
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
  /** Mean of the squared phase current since the last estimate, A^2 a leg.
    *
    * CONDUCTION IS A MEAN SQUARE AND A SAMPLE IS NOT ONE. `phase_amps` is
    * one instant, and one instant of a rotating three-phase current says
    * where the vector is pointing, not how big it has been: squared, it
    * ranges from zero to twice the true loss depending only on where in
    * the electrical period the sample landed. Worse on this board than
    * on most, because the sampler is SYNCHRONOUS - the trigger is a tick
    * inside the PWM period - so the alias can lock to one electrical
    * angle and stay there, and a leg carrying its peak can read as a leg
    * carrying nothing for as long as the speed holds.
    *
    * Zero or negative means "not measured", and the estimator falls back
    * to squaring `phase_amps` - which is what it did before this existed
    * and what a caller with only a sample can still ask for.
    *
    * The signed sample is still what the link current is estimated from:
    * a mean square has no sign, and `duty * a` needs one.
    */
  float phase_sq[3];
  float duty[3];         /**< 0..1 per leg, for the conduction split      */
  float link_volts;      /**< DC link, for the switching terms            */
  float link_amps;       /**< into the board. <0 = estimate from phases   */
  bool  switching;       /**< TIM1 driving the gates                      */
  bool  afe_on;          /**< AFE_ON high: the AFE draws, drivers do not  */
} thermal_load_t;

/** Resistances the estimator needs. Ohms, from electronics/ and the models. */
typedef struct
{
  float rds_on;          /**< one FET at 25 C, IAUCN10S7N021 = 1.8 mOhm   */
  float rds_alpha;       /**< its tempco, per K - rds_on*(1+a*(Tj-25))    */
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
  *
  * `phase_c` is the three phase nodes' current temperatures - the
  * observer's own estimate, fed back so the FET's on-resistance rises
  * with the junction it models: a 100 V Si FET conducts at ~1.6x its
  * 25 C figure at 100 C, which under-estimated exactly where margins
  * thin. NULL, or a NaN entry, keeps that leg at the 25 C figure.
  */
void thermal_power_estimate(thermal_power_t *out, const thermal_load_t *load,
                            const thermal_loss_t *loss,
                            const float *phase_c);

/** What each node may reach. Not the board's opinion: the ceilings live in
  * the calibration record (invariant 7). The board holds a limit it was
  * given, and acts on it - throttle, then trip - without judging a reading.
  */
typedef struct
{
  float limit_c[THERMAL_NODES];  /**< absolute ceiling per node, degrees C */
  float throttle_at;             /**< fraction of budget where derating starts */
  /** The reaction window the throttle keeps, seconds.
    *
    * DERATING ON HOW LONG A NODE HAS, not on where it is. The record
    * already said why: "a deep burst moves a node in seconds, so a
    * throttle that waits for the ceiling arrives after it." It arrives
    * after it on the throttle POINT too - measured on the stand-in, a
    * phase node at 45 A crossed from a fifth of its budget to over the
    * ceiling inside three polls, so the whole 85-to-100 band went past
    * between two looks and the derate never left 1.0.
    *
    * Each node's HOLD - its soak over the power spending it, the same
    * seconds `millis_to_limit` reports - is measured against this
    * window, and the fraction `1 - hold/window` joins the temperature
    * fraction. The derate takes whichever is worse.
    *
    * IT WAS A DISTANCE TO PROJECT A TEMPERATURE, forward this many
    * seconds at the present rate, and that shape forbade the transient
    * this board exists to make. Measured 2026-09-03 in
    * `test_thermal_core.py`: 100 A in one leg puts 18.4 W into a driver
    * node of 0.12 J/K, which is 0.67 s from ambient to a 125 C ceiling,
    * so a two second projection landed past the ceiling from a COLD
    * board and the clamp went to 0.00 before the burst began.
    *
    * Time does not do that. A node at ambient has its whole soak in
    * front of it however much power is on it, so the burst runs; what
    * closes the clamp is the hold falling into the window. It is also
    * scale-free across the nodes - a part with twice the power has half
    * the hold and derates twice as early in degrees, which is right,
    * because it has half the time to act - and the knob behaves: a
    * longer window is an earlier, gentler ramp rather than a stage that
    * will not start.
    *
    * A power so large the node cannot hold it for the window at all is
    * throttled from ambient. That is the rule working: a current a part
    * cannot survive the reaction to is not a burst.
    *
    * Zero disables it and leaves the throttle looking only at the
    * present, which is what it did before this existed. */
  float lookahead_s;
  /** Nodes the current clamp cannot cool. `false` for all of them is the
    * old behaviour, which is why the flag reads this way round: a caller
    * that zeroes this struct gets the envelope it had.
    *
    * A THROTTLE IS A CONTROL LOOP AND IT NEEDS AN ACTUATOR. The clamp
    * scales the phase current, so it moves the conduction and switching
    * loss in the drivers, the FETs and the shunts, and it moves NOTHING
    * on the MCU, the regulators or the front end - those draw the same
    * watts at zero duty as at full. Weighed into `worst`, they set a
    * floor under the margin that no derating can lift: measured on the
    * stand-in 2026-09-04, an idle board with nothing switching settles
    * with the regulators at 51.1 C and the MCU at 49.1 C, which against
    * a 125 C ceiling from a 20 C ambient is 0.30 of the budget spent
    * before the stage has done any work at all. The page showed a third
    * of the board's SOA gone on a cold bench, and the two thirds left
    * were the only part that ever moved.
    *
    * They are still judged: `used` and `soak_j` are filled for every
    * node and `tripped` spans all of them, because an MCU at its ceiling
    * is a stop whatever caused it. What changes is that they no longer
    * ask for a throttle that cannot answer.
    *
    * Which nodes these are is the CALIBRATION RECORD'S to say, like the
    * ceilings beside them - invariant 7. */
  bool undriven[THERMAL_NODES];
} thermal_soa_t;

/** What is spent of the thermal budget, and how long is left.
  *
  * `used` is one byte a node: 0 at ambient, 255 at the limit - a temperature
  * cannot say how close without the ceiling beside it.
  *
  * `millis_to_limit` is the dead reckoning a burst plans on; negative means
  * it is not heading there. Milliseconds because 35 W into the phase node
  * crosses the throttle point with well under a second left.
  */
typedef struct
{
  uint8_t used[THERMAL_NODES];
  /** The largest of `used` AMONG THE NODES THE CLAMP REACHES - see
    * `thermal_soa_t::undriven`. `tripped` still spans every node. */
  uint8_t worst;
  uint8_t worst_node;        /**< which one it was                        */
  int32_t millis_to_limit;   /**< for `worst_node`; -1 = not heading there */
  bool    throttling;        /**< past throttle_at: derate now            */
  bool    tripped;           /**< ANY node at or past a limit: stop       */
  /** What a current clamp should be multiplied by, 1.0 down to 0.0.
    *
    * ONE AT THE THROTTLE POINT AND ZERO AT THE CEILING, linear between.
    * A stage that runs at full current until the ceiling and then stops
    * is a cliff, and a cliff is what `tripped` alone made this: the
    * envelope computed a throttle band nothing acted on. Derating the
    * CLAMP rather than the duty keeps the current loop in charge of its
    * own limit - a duty ceiling applied behind its back is a
    * disturbance it cannot explain.
    *
    * Still not a verdict on a reading: the factor is arithmetic on the
    * ceilings the calibration record gave, and what uses it decides
    * what to do with it. */
  float   derate;
  /** How much energy each node can still absorb before its ceiling,
    * joules: `capacity * (limit - t)`.
    *
    * THE BUDGET A HOST CAN PLAN WITH. `used` is where a node is and
    * `millis_to_limit` is how long at THIS power; neither answers "how
    * much work is left in it", which is what a control system asking
    * for a burst actually wants. Joules do, and they are the honest
    * thermodynamic quantity: capacity times the temperature rise still
    * available. Divide by a planned power to get seconds, at any power
    * rather than only the present one. */
  float   soak_j[THERMAL_NODES];
} thermal_budget_t;

/* No thermal_soa_defaults here on purpose. The envelope lives in the
   board's calibration record - one definition, and one that travels with the
   board rather than with the firmware. A copy here would be a second answer
   to "what may this node reach", and the two would drift. */

/**
  * @brief  K/W off the board at a given rise over ambient.
  *
  * The calibration value scaled by how much better the two mechanisms
  * carry heat at this rise than at the one it was measured at. Below the
  * calibration rise it is not extrapolated downward - a fourth root has
  * no useful behaviour at zero, and a board that cool is losing nothing
  * anybody is waiting for.
  */
float thermal_board_to_ambient_at(const thermal_cfg_t *cfg, float rise_k);


/**
  * @brief  Spend of the thermal budget, and the time left at this power.
  *
  * Pure. Acting on it belongs where the acting belongs.
  */
void thermal_budget(const thermal_t *th, const thermal_power_t *p,
                    const thermal_soa_t *soa, thermal_budget_t *out);

/**
  * @brief  Start the thermal observer with every node at one temperature.
  * @param  cfg      Network parameters. Copied.
  * @param  celsius  What to assume everything is at - a first reading.
  */
void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius);

/** What the thermometers say now. NAN for any that is not answering.
  *
  * A die sensor measures its NODE, not the board. TSEN was written off as a
  * board thermometer for good reason - measured 2026-08-28 it FELL 1.88 K
  * during a run that warmed the board - but that self-heating is the signal
  * for the node it sits on. One thermistor against five estimated sources is
  * badly under-observed; each die is another equation, already on the wire.
  */
typedef struct
{
  float ntc_c;   /**< the thermistor, beside the middle gate driver */
  float afe_c;   /**< the A1335's own die, out in the AFE corner    */
  float mcu_c;   /**< the MCU's own die                             */
} thermal_sense_t;

/**
  * @brief  Advance the model and pull it toward what the sensors say.
  * @param  p     Dissipation now, per node.
  * @param  seen  The thermometers. Any of them may be NAN.
  * @param  dt_s  Seconds since the last step.
  *
  * Open loop between readings, so a silent sensor degrades the estimate
  * rather than stopping it.
  *
  * A die is worth more than its own node: the node's rise over the board is
  * its power times its spreading resistance, both of which the model has, so
  * subtracting them reaches the board without passing through the drivers'
  * hot spot. With none answering the NTC carries both and `settled` is false.
  */
void thermal_step(thermal_t *th, const thermal_power_t *p,
                  const thermal_sense_t *seen, float dt_s);

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
  * @brief  Change one node's parameters while the thermal observer runs.
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

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_H */
