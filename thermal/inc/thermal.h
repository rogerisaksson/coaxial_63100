/**
  ******************************************************************************
  * @file    thermal.h
  * @brief   Lumped-network thermal observer: what each region of the board,
  *          and of the motor behind it, is at.
  *
  * Not a finite element model. A mesh and a solver do not fit in a main loop
  * and are not needed to answer "how hot is the gate driver" - twenty nodes
  * and the conductances between them do, and the twenty follow the COPPER
  * rather than the schematic.
  *
  * THE GRAPH, since 2026-09-05. It was a star: every source into one board
  * node, the board into ambient - the coarsest topology that could tell a
  * hot part from a hot board. Judged against the lumped-network class the
  * papers in docs/papers put at about ten percent when the nodes follow the
  * geometry, it was a six: one node for a disc with a seventeen kelvin
  * gradient across it in the camera's switching state, six leg nodes that
  * could not warm each other except through that average, the switching
  * loss a point measurement scaled with voltage alone, the junction a
  * constant over its package, and still room air for a board that sits
  * behind a stator. Each of those is a coarser graph than the physics, not
  * a wrong equation, so each is closed by adding to the graph:
  *
  *     driver_u --+                              +-- winding
  *     phase_u  --+-- patch_u --+--- stator ------+
  *     driver_v --+             |      |          +-- rotor ---- air
  *     phase_v  --+-- patch_v --+   (mount, radiation)
  *     driver_w --+             |
  *     phase_w  --+-- patch_w --+
  *     regulators --- patch_left --+--- board (centre) --- patch_right --- hotswap
  *     mcu ------------------------+        |
  *     afe ----------------------- patch_bottom
  *
  *   * SEVEN LAMINATE PATCHES where there was one board: the centre, one
  *     under each leg's switches and shunts, the regulators' corner, the
  *     front end's edge, the hot swap's. Their areas come off the outline
  *     and the pick and place (the same partition the thermal picture
  *     draws), their in-plane conductances off a sheet conductance times
  *     shared boundary over centre distance - the copper's graph.
  *   * THE LEGS WARM EACH OTHER through the patches beside them, and the
  *     thermistor sits in the centre patch a driver's width from the V leg's.
  *   * THE HOT SWAP IS A NODE: 35 W in its FETs and fuse at 100 A had been
  *     booked on the regulators.
  *   * THE MOTOR IS THE BOARD'S BOUNDARY: the winding, the stator's iron
  *     and the rotor's bell, coupled to the rim patches through the mount
  *     and to their faces by radiation, and cooled by air the rotor moves -
  *     forced convection with speed. On the bench the mount is open and the
  *     motor is three nodes at the room, which is what a bench is.
  *   * LOSSES AS FUNCTIONS: the switching energy per event from the C_oss
  *     law, the overlap from the gate charge and the drive current, the
  *     body diode over the dead time, the gate charge - each with a
  *     physical name - beside the conduction that was already right.
  *   * JUNCTIONS as `R_th,JC` times the part's own power, not a constant.
  *
  * Nine remains the papers' own numerical methods, which do not belong in
  * a 10 Hz loop on the target. Twenty nodes and thirty edges are a few
  * hundred floats and a few hundred flops a step on an M7 at 475 MHz.
  *
  * EVERY NUMBER IN `thermal_defaults` HAS A DERIVATION AND MOST HAVE NO
  * MEASUREMENT. The four camera states of 2026-08-28 anchor the bulk and
  * the zones; the rest is geometry, datasheets and stated estimates, which
  * is why the record can overlay any of them and why an online
  * identification (board_thermal.c) exists to replace them with what the
  * board's own sensors say.
  *
  * TWO SENSORS ON THE BOARD, ONE OF THEM IN THE HOT SPOT. The NTC sits in
  * the centre patch beside a gate driver; TSEN is the MCU's own die; the
  * A1335's die is in the front end's corner. Each corrects the node it is on
  * and implies the patch under it. Measured 2026-08-28: gate off, NTC - TSEN
  * = -0.74 C; three legs switching, +10.94 C - the NTC overstates a board
  * average by 2.48x, which is why it is a hot-spot sensor here and not a
  * board thermometer.
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

/** The nodes. Order is the wire order; append only.
  *
  * The first ten are the star's, unchanged in meaning so a host on an
  * older codec reads them as it did: PER LEG - a leg that is not switching
  * does not get warm - and `board` the laminate's CENTRE patch, which is
  * what the old bulk node most nearly was (the bore, the MCU and the
  * thermistor sit in it). The ten after are the graph's.
  */
typedef enum
{
  THERMAL_DRIVER_U = 0,  /**< one 2EDL8034 and its two FETs' silicon    */
  THERMAL_DRIVER_V,      /**< V is the NTC's neighbour                   */
  THERMAL_DRIVER_W,
  THERMAL_PHASE_U,       /**< the leg's two WSHM2818 shunts              */
  THERMAL_PHASE_V,
  THERMAL_PHASE_W,
  THERMAL_MCU,           /**< STM32H753 at 475 MHz, through a linear LDO */
  THERMAL_REGULATORS,    /**< MP4541 x2 and the LDOs after them          */
  THERMAL_AFE,           /**< THS4551 x3 and the reference               */
  THERMAL_BOARD,         /**< the laminate's centre patch                */
  THERMAL_HOTSWAP,       /**< LM5069, its back-to-back FETs, the fuse    */
  THERMAL_PATCH_U,       /**< the laminate under U's switches and shunts */
  THERMAL_PATCH_V,
  THERMAL_PATCH_W,
  THERMAL_PATCH_LEFT,    /**< under the regulators                       */
  THERMAL_PATCH_BOTTOM,  /**< under the front end                        */
  THERMAL_PATCH_RIGHT,   /**< under the hot swap                         */
  THERMAL_WINDING,       /**< the stator's copper                        */
  THERMAL_STATOR,        /**< its iron and the motor's body              */
  THERMAL_ROTOR,         /**< the outrunner's bell and magnets           */
  THERMAL_NODES
} thermal_node_t;

/** The leg a node belongs to, for code that walks them three at a time. */
#define THERMAL_DRIVER(leg) ((thermal_node_t)(THERMAL_DRIVER_U + (leg)))
#define THERMAL_PHASE(leg)  ((thermal_node_t)(THERMAL_PHASE_U + (leg)))
#define THERMAL_PATCH(leg)  ((thermal_node_t)(THERMAL_PATCH_U + (leg)))

/** The thermistor's element sits in the centre patch, a driver's width
  * from the V leg's patch: it is tied between those two, and it is the V
  * leg's patch it corrects. */
#define THERMAL_NTC_PATCH   THERMAL_PATCH_V
#define THERMAL_NTC_NEIGHBOUR THERMAL_NTC_PATCH

/** The edges: which two nodes each conductance joins. A fixed table, so an
  * edge has a number the record and the wire can name; the K/W across it
  * is the record's (`thermal_cfg_t::r_edge`). Zero K/W is an open edge. */
#define THERMAL_EDGES 30

typedef struct
{
  uint8_t a;
  uint8_t b;
} thermal_edge_t;

extern const thermal_edge_t THERMAL_EDGE_ENDS[THERMAL_EDGES];

/** The edge each node sheds through first - a source into its patch, the
  * winding into the stator, the stator into the rotor - so a caller with
  * one number for a node has somewhere to put it. -1 for a node whose
  * first path is the air (`to_ambient`). */
int thermal_sink_edge(thermal_node_t node);

/** One node's thermal properties. Set from the calibration record. */
typedef struct
{
  float capacity;        /**< J/K - what sets how fast it responds. 0 = off */
  /** K/W to the air AT `board_cal_rise_k`, or zero for a node whose only
    * paths are edges. A patch's is the bulk's `board_to_ambient` divided
    * by its share of the face; the rotor's and stator's are their own. */
  float to_ambient;
  /** The node's share of the board's face, 0..1, for the patches: what
    * scales its convection, its radiation and its capacity out of the
    * bulk's measured figures. Zero for everything that is not laminate. */
  float area_share;
  /** Junction over node per watt in the part, K/W: `R_th,JC` for the
    * FETs (0.69, datasheets/mosfet), the die's own for the two that
    * report one. The die is `node + P * rth_die`. Zero if no die. */
  float rth_die;
  /** How much better the node's air path carries per sqrt(krpm) of
    * rotor speed - forced convection, `Nu ~ Re^1/2` over a plate. Zero
    * for a node the rotor's air does not reach. */
  float forced;
} thermal_node_cfg_t;

/** What the thermal observer needs to know about the board, once. */
typedef struct
{
  thermal_node_cfg_t node[THERMAL_NODES];
  /** K/W across each edge of THERMAL_EDGE_ENDS. Zero opens it. */
  float r_edge[THERMAL_EDGES];

  /** K/W off the whole board, AT `board_cal_rise_k`. Not a constant.
    *
    * A BOARD LOSES HEAT TO AIR TWO WAYS AND NEITHER IS LINEAR. Free
    * convection carries `h = Nu k/L` with `Nu` a power of the Rayleigh
    * number, so `h` grows as roughly the fourth root of the rise
    * (Ziegenfelder 2022, USU thesis, Eq. 2.4-2.6); radiation carries
    * `h_rad = eps sigma (T^2 + T0^2)(T + T0)` (Silva 2022, Eq. 5), which
    * grows faster still. This stays the value AT the calibration rise, so
    * the measurement is reproduced exactly at its own point and only the
    * shape away from it is the correlations'. Each patch carries its
    * share of it by area. */
  float board_to_ambient;
  float board_cal_rise_k;   /**< the rise it was measured at, K */
  /** How much of the loss at that rise is radiation, 0 to 1. The two
    * mechanisms have DIFFERENT SHAPES, so the split at the calibration
    * point is what lets them be scaled apart. The 30 to 40 % a compendium
    * of PCBA thermal work gives for passive cooling (docs/papers). */
  float board_rad_share;

  /** How far the thermistor's element sits toward the V leg's patch from
    * the centre patch, 0 to 1: the steady-state fraction of the element
    * it sits in. A weighted average of the two, so it cannot leave the
    * interval between them whatever this number is (Silva 2022: every
    * thermal object a resistance and a capacitor, joined). Geometry: the
    * part is 8 mm from the V driver and in the centre patch. */
  float ntc_sees;
  /** How slowly the modelled thermistor follows, seconds - the laminate
    * around it, which has no node of its own. Zero: no lag. */
  float ntc_tau_s;
  /** The NTC's disagreement with the camera in the passive state, K.
    * Recorded, not applied: an instrument disagreement, not a temperature. */
  float ntc_offset;

  /** Radiation between the board's face and the stator's back, W/K at
    * 300 K for the whole face - `eps sigma A F 4 T^3` - scaled by each
    * patch's share and by the two temperatures' bracket. Zero on a bench,
    * where nothing faces the board. */
  float rad_board_stator;
} thermal_cfg_t;

/** Live state. Owned by the caller; `thermal_init` fills it. */
typedef struct
{
  thermal_cfg_t cfg;
  float t[THERMAL_NODES];   /**< degrees C per node          */
  float ambient;            /**< estimated, not measured     */
  /** The modelled thermistor reading, LAGGED - the element's own
    * temperature, integrated toward the weighted average of the two
    * patches it sits between and never past either of them. */
  float ntc;
  bool  settled;            /**< true once a die has anchored a patch */
  uint32_t steps;
  /** The rotor's speed at the last step, rpm: the budget's air paths are
    * evaluated at the same speed the integrator just used. */
  float speed_rpm;
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
    * CONDUCTION IS A MEAN SQUARE AND A SAMPLE IS NOT ONE: one instant of a
    * rotating three-phase current says where the vector is pointing, not
    * how big it has been. Zero or negative means "not measured", and the
    * estimator squares `phase_amps` instead. */
  float phase_sq[3];
  float duty[3];         /**< 0..1 per leg, for the link estimate         */
  float link_volts;      /**< DC link, for the switching terms            */
  float link_amps;       /**< into the board. <0 = estimate from phases   */
  bool  switching;       /**< TIM1 driving the gates                      */
  bool  afe_on;          /**< AFE_ON high: the AFE draws, drivers do not  */
  /** The rotor's mechanical speed, rpm, for the air it moves and the
    * iron it magnetises. Zero at rest, and zero when nothing knows. */
  float speed_rpm;
  /** The dead time between a leg's two gates, seconds - the record's
    * `deadtime_ns` - for the body diode's conduction across it. */
  float t_dead_s;
} thermal_load_t;

/** Resistances, charges and times the estimator needs. From electronics/,
  * the datasheets in datasheets/ and the models in electronic_simulations. */
typedef struct
{
  float rds_on;          /**< one FET at 25 C, IAUCN10S7N021 = 1.8 mOhm   */
  float rds_alpha;       /**< its tempco, per K - rds_on*(1+a*(Tj-25))    */
  float r_shunt;         /**< phase shunt, RU1||RU2 = 3.5 mOhm            */
  float r_hotswap;       /**< LM5069 pass FETs, in the link               */
  float switching_watt;  /**< the no-load switching loss at `switch_volts` */
  float switch_volts;    /**< the link it was measured at                 */
  float driver_share;    /**< how much of it lands in the driver zone     */
  float mcu_watt;        /**< static, 475 MHz through the linear LDO      */
  float ldo_watt;        /**< the drop, plus what else the reg zone makes */
  float afe_watt;        /**< the AFE chain when AFE_ON is high           */
  /* Since 2026-09-05: the switching loss as functions of what switches. */
  float f_sw;            /**< the PWM, Hz - TIM1 at 50 kHz                 */
  float coss_cjo;        /**< C_oss at 0 V, F, and its law: C = CJO/(1+V/VJ)^M */
  float coss_m;
  float coss_vj;
  float t_switch_s;      /**< current-voltage overlap per period, on + off */
  float v_sd;            /**< the body diode's drop, V                     */
  float q_g;             /**< total gate charge, C, one FET                */
  float v_drive;         /**< what the gates are driven to, V              */
  float buck_eff;        /**< the +15V7 buck's efficiency, for its loss    */
  float r_phase;         /**< the winding, line to neutral: the record's   */
  float k_iron;          /**< stator iron loss, W per (krpm)^2; 0 unknown  */
} thermal_loss_t;

/** The loss constants as measured/traced on this board. */
void thermal_losses(thermal_loss_t *loss);

/** The energy C_oss stores at `volts` under the `loss`'s law, joules, for
  * ONE FET: the integral of `v C(v) dv`, closed form. What a hard-switched
  * leg dumps once a period per FET. */
float thermal_coss_energy(const thermal_loss_t *loss, float volts);

/**
  * @brief  Dissipation per node from what the board is doing.
  *
  * Conduction is the mean square through the FET (its resistance at the
  * node's own temperature) and through its shunt, each on its own node.
  * Switching, per driven leg: the no-load figure measured at `switch_volts`
  * scaled by the C_oss energy's own law rather than linearly; the overlap
  * `V I t_switch f`; the body diode `2 V_sd I t_dead f`; the gate charge
  * `2 Q_g V_drive f` - the FET terms on the driver node, the buck's share
  * on the regulators. The hot swap sees the link current squared through
  * its FETs. The winding sees the three mean squares through `r_phase`;
  * the stator its iron loss with speed. Housekeeping is static.
  *
  * `link_amps` below zero is estimated as sum(duty * phase_amps).
  * `phase_c` is the three driver nodes' temperatures, fed back so the FET's
  * on-resistance rises with the junction it models; NULL keeps 25 C.
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
  /** The reaction window the throttle keeps, seconds. Each node's HOLD -
    * its soak over the power spending it - is measured against this, and
    * `1 - hold/window` joins the temperature fraction; the derate takes
    * whichever is worse. Time, not a projected temperature: a node at
    * ambient has its whole soak in front of it however much power is on
    * it, so a burst runs, and what closes the clamp is the hold falling
    * into the window. Zero disables it. */
  float lookahead_s;
  /** Nodes the current clamp cannot cool - the housekeeping - judged but
    * not throttled on. A clamp on the phase current is a loop with no
    * actuator for them, and weighed in they floor the margin. The
    * record's to say (invariant 7). */
  bool undriven[THERMAL_NODES];
} thermal_soa_t;

/** What is spent of the thermal budget, and how long is left. */
typedef struct
{
  uint8_t used[THERMAL_NODES];   /**< 0 at ambient, 255 at the limit      */
  uint8_t worst;                 /**< among the nodes the clamp reaches   */
  uint8_t worst_node;
  int32_t millis_to_limit;       /**< for `worst_node`; -1 = not heading there */
  bool    throttling;
  bool    tripped;               /**< ANY node at or past a limit: stop   */
  /** What a current clamp should be multiplied by, 1.0 down to 0.0: one
    * at the throttle point and zero at the ceiling, linear between, on
    * the worse of where a node is and how long it has. */
  float   derate;
  float   soak_j[THERMAL_NODES]; /**< `capacity * (limit - t)`, never negative */
} thermal_budget_t;

/** One node's own clamp factor - the same ramp the whole budget uses, on
  * this node's spend and hold alone - so a host can say which node holds
  * the stage back. The winding's is what the wire reports beside the
  * whole. */
float thermal_node_derate(const thermal_t *th, const thermal_power_t *p,
                          const thermal_soa_t *soa, thermal_node_t node);

/** The junction of the part on `node`, degrees C: the node plus its power
  * through `rth_die`. For a leg, `p` is the two FETs' and each carries
  * half. What the datasheet's 175 C is against. */
float thermal_junction(const thermal_t *th, const thermal_power_t *p,
                       thermal_node_t node);

/** K/W off a node's air path at a rise over ambient and a rotor speed:
  * the calibration value scaled by how much better convection and
  * radiation carry at this rise (a fourth root, and the bracket), and by
  * the air the rotor moves. */
float thermal_to_ambient_at(const thermal_cfg_t *cfg, thermal_node_t node,
                            float rise_k, float speed_rpm);

/** The bulk's figure, for the ambient estimate and the host's arithmetic:
  * the whole face at this rise, still air. */
float thermal_board_to_ambient_at(const thermal_cfg_t *cfg, float rise_k);

/** Spend of the thermal budget, and the time left at this power. Pure. */
void thermal_budget(const thermal_t *th, const thermal_power_t *p,
                    const thermal_soa_t *soa, thermal_budget_t *out);

/** Start the thermal observer with every node at one temperature. `cfg`
  * NULL takes the defaults. */
void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius);

/** What the thermometers say now. NAN for any that is not answering. */
typedef struct
{
  float ntc_c;   /**< the thermistor, beside the middle gate driver */
  float afe_c;   /**< the A1335's own die, out in the AFE corner    */
  float mcu_c;   /**< the MCU's own die                             */
} thermal_sense_t;

/**
  * @brief  Advance the network one step and pull it toward the sensors.
  * @param  p     Dissipation now, per node.
  * @param  seen  The thermometers. Any of them may be NAN.
  * @param  load  What the board is doing - the speed, for the air.
  * @param  dt_s  Seconds since the last step.
  *
  * Explicit Euler over every edge and every air path, then the anchors:
  * each die corrects its node and implies the patch under it - the node is
  * patch + P * R into it, both of which the model has - and the thermistor
  * corrects the V leg's patch through the element it sits in. Ambient is
  * what the patches' own losses imply, once a die has anchored one.
  */
void thermal_step(thermal_t *th, const thermal_power_t *p,
                  const thermal_sense_t *seen, const thermal_load_t *load,
                  float dt_s);

/** What the NTC should read, given the model - the lagged element. */
float thermal_expected_ntc(const thermal_t *th);

/** The centre patch from the NTC with the V leg's share taken out. */
float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float patch_rise_k);

/** Defaults: the network as derived for this board. Every entry names its
  * source in thermal.c; the record overlays any of them. */
void thermal_defaults(thermal_cfg_t *cfg);

/**
  * @brief  Change one node's sink and capacity while the observer runs.
  * @return False for an unknown node or a non-positive value.
  *
  * `k_per_w` is the node's first path out - its edge into its patch, or
  * its air path where that is the first (`thermal_sink_edge`).
  */
bool thermal_set_node(thermal_t *th, thermal_node_t node,
                      float k_per_w, float capacity);

/** Change one edge's K/W. False for an unknown edge or a non-positive value. */
bool thermal_set_edge(thermal_t *th, int edge, float k_per_w);

/** Change the bulk's two numbers: K/W off the whole face at the
  * calibration rise, and the laminate's J/K, shared out by area. */
bool thermal_set_board(thermal_t *th, float to_ambient, float capacity);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_H */
