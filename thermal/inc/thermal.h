/** thermal.h - Lumped-network thermal observer: what each region of the
    board, and of the motor behind it, is at. */
#ifndef THERMAL_H
#define THERMAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The nodes. */
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

/** The thermistor's element sits in the centre patch, a driver's width from
    the V leg's patch: it is tied between those two, and it is the V leg's
    patch it corrects. */
#define THERMAL_NTC_PATCH   THERMAL_PATCH_V
#define THERMAL_NTC_NEIGHBOUR THERMAL_NTC_PATCH

/** The edges: which two nodes each conductance joins. */
#define THERMAL_EDGES 30

typedef struct
{
  uint8_t a;
  uint8_t b;
} thermal_edge_t;

/** Which two nodes edge `e` joins. */
thermal_edge_t thermal_edge(int e);

/** The edge each node sheds through first - a source into its patch, the
    winding into the stator, the stator into the rotor - so a caller with one
    number for a node has somewhere to put it. */
int thermal_sink_edge(thermal_node_t node);

/** One node's thermal properties. Set from the calibration record. */
typedef struct
{
  float capacity;        /**< J/K - what sets how fast it responds. 0 = off */
  /** K/W to the air AT `board_cal_rise_k`, or zero for a node whose only
      paths are edges. */
  float to_ambient;
  /** The node's share of the board's face, 0..1, for the patches: what
      scales its convection, its radiation and its capacity out of the bulk's
      measured figures. */
  float area_share;
  /** Junction over node per watt in the part, K/W: `R_th,JC` for the FETs
      (0.69, datasheets/mosfet), the die's own for the two that report one. */
  float rth_die;
  /** How much better the node's air path carries per sqrt(krpm) of rotor
      speed - forced convection, `Nu ~ Re^1/2` over a plate. */
  float forced;
} thermal_node_cfg_t;

/** What the thermal observer needs to know about the board, once. */
typedef struct
{
  thermal_node_cfg_t node[THERMAL_NODES];
  /** K/W across each edge of THERMAL_EDGE_ENDS. Zero opens it. */
  float r_edge[THERMAL_EDGES];

  /** K/W off the whole board, AT `board_cal_rise_k`. */
  float board_to_ambient;
  float board_cal_rise_k;   /**< the rise it was measured at, K */
  /** How much of the loss at that rise is radiation, 0 to 1. */
  float board_rad_share;

  /** How far the thermistor's element sits toward the V leg's patch from the
      centre patch, 0 to 1: the steady-state fraction of the element it sits
      in. */
  float ntc_sees;
  /** How slowly the modelled thermistor follows, seconds - the laminate
      around it, which has no node of its own. */
  float ntc_tau_s;
  /** The NTC's disagreement with the camera in the passive state, K. */
  float ntc_offset;

  /** Radiation between the board's face and the stator's back, W/K at 300 K
      for the whole face - `eps sigma A F 4 T^3` - scaled by each patch's
      share and by the two temperatures' bracket. */
  float rad_board_stator;
} thermal_cfg_t;

/** Live state. Owned by the caller; `thermal_init` fills it. */
typedef struct
{
  thermal_cfg_t cfg;
  float t[THERMAL_NODES];   /**< degrees C per node          */
  float ambient;            /**< estimated, not measured     */
  /** The modelled thermistor reading, LAGGED - the element's own
      temperature, integrated toward the weighted average of the two patches
      it sits between and never past either of them. */
  float ntc;
  bool  settled;            /**< true once a die has anchored a patch */
  uint32_t steps;
  /** The rotor's speed at the last step, rpm: the budget's air paths are
      evaluated at the same speed the integrator just used. */
  float speed_rpm;
  /** Seconds since a thermometer last anchored the state. */
  float since_seen_s;
} thermal_t;

/** Dissipation per node, watts. Whoever knows the board's state fills it. */
typedef struct
{
  float watt[THERMAL_NODES];
} thermal_power_t;

/** What the board is doing now, as measured. */
typedef struct
{
  float phase_amps[3];   /**< per leg, signed, as the shunts measure it   */
  /** Mean of the squared phase current since the last estimate, A^2 a leg. */
  float phase_sq[3];
  float duty[3];         /**< 0..1 per leg, for the link estimate         */
  float link_volts;      /**< DC link, for the switching terms            */
  float link_amps;       /**< into the board. <0 = estimate from phases   */
  bool  switching;       /**< TIM1 driving the gates                      */
  bool  afe_on;          /**< AFE_ON high: the AFE draws, drivers do not  */
  /** The rotor's mechanical speed, rpm, for the air it moves and the iron it
      magnetises. */
  float speed_rpm;
  /** The dead time between a leg's two gates, seconds - the record's
      `deadtime_ns` - for the body diode's conduction across it. */
  float t_dead_s;
} thermal_load_t;

/** Resistances, charges and times the estimator needs. */
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
    ONE FET: the integral of `v C(v) dv`, closed form. */
float thermal_coss_energy(const thermal_loss_t *loss, float volts);

/** Dissipation per node from what the board is doing. */
void thermal_power_estimate(thermal_power_t *out, const thermal_load_t *load,
                            const thermal_loss_t *loss,
                            const float *phase_c);

/** What each node may reach. */
typedef struct
{
  float limit_c[THERMAL_NODES];  /**< absolute ceiling per node, degrees C */
  float throttle_at;             /**< fraction of budget where derating starts */
  /** The reaction window the throttle keeps, seconds. */
  float lookahead_s;
  /** Nodes the current clamp cannot cool - the housekeeping - judged but not
      throttled on. */
  bool undriven[THERMAL_NODES];
  /** THE CEILING A TRIP IS JUDGED ON: the record's, untrimmed. */
  float trip_c[THERMAL_NODES];
} thermal_soa_t;

/** What is spent of the thermal budget, and how long is left. */
typedef struct
{
  uint8_t used[THERMAL_NODES];   /**< 0 at ambient, 255 at the limit      */
  uint8_t worst;                 /**< among the nodes the clamp reaches   */
  uint8_t worst_node;
  int32_t millis_to_limit;       /**< for `worst_node`; -1 = not heading there */
  bool    throttling;
  bool    tripped;               /**< ANY node at the record's ceiling (`trip_c`): stop */
  /** What a current clamp should be multiplied by, 1.0 down to 0.0: one at
      the throttle point and zero at the ceiling, linear between, on the
      worse of where a node is and how long it has. */
  float   derate;
  float   soak_j[THERMAL_NODES]; /**< `capacity * (limit - t)`, never negative */
} thermal_budget_t;

/** One node's own clamp factor - the same ramp the whole budget uses, on
    this node's spend and hold alone - so a host can say which node holds the
    stage back. */
float thermal_node_derate(const thermal_t *th, const thermal_power_t *p,
                          const thermal_soa_t *soa, thermal_node_t node);

/** The junction of the part on `node`, degrees C: the node plus its power
    through `rth_die`. */
float thermal_junction(const thermal_t *th, const thermal_power_t *p,
                       thermal_node_t node);

/** K/W off a node's air path at a rise over ambient and a rotor speed: the
    calibration value scaled by how much better convection and radiation
    carry at this rise (a fourth root, and the bracket), and by the air the
    rotor moves. */
float thermal_to_ambient_at(const thermal_cfg_t *cfg, thermal_node_t node,
                            float rise_k, float speed_rpm);

/** The bulk's figure, for the ambient estimate and the host's arithmetic:
    the whole face at this rise, still air. */
float thermal_board_to_ambient_at(const thermal_cfg_t *cfg, float rise_k);

/** Spend of the thermal budget, and the time left at this power. Pure. */
void thermal_budget(const thermal_t *th, const thermal_power_t *p,
                    const thermal_soa_t *soa, thermal_budget_t *out);

/** Net watts into every node at the present temperatures: what it makes,
    plus what flows in over the edges, less what it sheds to the air. */
void thermal_net_flows(const thermal_t *th, const thermal_power_t *p,
                       float speed_rpm, float *net);

/** One Euler slice over the whole graph, no anchors: the identification's
    shadow steps with this, the observer with `thermal_step`. */
void thermal_integrate(thermal_t *th, const thermal_power_t *p,
                       float speed_rpm, float dt_s);

/** Start the thermal observer with every node at one temperature. */
void thermal_init(thermal_t *th, const thermal_cfg_t *cfg, float celsius);

/** What the thermometers say now. NAN for any that is not answering. */
typedef struct
{
  float ntc_c;   /**< the thermistor, beside the middle gate driver */
  float afe_c;   /**< the A1335's own die, out in the AFE corner    */
  float mcu_c;   /**< the MCU's own die                             */
} thermal_sense_t;

/** Advance the network one step and pull it toward the sensors.
    @param  p     Dissipation now, per node.
    @param  seen  The thermometers. Any of them may be NAN.
    @param  load  What the board is doing - the speed, for the air.
    @param  dt_s  Seconds since the last step. */
void thermal_step(thermal_t *th, const thermal_power_t *p,
                  const thermal_sense_t *seen, const thermal_load_t *load,
                  float dt_s);

/** What the NTC should read, given the model - the lagged element. */
float thermal_expected_ntc(const thermal_t *th);

/** One step of the thermistor's element: first order toward the weighted
    average of the two patches it sits between at `ntc_tau_s`, and never past
    either of them. */
int thermal_ntc_follow(thermal_t *th, float dt_s);

/** The centre patch from the NTC with the V leg's share taken out. */
float thermal_board_from_ntc(const thermal_cfg_t *cfg, float ntc_c,
                             float patch_rise_k);

/** Defaults: the network as derived for this board. */
void thermal_defaults(thermal_cfg_t *cfg);

/** Change one node's sink and capacity while the observer runs.
    @return False for an unknown node or a non-positive value. */
bool thermal_set_node(thermal_t *th, thermal_node_t node,
                      float k_per_w, float capacity);

/** Change one edge's K/W. False for an unknown edge or a non-positive value. */
bool thermal_set_edge(thermal_t *th, int edge, float k_per_w);

/** Change the bulk's two numbers: K/W off the whole face at the calibration
    rise, and the laminate's J/K, shared out by area. */
bool thermal_set_board(thermal_t *th, float to_ambient, float capacity);

#ifdef __cplusplus
}
#endif

#endif /* THERMAL_H */
