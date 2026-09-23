/** board/thermal.h - what the comms stack needs from board_thermal.c; included by board.h. */
#ifndef COMMS_BOARD_THERMAL_H
#define COMMS_BOARD_THERMAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Thermal nodes: 10 board, hot swap, 6 laminate patches, winding, stator, rotor. */
#define BOARD_THERMAL_NODES 20

/** The edges of the network, `thermal.c`'s table: each a K/W the record can
    overlay and the wire can name. */
#define BOARD_THERMAL_EDGES 30

/** The identification's scales on the wire - `thermal_ident.h`'s
    THERMAL_IDENT_RECORD, held to it by a static assert in board_thermal.c. */
#define BOARD_THERMAL_IDENT_SCALES 4

/** The margin floor's default, parts per million of every ceiling's span:
    the bench's 80 % - "keep to 80 % of the SOA when switching starts, with
    the thermal situation unknown". */
#define BOARD_SOA_MARGIN_FLOOR_PPM 800000UL

/** The indices, for a record or a host that has to name one. */
#define BOARD_THERMAL_DRIVER_U     0
#define BOARD_THERMAL_DRIVER_V     1
#define BOARD_THERMAL_DRIVER_W     2
#define BOARD_THERMAL_PHASE_U      3
#define BOARD_THERMAL_PHASE_V      4
#define BOARD_THERMAL_PHASE_W      5
#define BOARD_THERMAL_MCU          6
#define BOARD_THERMAL_REGULATORS   7
#define BOARD_THERMAL_AFE          8
#define BOARD_THERMAL_BOARD        9    /**< the laminate's centre patch */
#define BOARD_THERMAL_HOTSWAP      10
#define BOARD_THERMAL_PATCH_U      11
#define BOARD_THERMAL_PATCH_V      12
#define BOARD_THERMAL_PATCH_W      13
#define BOARD_THERMAL_PATCH_LEFT   14
#define BOARD_THERMAL_PATCH_BOTTOM 15
#define BOARD_THERMAL_PATCH_RIGHT  16
#define BOARD_THERMAL_WINDING      17
#define BOARD_THERMAL_STATOR       18
#define BOARD_THERMAL_ROTOR        19

/** What the thermal observer knows: one measurement, the rest estimates. */
typedef struct
{
  bool    ntc_measured;                        /**< the thermistor answered              */
  int32_t ntc_centidegc;                       /**< MEASURED, valid only above           */
  bool    afe_measured;                        /**< the A1335's die answered             */
  int32_t afe_centidegc;                       /**< MEASURED, valid only above           */
  bool    mcu_measured;                        /**< the MCU's die answered               */
  int32_t mcu_centidegc;                       /**< MEASURED, valid only above           */
  uint32_t seen_ms_ago;                        /**< age of the whole sample              */
  uint32_t steps;                              /**< model integrations since boot        */
  int32_t node_centidegc[BOARD_THERMAL_NODES]; /**< ESTIMATED                            */
  int32_t ambient_centidegc;                   /**< ESTIMATED - there is no sensor       */
  int32_t expected_ntc_centidegc;              /**< the model's own NTC, for the error   */
  uint32_t seconds;                            /**< how long it has run                  */
  bool    settled;                             /**< the anchoring has converged          */
  /** MINOR 13: each leg's FET junction over its node, centi-K - half the
      node's watts through R_th,JC - and the rotor speed the air paths were
      evaluated at. */
  int32_t junction_over_centi[3];
  int32_t speed_rpm;
} board_thermal_t;

/** The online identification beside the observer (`thermal_ident.h`): what
    it believes the network's scales are, how sure, and what the envelope
    keeps in hand for that. */
typedef struct
{
  uint8_t  state;                   /**< thermal_ident_state_t - a word    */
  uint8_t  online_mask;             /**< bit k: scale k is moved by samples */
  float    scale[BOARD_THERMAL_IDENT_SCALES];
  float    sigma[BOARD_THERMAL_IDENT_SCALES];
  float    innovation_k;            /**< filtered prediction error, kelvin  */
  float    margin;                  /**< the envelope's factor now, floor..1*/
  uint32_t updates;                 /**< samples that moved the scales      */
  /** MINOR 15: the room as identified beside the scales, degrees C, and how
      sure - the board has no ambient sensor; this is what the observer's
      `ambient` is set from. */
  float    ambient_c;
  float    ambient_sigma_k;
  /** MINOR 16: the floor the margin rises from, the record's. */
  float    margin_floor;
  /** MINOR 17: the trip cap as it stands - THERMAL_TRIP_MARGIN at a trip,
      recovering at THERMAL_TRIP_RECOVER_PER_S - or one with no trip in hand. */
  float    trip_cap;
} board_thermal_ident_t;

bool Board_ThermalIdent(board_thermal_ident_t *out);

/** Forget what was identified: scales to one, the room where the observer
    has it, UNCERTAIN, the margin at the floor. */
bool Board_ThermalIdentReset(void);

/** One edge of the network: which two nodes, and the K/W across it now. */
bool Board_ThermalEdge(uint8_t edge, uint8_t *a, uint8_t *b, float *k_per_w);

/** Change one edge's K/W, in the observer and in the record's RAM copy;
    negative opens it. */
bool Board_ThermalSetEdge(uint8_t edge, float k_per_w);

/** One node's network entry as the observer runs it. */
bool Board_ThermalNodeCfg(uint8_t node, float *capacity, float *to_ambient,
                          float *area_share, float *rth_die, float *forced);

/** The thermal budget: how much is spent and how long is left. */
typedef struct
{
  uint8_t  used[BOARD_THERMAL_NODES];
  uint8_t  worst;
  uint8_t  worst_node;
  int32_t  millis_to_limit;  /**< -1 when it is not heading for a limit */
  bool     throttling;
  bool     tripped;
  uint32_t trips;            /**< how many times it has stopped the stage */
  /** What the current clamp is being multiplied by right now, 1 to 0. */
  float    derate;
  /** Joules each node can still absorb before its ceiling. */
  float    soak_j[BOARD_THERMAL_NODES];
  /** What the compares actually hold, as a fraction of the period. */
  float    duty[BOARD_PWM_PHASES];
  /** MINOR 12: THE WINDING, the one node that is not on the board. */
  float    winding_c;
  uint8_t  winding_used;
  float    winding_derate;
} board_budget_t;

bool Board_ThermalBudget(board_budget_t *out);

/** Set one node's ceiling, degrees C. Zero disables that node's limit. */
bool Board_ThermalSetLimit(uint8_t node, float limit_c, float throttle_at);

/** The winding's ceiling, degrees C (zero disables), and its K/W and J/K:
    written to the record and taken up by the observer at once. */
bool Board_ThermalSetWinding(float limit_c, float k_per_w, float j_per_k);

void Board_ThermalInit(void);
void Board_ThermalPoll(void);
bool Board_ThermalState(board_thermal_t *out);
bool Board_ThermalSetNode(uint8_t node, float to_board, float capacity);
bool Board_ThermalSetBoard(float to_ambient, float capacity);

/** How often the thermal observer borrows the AFE rail for an NTC sample.
    @param  every_ms   period between samples; 0 stops sampling entirely
    @param  settle_ms  how long the reference is given before the read */
bool Board_ThermalSetSample(uint32_t every_ms, uint32_t settle_ms);

/** What the sampling is set to now. */
void Board_ThermalSampling(uint32_t *every_ms, uint32_t *settle_ms);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_THERMAL_H */
