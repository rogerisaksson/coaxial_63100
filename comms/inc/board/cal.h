/** board/cal.h - what the comms stack needs from board_cal.c; included by board.h. */
#ifndef COMMS_BOARD_CAL_H
#define COMMS_BOARD_CAL_H

#include <stdbool.h>
#include <stdint.h>
#include "board/thermal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** One node's network entry in the record, milli-units; ZERO MEANS THE
    CORE'S DEFAULT for that field, so a record that never carried the network
    gets the derived one, and a default that improves reaches a board whose
    record has nothing to say about it. */
typedef struct
{
  uint32_t capacity_milli;     /**< J/K                                  */
  uint32_t to_ambient_milli;   /**< K/W to the air; a patch's own share  */
  uint32_t forced_milli;       /**< per sqrt(krpm)                       */
  uint32_t rth_milli;          /**< junction over node per watt          */
} board_cal_node_t;

/** An edge the record OPENS rather than defaults: the mount on a bench. */
#define BOARD_CAL_EDGE_OPEN 0xFFFFFFFFUL

#define BOARD_CAL_CHANNELS 10U

/** Which scalar Board_CalSetParam/GetParam addresses. */
#define BOARD_CAL_VREF_UV      0U  /**< ADC reference, microvolts           */
#define BOARD_CAL_SHUNT_UOHM   1U  /**< phase shunt, microhms               */
#define BOARD_CAL_AMP_GAIN_PPM 2U  /**< phase amplifier gain, ppm of 1 V/V  */
#define BOARD_CAL_BUS_R_TOP    3U  /**< DC link divider top, ohms           */
#define BOARD_CAL_BUS_R_BOTTOM 4U  /**< DC link divider bottom, ohms        */
#define BOARD_CAL_NTC_R25      5U  /**< thermistor at 25 C, ohms            */
#define BOARD_CAL_NTC_BETA_MK  6U  /**< B constant, milli-kelvin            */
#define BOARD_CAL_NTC_RFIXED   7U  /**< divider partner, ohms               */
#define BOARD_CAL_NTC_T25_CK   8U  /**< reference temperature, centikelvin  */
/* The two supply senses. */
#define BOARD_CAL_R5_R_TOP     9U  /**< +5 sense divider top, ohms          */
#define BOARD_CAL_R5_R_BOTTOM 10U  /**< +5 sense divider bottom, ohms       */
#define BOARD_CAL_VG_R_TOP    11U  /**< gate supply divider top, ohms       */
#define BOARD_CAL_VG_R_BOTTOM 12U  /**< gate supply divider bottom, ohms    */
#define BOARD_CAL_DEADTIME_NS 13U  /**< half-bridge dead time, nanoseconds  */
#define BOARD_CAL_DEADTIME_SKEW 14U /**< lead-lag trim, DTG counts         */
/* One past the last id above. */
/* CAL_VERSION 8: what the drive is told. */
#define BOARD_CAL_MOTOR_R_UOHM        15U  /**< phase resistance, microhms     */
#define BOARD_CAL_MOTOR_LD_NH         16U  /**< d inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LQ_NH         17U  /**< q inductance, nanohenry        */
#define BOARD_CAL_MOTOR_LAMBDA_UVS    18U  /**< PM flux linkage, uV.s          */
#define BOARD_CAL_MOTOR_POLE_PAIRS    19U
#define BOARD_CAL_DRV_KP_MV_PER_A     20U  /**< current loop kp, mV/A          */
#define BOARD_CAL_DRV_KI_V_PER_AS     21U  /**< current loop ki, V/(A.s)       */
#define BOARD_CAL_DRV_L1_MILLI        22U  /**< rotor observer angle gain, 1e-3      */
#define BOARD_CAL_DRV_L2_MILLI        23U  /**< rotor observer speed gain, 1e-3/s    */
#define BOARD_CAL_DRV_INJ_MV          24U  /**< injection amplitude, mV; 0 off */
#define BOARD_CAL_DRV_INJ_PERIODS     25U  /**< PWM periods per half cycle     */
#define BOARD_CAL_DRV_INJ_PHASE_MRAD  26U  /**< injection axis off d, signed   */
#define BOARD_CAL_DRV_EPS_GAIN_UA_PER_RAD 27U /**< demodulated uA/rad, signed */
#define BOARD_CAL_DRV_I_MAX_MA        28U  /**< reference clamp, mA            */
#define BOARD_CAL_DRV_I_TRIP_MA       29U  /**< the stage drops past this, mA  */
#define BOARD_CAL_DRV_V_FRAC_PPM      30U  /**< of Vdc/sqrt3 the vector may use*/
#define BOARD_CAL_DRV_SIGN            31U  /**< 1, or -1 as 0xFFFFFFFF         */
#define BOARD_CAL_DRV_W_LO_MRAD_S     32U  /**< back-EMF blend starts, mrad/s  */
#define BOARD_CAL_DRV_W_HI_MRAD_S     33U  /**< injection off above, mrad/s    */
#define BOARD_CAL_DRV_DT_STEP_MA      34U  /**< dead-time table spacing, mA    */
#define BOARD_CAL_DRV_DT_MV           35U  /**< 35..42: the table, mV          */
#define BOARD_CAL_DRV_SIGMA_I_UA      43U  /**< measured current noise, uA rms */
#define BOARD_CAL_DRV_TRIGGER_TICKS   44U  /**< the sample point chosen; 0 none*/
/* CAL_VERSION 9. */
#define BOARD_CAL_LINK_RATE           45U  /** < the RS485 pair's rate (the wire and the host say `link_baud`) */
/* CAL_VERSION 12: the winding's envelope. */
#define BOARD_CAL_WINDING_K_MILLI     46U  /**< K/W to the air, milli         */
#define BOARD_CAL_WINDING_J_MILLI     47U  /**< J/K, milli                     */
#define BOARD_CAL_WINDING_LIMIT_CENTI 48U  /**< ceiling, centi-degrees; 0 off  */
#define BOARD_CAL_PARAM_COUNT 46U

/** One channel's correction, applied to the raw code before any scaling. */
typedef struct
{
  int32_t offset_raw;   /**< subtracted first; what a zero measures       */
  int32_t gain_ppm;     /**< then scaled by 1 + gain_ppm/1e6              */
} board_cal_chan_t;

/** The whole record, as it sits in flash. */
typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t channels;
  uint32_t vref_uv;
  uint32_t shunt_uohm;
  uint32_t amp_gain_ppm;
  uint32_t bus_r_top_ohm;
  uint32_t bus_r_bottom_ohm;
  uint32_t r5_r_top_ohm;
  uint32_t r5_r_bottom_ohm;
  uint32_t vg_r_top_ohm;
  uint32_t vg_r_bottom_ohm;

  /* The half-bridge dead time. */
  uint32_t deadtime_ns;

  /* Lead against lag, in DTG counts. */
  uint32_t deadtime_skew;
  uint32_t ntc_r25_ohm;
  uint32_t ntc_beta_mk;
  uint32_t ntc_rfixed_ohm;
  uint32_t ntc_t25_ck;
  board_cal_chan_t chan[BOARD_CAL_CHANNELS];

  /* The thermal envelope. */
  int32_t  soa_limit_centi[BOARD_THERMAL_NODES];
  uint32_t soa_throttle_ppm;   /**< where derating starts, parts per million */
  /* CAL_VERSION 10: how far ahead the throttle looks, milliseconds. */
  uint32_t soa_lookahead_ms;
  /* CAL_VERSION 11: which nodes the current clamp cannot cool, one bit per
     BOARD_THERMAL_NODES index, bit 0 the first. */
  uint32_t soa_undriven_mask;

  /* CAL_VERSION 8: the drive. */
  uint32_t motor_r_uohm;
  uint32_t motor_ld_nh;
  uint32_t motor_lq_nh;
  uint32_t motor_lambda_uvs;
  uint32_t motor_pole_pairs;
  uint32_t drv_kp_mv_per_a;
  uint32_t drv_ki_v_per_as;
  uint32_t drv_l1_milli;
  uint32_t drv_l2_milli;
  uint32_t drv_inj_mv;
  uint32_t drv_inj_periods;
  uint32_t drv_inj_phase_mrad;
  uint32_t drv_eps_gain_ua_per_rad;
  uint32_t drv_i_max_ma;
  uint32_t drv_i_trip_ma;
  uint32_t drv_v_frac_ppm;
  uint32_t drv_sign;
  uint32_t drv_w_lo_mrad_s;
  uint32_t drv_w_hi_mrad_s;
  uint32_t drv_dt_step_ma;
  uint32_t drv_dt_mv[8];
  uint32_t drv_sigma_i_ua;
  uint32_t drv_trigger_ticks;

  /* CAL_VERSION 9: the RS485 pair's baud, applied to USART2 and UART5 at
     init. */
  uint32_t link_baud;

  /* CAL_VERSION 12: the winding's envelope. */
  uint32_t winding_k_per_w_milli;
  uint32_t winding_j_per_k_milli;
  int32_t  winding_limit_centi;

  /* CAL_VERSION 13: THE NETWORK, so the board carries the model it runs and
     an identification running on the board has somewhere to put what it
     learns. */
  board_cal_node_t thermal_node[BOARD_THERMAL_NODES];
  uint32_t thermal_edge_milli[BOARD_THERMAL_EDGES];
  uint32_t thermal_to_ambient_milli;     /**< the whole face, K/W          */
  uint32_t thermal_capacity_milli;       /**< the whole laminate, J/K      */
  uint32_t thermal_rad_share_ppm;
  uint32_t thermal_ntc_sees_ppm;
  uint32_t thermal_ntc_tau_ms;
  uint32_t thermal_rad_board_stator_micro; /**< W/K at 300 K; 0 = bench    */
  uint32_t thermal_k_iron_milli;         /**< W per (krpm)^2              */

  /* CAL_VERSION 15: THE MARGIN FLOOR, parts per million of every ceiling's
     span over 25 C - what the envelope keeps while the identification has no
     evidence for its model, rising to the whole span as the evidence comes
     in (`thermal_ident_margin`). */
  uint32_t soa_margin_floor_ppm;

  uint16_t crc;
} board_cal_t;

/** The margin floor into the record's RAM copy, ppm of the span;
    `Board_CalSave` is what commits it. */
bool Board_CalSetMarginFloor(uint32_t ppm);

/** Overlay one node's, one edge's or the bulk's network entry in the
    record's RAM copy; `Board_CalSave` is what commits it. */
bool Board_CalSetThermalNode(uint8_t node, uint32_t capacity_milli,
                             uint32_t to_ambient_milli);
bool Board_CalSetThermalEdge(uint8_t edge, uint32_t k_per_w_milli);
bool Board_CalSetThermalBulk(uint32_t to_ambient_milli,
                             uint32_t capacity_milli);

/** Load the stored record, or fall back to the compiled-in defaults. */
void Board_CalInit(void);

/** The record in force now, stored or default. */
const board_cal_t *Board_Cal(void);

/** Whether flash holds a valid record, as against these being the defaults. */
bool Board_CalStored(void);

/** Replace the working record with the compiled-in defaults. */
void Board_CalDefaults(void);

/** Re-read flash, discarding uncommitted edits. */
bool Board_CalLoad(void);

/** Commit the working record to flash and read it back to prove it landed. */
bool Board_CalSave(void);

/* False from either of these means the id or the index does not exist, or
   the value would make a conversion divide by zero. */
bool Board_CalSetParam(uint8_t id, uint32_t value);
bool Board_CalGetParam(uint8_t id, uint32_t *value);

bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm);

/** One node's ceiling, centi-degrees C. */
bool Board_CalSetLimit(uint8_t node, int32_t limit_centi);

/** Where derating starts, parts per million of the budget. */
bool Board_CalSetThrottle(uint32_t ppm);

/** The winding's envelope: ceiling in centi-degrees (zero disables), K/W and
    J/K in milli. */
bool Board_CalSetWinding(int32_t limit_centi, uint32_t k_per_w_milli,
                         uint32_t j_per_k_milli);
bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm);

/** Correct one raw code: offset first, then gain.
    @return The code unchanged for an index the record does not cover, because */
int32_t Board_CalApply(uint8_t index, int32_t raw);

/** Measure a channel now and store the reading as its offset.
    @param  measured  The code that was stored, before correction. */
bool Board_CalZero(uint8_t index, int32_t *measured);

/** Measure a channel now and trim its gain so the reading equals */
bool Board_CalSpan(uint8_t index, int32_t reference, int32_t *measured);

/** The margin floor, a fraction of every ceiling's span, through the record
    - `Board_CalSave` persists it. */
bool Board_ThermalSetMarginFloor(float floor);

#ifdef __cplusplus
}
#endif

#endif /* COMMS_BOARD_CAL_H */
