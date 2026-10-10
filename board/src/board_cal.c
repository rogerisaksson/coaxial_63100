/** board_cal.c - The calibration record: scaling, corrections, its flash image. */
#include "board.h"

#include "modbus_crc.h"
#include "board_units.h"

#include <stddef.h>
#include <string.h>

#define CAL_MAGIC   0x43583633U   /* 'CX63'; erased flash reads 0xFF */
/* Layout history: 2 supply senses, 4 thermal envelope, 5 dead time, 6 skew,
   7 per-leg nodes, 8 drive, 9 RS485 baud, 10 lookahead, 11 undriven mask,
   12 winding, 13 twenty-node network, 14 identified scales, 15 margin floor
   (nothing identified is kept), 16 application. */
#define CAL_VERSION 16U

#define CAL_FLOOR_AT offsetof(board_cal_t, soa_margin_floor_ppm)
#define CAL_APP_AT   offsetof(board_cal_t, thermal_app)

/** The versions a stored record is taken from: the bytes of it kept, where its CRC is. */
static const struct
{
  uint32_t version;
  size_t kept;
  size_t crc_at;
} CAL_EARLIER[] =
{
  { 15U, CAL_APP_AT, CAL_APP_AT },
  { 14U, CAL_FLOOR_AT, CAL_FLOOR_AT + 4U * sizeof(uint32_t) },
  { 13U, CAL_FLOOR_AT, CAL_FLOOR_AT },
};

/* The image written is padded to whole flash words; the record is a few hundred
   bytes against a 128 KB sector. */
#define CAL_IMAGE_BYTES (((sizeof(board_cal_t) + BOARD_FLASH_WORD_BYTES - 1U) / \
                          BOARD_FLASH_WORD_BYTES) * BOARD_FLASH_WORD_BYTES)

static board_cal_t s_cal;

/* Each channel's offset and gain_ppm/1e6 in Q28, laid from the record wherever it changes - zero
   the identity, the row past them an unknown index's: a read an SMULL and a shift, where a 64-bit
   divide ran __udivmoddi4's 40 branches a DAQ sample (2026-10-10). */
#define CAL_SCALE_Q 28
static struct
{
  int32_t offset;
  int32_t gain;
} s_laid[BOARD_CAL_CHANNELS + 1U];

static void lay(void)
{
  for (uint32_t i = 0U; i < BOARD_CAL_CHANNELS; i++)
  {
    const int64_t gain = (int64_t)s_cal.chan[i].gain_ppm * ((int64_t)1 << CAL_SCALE_Q) / 1000000;
    const int64_t most = INT32_MAX - ((int64_t)1 << CAL_SCALE_Q);

    s_laid[i].offset = s_cal.chan[i].offset_raw;
    s_laid[i].gain = (int32_t)((gain < most) ? gain : most);
  }
}

/* Compiled-in defaults: what the schematic says, traced 2026-08-26 from
   electronics/Coaxial 63100 Schematics.pdf. */
static const board_cal_t CAL_DEFAULTS =
{
  .magic            = CAL_MAGIC,
  .version          = CAL_VERSION,
  .channels         = BOARD_CAL_CHANNELS,

  /* The thermal envelope, centi-degrees C per node in thermal_node_t order:
     driver U/V/W, phase U/V/W, mcu, regulators, afe, board. A leg is judged
     on its FETs' junction: the IAUCN10S7N021's 175 C (Rev 1.2); its
     2EDL8034 sits 8-13 mm off them on the leg's patch, whose laminate holds
     it 20 K under its own 125. */
  .soa_limit_centi  = { 17500, 17500, 17500,      /* driver U, V, W */
                        12500, 12500, 12500,      /* phase  U, V, W */
                        12500, 12500, 12500, 10500,  /* mcu, regs, afe, centre */
                        /* CAL_VERSION 13: the hot swap's FETs are the
                           bridge's part, so its junction limit; the six
                           patches are laminate like the centre; the winding
                           its own 120 (see below), the stator's iron and the
                           rotor's magnets ESTIMATES at the winding's, since
                           a bonded magnet's flux is the first thing lost
                           above it. */
                        12500,                         /* hotswap */
                        10500, 10500, 10500,           /* patch U, V, W */
                        10500, 10500, 10500,           /* left, bottom, right */
                        12000, 12000, 12000 },         /* winding, stator, rotor */
  .soa_throttle_ppm = 900000UL,
  /* Two seconds of reaction window. */
  .soa_lookahead_ms = 2000UL,
  /* The MCU, the regulators and the front end. */
  .soa_undriven_mask = (1UL << BOARD_THERMAL_MCU)
                     | (1UL << BOARD_THERMAL_REGULATORS)
                     | (1UL << BOARD_THERMAL_AFE),
  /* The winding, CAL_VERSION 12. */
  .winding_k_per_w_milli = 2200UL,
  .winding_j_per_k_milli = 180000UL,
  .winding_limit_centi   = 12000,
  .vref_uv          = 3300000UL,      /* U2 REF2033, 3.3 V +/-0.05 % */
  .shunt_uohm       = 3500UL,         /* RU1 || RU2, 7 mohm each */
  .amp_gain_ppm     = 4545455UL,      /* THS4551, Rf 1.5k / Rg 330 */
  .bus_r_top_ohm    = 49900UL,        /* R12 */
  .bus_r_bottom_ohm = 2200UL,         /* R11 */
  .r5_r_top_ohm     = 10000UL,        /* R113 element 2, +5 to PA4 */
  .r5_r_bottom_ohm  = 10000UL,        /* R113 element 1, PA4 to GND */
  .vg_r_top_ohm     = 57000UL,        /* R119 47k + R113 element 3 10k */
  .vg_r_bottom_ohm  = 10000UL,        /* R113 element 4, PA5 to GND */

  /* DTG 15, 63.2 ns: 33.7 shot through dry, 42 did not (bench, 2026-10-05). */
  .deadtime_ns      = 60UL,

  /* No trim until something is measured. */
  .deadtime_skew    = 0UL,
  .ntc_r25_ohm      = 10000UL,        /* NCU18XH103D60RB */
  .ntc_beta_mk      = 3380000UL,      /* B25/50 = 3380 K, in milli-kelvin */
  .ntc_rfixed_ohm   = 10000UL,        /* R100, ERA-3AEB103V 0.1 % */
  .ntc_t25_ck       = 29815UL,        /* 298.15 K */

  /* The drive, CAL_VERSION 8. */
  .motor_r_uohm             = 50000UL,      /* 50 mohm */
  .motor_ld_nh              = 20000UL,      /* 20 uH */
  .motor_lq_nh              = 25000UL,
  .motor_lambda_uvs         = 5000UL,       /* 5 mV.s */
  .motor_pole_pairs         = 7UL,
  .drv_kp_mv_per_a          = 100UL,
  .drv_ki_v_per_as          = 250UL,
  .drv_l1_milli             = 100UL,
  .drv_l2_milli             = 100000UL,
  .drv_inj_mv               = 0UL,
  .drv_inj_periods          = 1UL,
  .drv_inj_phase_mrad       = 0UL,
  .drv_eps_gain_ua_per_rad  = 0UL,
  .drv_i_max_ma             = 5000UL,
  .drv_i_trip_ma            = 100000UL,     /* the rating */
  .drv_v_frac_ppm           = 950000UL,
  .drv_sign                 = 1UL,
  .drv_w_lo_mrad_s          = 60000UL,
  .drv_w_hi_mrad_s          = 120000UL,
  .drv_dt_step_ma           = 1000UL,
  .drv_dt_mv                = { 0UL },
  .drv_sigma_i_ua           = 0UL,
  .drv_trigger_ticks        = 0UL,
  .link_baud                = 115200UL,     /* the number the docs promised */
  .chan             = { { 0, 0 } },   /* no offset, no gain trim */
  .soa_margin_floor_ppm     = BOARD_SOA_MARGIN_FLOOR_PPM,  /* the bench's 80 % */
};

/* CRC-16 over everything ahead of the crc field itself. */
static uint16_t cal_crc(const board_cal_t *cal)
{
  return modbus_crc16((const uint8_t *)cal,
                      offsetof(board_cal_t, crc));
}

static bool cal_valid(const board_cal_t *cal)
{
  return (cal->magic == CAL_MAGIC) &&
         (cal->version == CAL_VERSION) &&
         (cal->channels == BOARD_CAL_CHANNELS) &&
         (cal->crc == cal_crc(cal));
}

/** Of a record an earlier version wrote, the bytes kept; 0 for none. */
static size_t cal_earlier(const board_cal_t *stored)
{
  const uint8_t *bytes = (const uint8_t *)stored;

  for (size_t i = 0U; i < sizeof(CAL_EARLIER) / sizeof(CAL_EARLIER[0]); i++)
  {
    uint16_t crc;

    memcpy(&crc, bytes + CAL_EARLIER[i].crc_at, sizeof(crc));
    if ((stored->version == CAL_EARLIER[i].version) && (stored->magic == CAL_MAGIC) &&
        (stored->channels == BOARD_CAL_CHANNELS) &&
        (crc == modbus_crc16(bytes, CAL_EARLIER[i].crc_at)))
    {
      return CAL_EARLIER[i].kept;
    }
  }
  return 0U;
}

/** Take a stored record into RAM: this version whole, an earlier one's kept bytes over
    the defaults. */
static bool cal_take(const board_cal_t *stored)
{
  const size_t kept = cal_earlier(stored);

  if (cal_valid(stored))
  {
    s_cal = *stored;
    lay();
    return true;
  }
  if (kept > 0U)
  {
    s_cal = CAL_DEFAULTS;
    memcpy(&s_cal, stored, kept);
    s_cal.version = CAL_VERSION;
    s_cal.crc = cal_crc(&s_cal);
    lay();
    return true;
  }
  return false;
}

void Board_CalInit(void)
{
  if (cal_take((const board_cal_t *)Board_FlashRecord()))
  {
    return;
  }

  /* Never written, or written by an older layout, or corrupted. */
  s_cal = CAL_DEFAULTS;
  s_cal.crc = cal_crc(&s_cal);
  lay();
}

const board_cal_t *Board_Cal(void)
{
  return &s_cal;
}

bool Board_CalStored(void)
{
  const board_cal_t *stored = (const board_cal_t *)Board_FlashRecord();

  return cal_valid(stored) || (cal_earlier(stored) > 0U);
}

void Board_CalDefaults(void)
{
  s_cal = CAL_DEFAULTS;
  s_cal.crc = cal_crc(&s_cal);
  lay();
}

bool Board_CalLoad(void)
{
  return cal_take((const board_cal_t *)Board_FlashRecord());
}

bool Board_CalSave(void)
{
  uint8_t image[CAL_IMAGE_BYTES];

  s_cal.crc = cal_crc(&s_cal);

  /* Pad with 0xFF, which is what an erased cell reads, so the tail of the
     last flash word is indistinguishable from never having been written. */
  memset(image, 0xFF, sizeof(image));
  memcpy(image, &s_cal, sizeof(s_cal));

  /* Read it back rather than trust the programmer's return: a save that
     reports success and left the sector unreadable is the failure this whole
     record exists to survive. */
  return Board_FlashRecordWrite(image, (uint32_t)sizeof(image)) &&
         cal_valid((const board_cal_t *)Board_FlashRecord());
}

/* Which scalar an id names. */
static uint32_t *cal_field(uint8_t id)
{
  if ((id >= BOARD_CAL_DRV_DT_MV) && (id < (BOARD_CAL_DRV_DT_MV + 8U)))
  {
    return &s_cal.drv_dt_mv[id - BOARD_CAL_DRV_DT_MV];
  }
  switch (id)
  {
    case BOARD_CAL_VREF_UV:      return &s_cal.vref_uv;
    case BOARD_CAL_SHUNT_UOHM:   return &s_cal.shunt_uohm;
    case BOARD_CAL_AMP_GAIN_PPM: return &s_cal.amp_gain_ppm;
    case BOARD_CAL_BUS_R_TOP:    return &s_cal.bus_r_top_ohm;
    case BOARD_CAL_BUS_R_BOTTOM: return &s_cal.bus_r_bottom_ohm;
    case BOARD_CAL_NTC_R25:      return &s_cal.ntc_r25_ohm;
    case BOARD_CAL_NTC_BETA_MK:  return &s_cal.ntc_beta_mk;
    case BOARD_CAL_NTC_RFIXED:   return &s_cal.ntc_rfixed_ohm;
    case BOARD_CAL_NTC_T25_CK:   return &s_cal.ntc_t25_ck;
    case BOARD_CAL_R5_R_TOP:     return &s_cal.r5_r_top_ohm;
    case BOARD_CAL_R5_R_BOTTOM:  return &s_cal.r5_r_bottom_ohm;
    case BOARD_CAL_VG_R_TOP:     return &s_cal.vg_r_top_ohm;
    case BOARD_CAL_VG_R_BOTTOM:  return &s_cal.vg_r_bottom_ohm;
    case BOARD_CAL_DEADTIME_NS:  return &s_cal.deadtime_ns;
    case BOARD_CAL_DEADTIME_SKEW: return &s_cal.deadtime_skew;
    case BOARD_CAL_LINK_RATE:    return &s_cal.link_baud;
    case BOARD_CAL_MOTOR_R_UOHM:  return &s_cal.motor_r_uohm;
    case BOARD_CAL_MOTOR_LD_NH:   return &s_cal.motor_ld_nh;
    case BOARD_CAL_MOTOR_LQ_NH:   return &s_cal.motor_lq_nh;
    case BOARD_CAL_MOTOR_LAMBDA_UVS: return &s_cal.motor_lambda_uvs;
    case BOARD_CAL_MOTOR_POLE_PAIRS: return &s_cal.motor_pole_pairs;
    case BOARD_CAL_DRV_KP_MV_PER_A: return &s_cal.drv_kp_mv_per_a;
    case BOARD_CAL_DRV_KI_V_PER_AS: return &s_cal.drv_ki_v_per_as;
    case BOARD_CAL_DRV_L1_MILLI:  return &s_cal.drv_l1_milli;
    case BOARD_CAL_DRV_L2_MILLI:  return &s_cal.drv_l2_milli;
    case BOARD_CAL_DRV_INJ_MV:    return &s_cal.drv_inj_mv;
    case BOARD_CAL_DRV_INJ_PERIODS: return &s_cal.drv_inj_periods;
    case BOARD_CAL_DRV_INJ_PHASE_MRAD: return &s_cal.drv_inj_phase_mrad;
    case BOARD_CAL_DRV_EPS_GAIN_UA_PER_RAD:
      return &s_cal.drv_eps_gain_ua_per_rad;
    case BOARD_CAL_DRV_I_MAX_MA:  return &s_cal.drv_i_max_ma;
    case BOARD_CAL_DRV_I_TRIP_MA: return &s_cal.drv_i_trip_ma;
    case BOARD_CAL_DRV_V_FRAC_PPM: return &s_cal.drv_v_frac_ppm;
    case BOARD_CAL_DRV_SIGN:      return &s_cal.drv_sign;
    case BOARD_CAL_DRV_W_LO_MRAD_S: return &s_cal.drv_w_lo_mrad_s;
    case BOARD_CAL_DRV_W_HI_MRAD_S: return &s_cal.drv_w_hi_mrad_s;
    case BOARD_CAL_DRV_DT_STEP_MA: return &s_cal.drv_dt_step_ma;
    case BOARD_CAL_DRV_SIGMA_I_UA: return &s_cal.drv_sigma_i_ua;
    case BOARD_CAL_DRV_TRIGGER_TICKS: return &s_cal.drv_trigger_ticks;
    case BOARD_CAL_WINDING_K_MILLI: return &s_cal.winding_k_per_w_milli;
    case BOARD_CAL_WINDING_J_MILLI: return &s_cal.winding_j_per_k_milli;
    /* Signed in the record, a u32 on the wire and here: the same
       two's-complement carriage every signed parameter uses. */
    case BOARD_CAL_WINDING_LIMIT_CENTI:
      return (uint32_t *)&s_cal.winding_limit_centi;
    default:                     return NULL;
  }
}

bool Board_CalSetParam(uint8_t id, uint32_t value)
{
  uint32_t *field = cal_field(id);

  if (field == NULL)
  {
    return false;
  }

  /* Every one of the thirteen scaling parameters is a divisor or a
     multiplicand somewhere, so zero is refused for those alike - and only
     those: the dead time's skew and the drive's numbers are legitimately
     zero (no skew, injection off), and were refused at zero until
     2026-08-31. */
  if ((value == 0U) && (id <= BOARD_CAL_VG_R_BOTTOM))
  {
    return false;
  }

  /* The RS485 baud is bounded, not judged: below 9600 the RTU silences stop
     fitting the deadman's numbers, above 921600 nothing on this bench has
     been measured (the THVD1450 itself is rated 50 Mbps). */
  if ((id == BOARD_CAL_LINK_RATE)
      && ((value < 9600U) || (value > 921600U)))
  {
    return false;
  }

  *field = value;
  return true;
}

bool Board_CalGetParam(uint8_t id, uint32_t *value)
{
  const uint32_t *field = cal_field(id);

  if ((field == NULL) || (value == NULL))
  {
    return false;
  }

  *value = *field;
  return true;
}

bool Board_CalSetLimit(uint8_t node, int32_t limit_centi)
{
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    return false;
  }
  s_cal.soa_limit_centi[node] = limit_centi;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetThrottle(uint32_t ppm)
{
  if ((ppm == 0U) || (ppm >= PPM_WHOLE))
  {
    return false;
  }
  s_cal.soa_throttle_ppm = ppm;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetWinding(int32_t limit_centi, uint32_t k_per_w_milli,
                         uint32_t j_per_k_milli)
{
  /* A zero ceiling disables the winding and is allowed; the two constants
     divide the step and are not. */
  if ((limit_centi < 0) || (k_per_w_milli == 0U) || (j_per_k_milli == 0U))
  {
    return false;
  }
  s_cal.winding_limit_centi = limit_centi;
  s_cal.winding_k_per_w_milli = k_per_w_milli;
  s_cal.winding_j_per_k_milli = j_per_k_milli;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetThermalNode(uint8_t node, uint32_t capacity_milli,
                             uint32_t to_ambient_milli)
{
  if (node >= (uint8_t)BOARD_THERMAL_NODES)
  {
    return false;
  }
  s_cal.thermal_node[node].capacity_milli = capacity_milli;
  s_cal.thermal_node[node].to_ambient_milli = to_ambient_milli;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetThermalEdge(uint8_t edge, uint32_t k_per_w_milli)
{
  if (edge >= (uint8_t)BOARD_THERMAL_EDGES)
  {
    return false;
  }
  s_cal.thermal_edge_milli[edge] = k_per_w_milli;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetThermalBulk(uint32_t to_ambient_milli,
                             uint32_t capacity_milli)
{
  s_cal.thermal_to_ambient_milli = to_ambient_milli;
  s_cal.thermal_capacity_milli = capacity_milli;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetMarginFloor(uint32_t ppm)
{
  /* Zero would put every ceiling at the reference the moment the board
     booted and trip the stage on its first sample; above the span is a
     margin the record's own ceilings do not have. */
  if ((ppm == 0U) || (ppm > PPM_WHOLE))
  {
    return false;
  }
  s_cal.soa_margin_floor_ppm = ppm;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetApplication(uint32_t app)
{
  s_cal.thermal_app = app;
  s_cal.crc = cal_crc(&s_cal);
  return true;
}

bool Board_CalSetChannel(uint8_t index, int32_t offset_raw, int32_t gain_ppm)
{
  if (index >= BOARD_CAL_CHANNELS)
  {
    return false;
  }

  /* A gain trim of -1e6 ppm is a scale factor of zero, and everything below
     it changes the sign of the reading. */
  if (gain_ppm <= -1000000)
  {
    return false;
  }

  s_cal.chan[index].offset_raw = offset_raw;
  s_cal.chan[index].gain_ppm = gain_ppm;
  lay();
  return true;
}

bool Board_CalChannel(uint8_t index, int32_t *offset_raw, int32_t *gain_ppm)
{
  if ((index >= BOARD_CAL_CHANNELS) || (offset_raw == NULL) ||
      (gain_ppm == NULL))
  {
    return false;
  }

  *offset_raw = s_cal.chan[index].offset_raw;
  *gain_ppm = s_cal.chan[index].gain_ppm;
  return true;
}

int32_t Board_CalApply(uint8_t index, int32_t raw)
{
  const uint32_t k = (index < BOARD_CAL_CHANNELS) ? index : BOARD_CAL_CHANNELS;
  const int64_t scaled = (int64_t)(raw - s_laid[k].offset)
                         * (s_laid[k].gain + ((int32_t)1 << CAL_SCALE_Q));

  return (int32_t)((scaled + ((int64_t)1 << (CAL_SCALE_Q - 1))) >> CAL_SCALE_Q);
}
