/**
  ******************************************************************************
  * @file    board_limits.h
  * @brief   The DRIVERS' fixed numbers, and why each is that number.
  ******************************************************************************
  */
#ifndef BOARD_LIMITS_H
#define BOARD_LIMITS_H


/* ---- THE GATE STAGE ---------------------------------------------------- */

/* Dead time, at runtime. */

#define BOARD_PWM_DEADTIME_MIN_NS 20U



#define BOARD_PWM_DTG_MAX 127U

/* ---- THE IMU, ON SPI2 -------------------------------------------------- */

/* Figure 6-8 puts the ceiling at 3 MHz. */

#define IMU_MAX_HZ 3000000U

/* One transaction's worth. */

#define IMU_BUF 320U

/** One SPI transfer, split so the STO charge pump keeps getting edges. */

#define IMU_CHUNK 8U

/* How often to clock a header out when the H_INTN edge was missed - see the
   file comment. */

#define IMU_POLL_HZ 1000U

/* Figure 6-6: tcssu, chip select to the first clock edge, is 0.1 us minimum,
   and tcssh, the hold after the last one, 16.83 ns. */

#define IMU_SETTLE_US 1U

/* How long to wait for the part to say it has something. */

#define IMU_INTN_WAIT_MS 5U

/* Waking is not polling. */

#define IMU_WAKE_WAIT_MS 50U

/* Figure 6-8: tnrst is 10 ns minimum, t1 is 90 ms of internal initialisation
   before the part is ready, t2 another 4 ms of configuration. */

#define IMU_RESET_HOLD_MS 1U

#define IMU_RESET_WAIT_MS 120U




/* How long the part must have been quiet before a Set Feature goes out. */

#define IMU_QUIET_MS 60U

/* ---- THE ANGLE SENSOR, ON SPI4 ----------------------------------------- */

/* Well under the datasheet's 10 MHz ceiling. */

#define ANGLE_MAX_HZ 3000000U



/* tCS is 50 ns to the first clock edge and tCS_IDLE is 200 ns between
   frames. */

#define ANGLE_SETTLE_US 1U

/* ---- ACQUISITION ------------------------------------------------------- */

/** 16 KB of DTCM. At one channel that is 2048 records, at all nine 409. */

/* THE ACQUISITION RING, in the AXI SRAM rather than DTCM. */
#define DAQ_BYTES (448U * 1024U)



/** Most samples the running accumulator may take before it stops widening. */

#define LIVE_MAX_ADDITIONS 32767U

/** Tone samples one poll may generate before it hands the loop back. */
#define BOARD_DAQ_TONE_BURST 64U

/** Digital pins a record can carry a duty for. */
#define BOARD_DAQ_MAX_PINS 16U

/** Rungs of the ladder a task can climb when its ring fills. */
#define BOARD_DAQ_LADDER 4U

/** Where the ring is when a task climbs, and when it comes back down, in
    eighths of capacity. */
#define BOARD_DAQ_CLIMB_AT 6U
#define BOARD_DAQ_FALL_AT  1U
#define BOARD_DAQ_RUNG_EIGHTHS 8U   /* both above are eighths of the ring */

/** And a ceiling on that in records, because THE LADDER ANSWERS LATENCY AND
    THE RING ABSORBS BURSTS - two jobs for one buffer, and only the first
    should move a rung. */
#define BOARD_DAQ_CLIMB_MAX 512U

/** Records at the low mark before a task steps back down. */
#define BOARD_DAQ_FALL_AFTER 64U

/* ---- THE THERMAL OBSERVER ---------------------------------------------- */

/** How often the model is stepped from the main loop. */

#define THERMAL_STEP_MS 100U

/** The most catch-up one poll will integrate, milliseconds. */
#define THERMAL_CATCHUP_MS 2000U



/** How often the rail is borrowed for a sample, by default. */
#define THERMAL_SAMPLE_EVERY_MS 30000U



/** Settle before the sample is believed. */

#define THERMAL_SAMPLE_SETTLE_MS 500U

/** The thermometers' floor for the online identification, kelvin: the NTC
    quantises at about 30 mK and TSEN at 125 mK, and a sample is one reading
    of each. */
#define THERMAL_IDENT_NOISE_K 0.1f

/** THE MARGIN POLICY'S REFERENCE, degrees C: what a ceiling's span is
    measured up from when the identification's doubt trims it (the record's
    floor, 0.8 by default, up to one as the evidence comes in -
    `thermal_ident_margin`). */
#define THERMAL_MARGIN_REF_C 25.0f

/** How much the margin must have moved since the ceilings were last trimmed
    for them to be trimmed again - a thousandth, so the twenty spans are not
    rewritten on every slice for a number that did not change to the
    precision the wire carries. */
#define THERMAL_MARGIN_STEP 0.001f

/** THE TRIP CAP: what the margin is held to after the envelope has dropped
    the stage, and how fast that hold lets go. */
#define THERMAL_TRIP_MARGIN        0.70f
#define THERMAL_TRIP_RECOVER_PER_S (0.30f / 1800.0f)

/** How long the link may be silent before the HOST's holds are dropped. */
#define BOARD_POWER_HOST_QUIET_MS 10000U

/* ---- POWER ------------------------------------------------------------- */

/** How long a borrowed hold lasts without renewal, milliseconds. */

#define BOARD_POWER_LEASE_MS 3000U

#endif /* BOARD_LIMITS_H */
