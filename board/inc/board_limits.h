/** board_limits.h - The drivers' fixed numbers, and why each is that number. */
#ifndef BOARD_LIMITS_H
#define BOARD_LIMITS_H

/* ---- Gate stage -------------------------------------------------------- */

/* Dead time, at runtime. */

#define BOARD_PWM_DEADTIME_MIN_NS 20U

#define BOARD_PWM_DTG_MAX 127U

/* ---- IMU, on SPI2 ------------------------------------------------------ */

/* Figure 6-8 puts the ceiling at 3 MHz. */

#define IMU_MAX_HZ 3000000U

/* One transaction's worth. */

#define IMU_BUF 320U

/** One SPI transfer, split so the STO charge pump keeps getting edges. */

#define IMU_CHUNK 8U

/* Header poll interval when an H_INTN edge was missed. */

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

/* ---- Angle sensor, on SPI4 --------------------------------------------- */

/* Well under the datasheet's 10 MHz ceiling. */

#define ANGLE_MAX_HZ 3000000U

/* tCS is 50 ns to the first clock edge and tCS_IDLE is 200 ns between
   frames. */

#define ANGLE_SETTLE_US 1U

/* ---- Acquisition ------------------------------------------------------- */

/** The reference up behind AFE_ON before the converters calibrate on it: the
    link's code is within its noise 25 ms after the rail (bench, 2026-10-05).
    Uncalibrated, a code sticks 70 under every 512. */
#define ADC_REFERENCE_SETTLE_MS 100U

/* The acquisition ring, in the AXI SRAM rather than DTCM. At one channel
   that is 45 875 records, at all ten 9 972 (`DAQ_RECORD_BYTES`, no pins or
   sensors). */
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

/** A ceiling on that in records: the ladder answers latency and the ring
    absorbs bursts, two jobs for one buffer, and only the first should move a
    rung. */
#define BOARD_DAQ_CLIMB_MAX 512U

/** Records at the low mark before a task steps back down. */
#define BOARD_DAQ_FALL_AFTER 64U

/* ---- Thermal observer -------------------------------------------------- */

/** How often the model is stepped from the main loop. */

#define THERMAL_STEP_MS 100U

/** The most catch-up one poll will integrate, milliseconds. */
#define THERMAL_CATCHUP_MS 2000U

/** The fastest thermal clock op 13 takes, thermal ms per wall ms: a poll's
    span stays inside a u32. */
#define THERMAL_HASTE_MAX 1000U

/** How often the rail is borrowed for a sample, by default. */
#define THERMAL_SAMPLE_EVERY_MS 30000U

/** Settle before the sample is believed. */

#define THERMAL_SAMPLE_SETTLE_MS 500U

/* The identification's floor, the margin's reference and step, the trip cap and the derate's
   recovery are the envelope's own: thermal_run.h. */

/** The longest WEP (thermal op 14) holds the derate off for one ask: a host lost in
    one gives the envelope back this soon. */
#define THERMAL_WEP_MAX_MS 2000U

/** The host's word on the airspeed (thermal op 16) holds this long, then none: a host gone
    quiet leaves the observer the rotor's wash alone. Past THERMAL_AIRSPEED_MAX_MM_S, refused. */
#define THERMAL_AIRSPEED_HOLD_MS   1000U
#define THERMAL_AIRSPEED_MAX_MM_S  100000U

/** How long the link may be silent before the host's holds are dropped. */
#define BOARD_POWER_HOST_QUIET_MS 10000U

/* ---- Power ------------------------------------------------------------- */

/** How long a borrowed hold lasts without renewal, milliseconds. */

#define BOARD_POWER_LEASE_MS 3000U

#endif /* BOARD_LIMITS_H */
