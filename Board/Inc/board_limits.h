/**
  ******************************************************************************
  * @file    board_limits.h
  * @brief   The DRIVERS' fixed numbers, and why each is that number.
  *
  * A limit in the widest sense: what a part will take, what a buffer must
  * hold, how often something may run, and the floors under all three. One
  * #define per file is where they belong until the day one is wrong, and
  * then there is no way to compare two of them without opening two files.
  *
  * WHAT IS NOT HERE. Anything the board can be TOLD is in the calibration
  * record (invariant 7): the dead time, the thermal ceilings, every scaling
  * parameter. This file is what a rebuild is needed to change, and the floor
  * a stored value is checked against. Op codes, register maps and channel
  * indices stay in the one file that dispatches on them, and so do pins.
  *
  * The comms stack has its own, `comms_limits.h`, and includes this one where
  * a number on the wire has to be checked against a number in a driver.
  * Never the other way: Board/ does not reach up into Comms/.
  ******************************************************************************
  */
#ifndef BOARD_LIMITS_H
#define BOARD_LIMITS_H


/* ---- THE GATE STAGE ---------------------------------------------------- */

/* Dead time, at runtime. 20 ns is a FLOOR, not a default: the 2EDL8034 has
 * no interlock, so this is the only thing between the two FETs of a leg, and
 * asking for less gets 20 ns and a sentence.
 *
 * DTG is not linear - only the low range steps by one t_DTS. This uses that
 * alone, capping at 127 x t_DTS = 535 ns, six times what the bridge needs.
 */

#define BOARD_PWM_DEADTIME_MIN_NS 20U



#define BOARD_PWM_DTG_MAX 127U

/* ---- THE IMU, ON SPI2 -------------------------------------------------- */

/* Figure 6-8 puts the ceiling at 3 MHz. The divider is a power of two, so
   at a 190 MHz kernel clock the choice is 2.97 MHz or 1.48 MHz - nothing
   between.
   2.97 MHz was rejected once, with "every read came back FF". That did NOT
   reproduce on 2026-08-29: 47 000 reports at 394 Hz, zero read errors, and
   the product id answers. What is different since is the chip select held
   across header and cargo, and SPI2 set to mode 3 in the .ioc as well as
   here. The FF reads are more likely to have been that.
   Kept at 2.97 because the transfer cost was the rate limit: at 1.48 MHz the
   part gave 381 Hz of a requested 400 and 28 errors in 5 s; at 2.97 it gives
   394 Hz and none. Polling more often was tried instead and was worse - at
   4 kHz the rate FELL to 360 Hz and the errors tripled. */

#define IMU_MAX_HZ 3000000U

/* One transaction's worth. Sized for the SHTP advertisement, which is the
   largest thing the part sends unprompted: measured on this board, 276 bytes
   including the header, carrying the channel map and version strings. Sixty
   four was enough for a product id response and refused the advertisement
   outright, which is what CMD_ERR_DEVICE on every read after a reset was. */

#define IMU_BUF 320U

/** One SPI transfer, split so the STO charge pump keeps getting edges.
   A 320-byte cargo at 1.48 MHz is 1.73 ms of blocking transfer, and the
   keepalive latch holds only a few hundred microseconds (FINDINGS). Chip
   select is ours - NSS_SOFT, PB12 by hand - and is NOT touched here, so the
   part still sees one unbroken transaction however this is chunked.
   Releasing it between chunks is the FF FF FF FF bug again.
   8 bytes is 43 us at 1.48 MHz, against the 52 us the main loop already
   takes per iteration once the IMU is being polled. Finer buys nothing the
   loop does not already cost; coarser makes this the worst gap in the
   system. */

#define IMU_CHUNK 8U

/* How often to clock a header out when the H_INTN edge was missed - see the
   file comment. Rate limited because it is not free: a four-byte transfer at
   2.97 MHz is 13 us and the main loop also carries Modbus, whose t1.5 at
   115200 is 143 us. At 1 kHz that is 1.3 % of the loop.
   Measured 2026-08-29, and this is why it stays at 1 kHz: raising it to
   4 kHz LOWERED the report rate from 394 Hz to 360 and tripled the errors.
   The poll is not what the rate is short of - the transfer is.
   Raw CYCCNT and unsigned subtraction, so the wrap costs nothing
   (invariant 2). */

#define IMU_POLL_HZ 1000U

/* Figure 6-6: tcssu, chip select to the first clock edge, is 0.1 us minimum,
   and tcssh, the hold after the last one, 16.83 ns. A GPIO write followed
   straight away by HAL_SPI_TransmitReceive is a couple of core cycles - 27 ns
   at the 75 MHz this board currently runs at - so the setup was a quarter of
   what the part asks for. One microsecond is ten times the requirement and
   costs nothing at these transfer sizes. */

#define IMU_SETTLE_US 1U

/* How long to wait for the part to say it has something. Anything longer
   would hold the Modbus link past the master's patience for a part that is
   simply idle, which is not an error. */

#define IMU_INTN_WAIT_MS 5U

/* Waking is not polling. Asserting PS0/WAKE takes the part out of a sleep
   state and it answers by asserting H_INTN "at which point the host can
   initiate SPI accesses" (1.2.4.3) - that is a wake-up, not a sample period.
   Five milliseconds was enough for a part that was already awake and not for
   one that was not: measured, every write failed once the reset's queue had
   been drained. */

#define IMU_WAKE_WAIT_MS 50U

/* Figure 6-8: tnrst is 10 ns minimum, t1 is 90 ms of internal initialisation
   before the part is ready, t2 another 4 ms of configuration. One millisecond
   of reset is four orders of magnitude past the minimum and costs nothing;
   120 ms afterwards leaves margin on t1+t2 without a pin to be told on. */

#define IMU_RESET_HOLD_MS 1U

#define IMU_RESET_WAIT_MS 120U




/* How long the part must have been quiet before a Set Feature goes out.
   The advertisement is 276 bytes over several cargoes and a write into
   the middle of it is accepted and discarded. */

#define IMU_QUIET_MS 60U

/* ---- THE ANGLE SENSOR, ON SPI4 ----------------------------------------- */

/* Well under the datasheet's 10 MHz ceiling. The divider is a power of two,
   so at a 100 MHz kernel clock the choice either side is 6.25 MHz or
   1.56 MHz; the lower one costs 13 us a packet and buys the margin. */

#define ANGLE_MAX_HZ 3000000U



/* tCS is 50 ns to the first clock edge and tCS_IDLE is 200 ns between
   frames. One microsecond covers both several times over and costs nothing
   at one packet per read. */

#define ANGLE_SETTLE_US 1U

/* ---- ACQUISITION ------------------------------------------------------- */

/** 16 KB of DTCM. At one channel that is 2048 records, at all nine 409. */

/* THE ACQUISITION RING, in the AXI SRAM rather than DTCM.

   256 K of the 512 K that was standing empty. At ten channels and a
   49-byte record that is 5350 records - a minute and a half at the 63
   a second ten channels leave, or 22 seconds at the 240 one channel
   does. The old 16 K held 334 records, five seconds, and a terminal
   that stopped drawing for six overflowed it (FINDINGS).

   Nothing on an interrupt path touches it: the main loop fills it a
   byte at a time and a command handler empties it the same way, so
   the AXI bus costs it nothing that matters - and with no DMA and the
   data cache off there is no coherency question either. */
#define DAQ_BYTES (448U * 1024U)



/** Most samples the running accumulator may take before it stops widening.
  * INT32_MAX / 65535: the largest a single-ended code can be, so one more
  * addition can never overflow `sum`. */

#define LIVE_MAX_ADDITIONS 32767U

/** Tone samples one poll may generate before it hands the loop back.
  *
  * The generator owes whatever the elapsed cycles bought, and a long gap
  * owes thousands: a round trip between `tone` and `start` is 15 ms,
  * which at 1 Msps is 15 000 samples. Bounded because it runs beside
  * the link - RTU discards a frame whose characters arrive more than
  * t1.5, 143 us at 115200, apart.
  *
  * MEASURED TWICE. 4096 was the first number and the board went silent
  * on the next 0x6E the moment a tone started - one burst of them is
  * milliseconds. 256 was the second, and the worst keepalive gap went
  * from 24.9 us idle to 262 us: a SAMPLE COSTS 440 CYCLES through
  * feed() with two fields, not the 40 that was estimated, so 256 of
  * them is 237 us and over t1.5 all by itself.
  *
  * 64 costs nothing in throughput and buys the budget back: the loop
  * is generator-dominated while a tone runs, so what it sustains is
  * one sample per 440 cycles whatever the burst - about 1 Msample/s -
  * and only the gap scales with the bound. What the clamp drops is
  * dropped rather than owed: a debt carried forward bursts again next
  * turn and never catches up. */
#define BOARD_DAQ_TONE_BURST 64U

/** Digital pins a record can carry a duty for. Three are drivable on
  * this board (AFE_ON, UART5_TERM, KEEPALIVE); the headroom is for a
  * revision that grows some, and costs one byte of the record each. */
#define BOARD_DAQ_MAX_PINS 16U

/** Rungs of the ladder a task can climb when its ring fills.
  *
  * Each rung is a WHOLE design - boxcar, coefficients, decimation - so
  * climbing one is still an anti-alias filter and not just fewer
  * samples. Decimating harder without redesigning is how a fold gets
  * in, and the board has no way to design anything: the host does that
  * and sends the ladder. Four rungs is 336 bytes and covers 8x. */
#define BOARD_DAQ_LADDER 4U

/** Where the ring is when a task climbs, and when it comes back down,
  * in eighths of capacity. Hysteresis, because a level that crosses one
  * threshold both ways chatters between rungs and every change costs
  * the filter its settling. */
#define BOARD_DAQ_CLIMB_AT 6U
#define BOARD_DAQ_FALL_AT  1U

/** And a ceiling on that in records, because THE LADDER ANSWERS LATENCY
  * AND THE RING ABSORBS BURSTS - two jobs for one buffer, and only the
  * first should move a rung.
  *
  * MEASURED: the ring went from 16 K to 256 K and the ladder stopped
  * working. Six eighths of 780 records was five seconds of silence;
  * six eighths of 14 000 is seventy, so a host that stopped reading was
  * a minute of backlog before anything adapted - and every record in it
  * older than the last. A backlog is what a rung is for; headroom for a
  * burst is what the rest of the ring is for. 512 records is three or
  * four seconds at the rates this link carries, and the fall mark is an
  * eighth of it for the same hysteresis as before. */
#define BOARD_DAQ_CLIMB_MAX 512U

/** Records at the low mark before a task steps back down. A ring empties
  * the instant a host reads it, so the level alone says nothing about
  * whether the link has caught up - only that it just drained. */
#define BOARD_DAQ_FALL_AFTER 64U

/* ---- THE THERMAL OBSERVER ---------------------------------------------- */

/** How often the model is stepped from the main loop. The fastest node has a
  * time constant of tens of seconds, so 10 Hz is ample and costs nothing. */

#define THERMAL_STEP_MS 100U



/** How often the rail is borrowed for a sample, by default.
  *
  * THIRTY SECONDS, and it was five. The board's time constant is 6.8
  * minutes, so five was 80 samples a tau - eighty rail toggles for
  * resolution nobody was short of, and AFE_ON drives an LED, so it read as
  * the board blinking at itself.
  *
  * Thirty is 13 a tau, still finer than the slowest thing it estimates, and
  * it puts the front end's duty at 0.5 s in 30 rather than in 5. */
#define THERMAL_SAMPLE_EVERY_MS 30000U



/** Settle before the sample is believed.
  *
  * Paired A/B, 12 pairs, 2026-08-28: 500 ms minus 100 ms is +0.005 K, sem
  * 0.008 - 0.6 sigma, under the NTC's 30 mK quantisation. The reference is up
  * before 100 ms. An earlier reading took four samples ALL at 300 ms, saw
  * 50 mK spread and called the settle done; equally early samples agree
  * equally well while being equally wrong. 500 because the margin is free -
  * sampling is refused while the stage is armed. */

#define THERMAL_SAMPLE_SETTLE_MS 500U

/** How long the link may be silent before the HOST's holds are dropped.
  *
  * The host's reference is the one with no lease, so that a session can keep
  * a rail as long as it likes. The cost is that a script which was killed
  * keeps it FOR EVER - measured over and over on this bench, AFE_ON high with
  * nobody using it, warm and blinking, because a run had been interrupted.
  *
  * Silence is the evidence. A session polls, at 2 Hz in a view and far faster
  * in a test; a process that is gone sends nothing. Ten seconds is longer
  * than any gap a live host leaves and shorter than anyone would sit looking
  * at a rail nobody asked for. */
#define BOARD_POWER_HOST_QUIET_MS 10000U

/* ---- POWER ------------------------------------------------------------- */

/** How long a borrowed hold lasts without renewal, milliseconds. */

#define BOARD_POWER_LEASE_MS 3000U

#endif /* BOARD_LIMITS_H */
