# Findings

A record of what was measured on this bench and what it settled. Dated
where the source is. Nothing here is a limit or a verdict - invariant
10 - and nothing here is shortened.

## The AFE and the reference

* PB2 AFE_ON powers the ADC reference (U2 REF2033). Off, every channel
  reads exact mid-scale and the NTC exactly 25.00 C: mid-scale puts the
  divider at R25 by definition. `read_all`, `ntc_temperature` and
  `dcbus_voltage` refuse with the AFE off; `analog_read` labels
  instead.
* The same signal powers the BNO085 and the A1335. Unpowered, the
  BNO085 still drives MISO, resets and advertises - a valid 276-byte
  advertisement reads back - and acts on no write; the wake handshake
  answers sometimes and not others. Every symptom pointed at SPI and a
  day was spent there before the supply was checked.
* PE15 follows AFE_ON inversely. 0 with the front end powered reads as
  a fault asserted, and what drives it is not established.
* The ADC differential offset calibration in `main()` runs with AFE_ON
  low, so it calibrates against an unpowered input: the offsets vary
  by about 100 mV from boot to boot.
* HAL only ORs into PCSEL and never clears it, so every channel ever
  configured on an ADC stays preselected and connected to the sampling
  network. Measured on target: ADC3 PCSEL = 0xC03, channels 0, 1, 10
  and 11 all live at once. Every read path clears PCSEL first
  (invariant 6); a second path, `board_sync.c`, cast the injected JDR
  straight to int16_t and every quiet phase came back near the
  negative rail.
* Phase noise floor with the AFE on: 0.35 to 0.41 A rms per phase
  (`inverter.NOISE_A`).
* Under a full suite run `test_parity.py`'s AFE row failed about one
  run in three: the live side saw the AFE-on row set and the stand-in
  the other, because another suite borrows the rail. The conformance
  suite's borrow is 500 ms every 5 s.

## The link

* USART3 was polled until 2026-08-29 and cost 0.45 % of frames: 1393
  requests, 7 silent, `char_overrun` +7 to match. The cause was the
  IMU poll - a 276-byte cargo at 1.48 MHz is 1.5 ms, longer than one
  character (87 us at 115200) with the FIFO disabled. Every port
  receives on interrupt now.
* The board samples while it waits on the line: a 229-byte DAQ reply is
  19.9 ms of line time, and spinning through it cost the acquisition
  72 % of its rate - measured 2026-09-01, 477 records/s with the link
  idle and 133 while serving it, so a link that could carry 194 was
  fed by a board that could no longer make them.
* CubeMX left USART2 and UART5 at 9 216 000 baud and nothing wrote the
  115 200 everything reported: the wire ran at 80x the number in the
  link report. The baud joined the calibration record at CAL_VERSION 9,
  and `main()` applies it before `link_init` derives the RTU silences.
* Asking a port to echo-test itself put `00 ff 5a a5` in front of the
  reply and the master saw a checksum failure. Refused since.
* A reply read one byte at a time took 17.8 ms for a 20-byte frame that
  arrives whole; whatever is buffered is now taken in one read.
* Stopping a general reply on a valid CRC was measured and rejected: a
  prefix of a 20-byte frame passes about once in 4096, a wrong reading
  every few minutes rather than an error.
* Opening into the window while pyserial lets a port go is `could not
  open port` with nothing wrong: it crashed the suite whenever an
  earlier one had opened a session.
* A port already held by a session cannot be probed: Windows refuses
  the second open and every probe reads silent, which looks exactly
  like a board that has stopped answering. The diagnosis asks the
  session it has rather than opening the port again.
* An unplugged ST-Link read `Voltage: 0.00V` where serial alone only
  ever said silence.
* Three switching runs ended the moment a second session asked the
  board an unrelated question (2026-08-29): `close()` disarmed the
  stage. The broker exists for this; `open()` through a live broker is
  0.05 s against 5.85 s starting one, so the broker lingers 45 s.
* A view that reads in its draw loop runs at the link's pace: on the
  meter bridge a frame spending three round trips took 190 ms of a
  125 ms budget. Probing the port inline on the front page cost 2 029
  ms a frame; probing every port with the board unpowered took 8.4 s.
* Modbus round trips 15 ms apart cannot catch a microsecond pulse: an
  earlier reading of 77 H_INTN highs is retracted.

## Where the write-class transaction's 15 ms goes

Measured on the debug probe's VCP at 115 200:

* t3.5 is 1.75 ms of board-side silence per request, paid until the
  request-length oracle (MINOR 9) proved the shape on its own bytes.
* `QUIET_TIME` was 20 ms, six times the margin the link needs, and
  paid at the end of every transaction: 46.6 ms became 12.9 ms with it
  at 8 ms and the pre-TX gap. A 20-byte reply arrives whole in one
  chunk and a 215-byte one in 175.
* Assigning `serial.timeout` costs 3.25 ms whatever is assigned -
  pyserial reconfigures the port, a control transfer - and the old code
  paid it three times a transaction: 9.75 ms of the 46.6, none of it
  the link.
* The `u8 took` reply is sized (`ACK` shape), so the read stops on its
  last byte instead of waiting out 8 ms of quiet: that was most of the
  write class's 15 ms. An exception frame is always five bytes.
* A compare write lands in 15 ms, about 800 PWM cycles minimum; a
  link-timed 100 ms hold is 93 to 108 ms at the FETs. The counted hold
  (MINOR 8) makes 10 ms exactly 500 periods.

## The gate stage

* 2026-08-27, drivers powered: every duty 1 to 100 %, no supply trip,
  no overruns - all legs equal, so no phase current.
* 2026-08-29: 30 ns of dead time truncated to 7 DTG counts = 29.5 ns
  and the bench supply tripped its over-current protection on a
  dry-switching run. 8 counts is 33.7 ns; the rounding is up.
* Two gate driver stages ran 15 C hotter than the third. What found it
  was a 600-sample pin count and a register dump: the gate pins were at
  CubeMX's LOW speed. VERY_HIGH since.
* Gate short probe, measured on a board with the W pair joined: the
  neighbour follows within 76 ns, against the 4 us a few hundred k into
  the pin capacitance would take. The observing pin sinks through its
  own pull-down of about 40 k.
* 2026-08-30 into a load, about 8 ohm across U and V, DC link 25 then
  31 V, one leg at 2 to 50 % against the other held low, 15 ms to 30 s,
  both directions: 26 runs, break clear under the bypass, 0 overruns,
  no gate shorts, clean disarms; 3.1 to 3.75 A on-time, up to 39 W
  mean in the resistor. `tools/pulse.py` is that test. The board cannot
  measure current while switching on this bench (AFE_ON high unpowers
  the drivers), so the amps are V/R.
* The alternate (op 10) proven 2026-08-30: twelve mid-run state reads
  showing both triples and nothing else, and both half-bridges on the
  scope.
* The counted hold was built 2026-09-02, dry only; no counted hold has
  been scoped.
* The STO interlock reads Cinj 0.77 V and Clevel 0.06 V against 3 V
  each on the unmodified bench board, 2026-08-27. The keepalive latch
  holds a few hundred microseconds.
* The BNO085's wake answers in under a millisecond and then now and
  again not at all - twice in ten over eight seconds, and permanently
  after the part had been left alone for a few minutes; releasing WAKE
  and asserting it again recovers it.

## The caches were off

CubeMX generated neither cache. Measured 2026-08-31:

* One virtual drive step cost 7 400 cycles with the instruction cache
  off; at -O0 the interrupt was 10 040 cycles = 21 us against a 20 us
  PWM period and outgrew it.
* With the instruction cache on and -O2: 6 756 cycles. The step
  called `Board_PhaseAmps` three times.
* Four newlib `sinf`/`cosf` a period were a fifth of the interrupt; the
  polynomial in `drive_math.c` replaced them.
* The board steps the law at 2 922 cycles a period with the drivers
  unpowered (exit ticks 2 921): sample 610, step 1 690, advance 620.
* The data cache stays off: `Board_CalSave` reads the sector back
  through a pointer and `.data`/`.bss` live in DTCM, which no cache
  touches.

## The IMU

Six firmware defects and four hardware hypotheses; none of the latter
survived a measurement.

| Symptom | Cause |
|---|---|
| chip select never moved | configured before `HAL_SPI_DeInit`, which runs the MSP and hands the pin back |
| every read `FF FF FF FF` | CS released between header and cargo, the part restarted the message; also CubeMX's prescaler 32 = 5.94 MBit/s against the part's 3 MHz |
| every read after a reset refused | the advertisement is 276 bytes, the buffer was 64 |
| a sensor enabled at 60 ms never reported | the interval went out little-endian on a big-endian wire |
| a write worked twice, failed the third | gated on an INTN an already-awake part never asserts |
| the four header bytes `00 00 00 00` | NRSTN and BOOTN both driven low at boot by `MX_GPIO_Init`: a part held in reset and strapped for the bootloader |

* Reading without waiting on H_INTN: the advertisement turned up in
  one sample out of six.
* 2026-08-27: reset then Set Feature, 0 rotation vectors; feature
  alone, 49.0 a second. Three empties in a row a couple of milliseconds
  apart is quiet; the write goes after that.
* With a reset's three announcements still queued, every write came
  back SERVER DEVICE FAILURE. `Board_ImuWrite` drains first.
* A write with PS0 left alone fails outright.
* Executable ON, SLEEP and RESET all produced the identical answer -
  the unsolicited product id after every reset, not a reply.
* With the AFE switched on under a part already "ready" the stream
  never started; the same sequence with a reset after it gave 135
  rotation vectors in four seconds.
* 2026-08-29 across an AFE power cycle: the loop came back `running`
  in 0.71 s with feature 5 at 2500 us and pending false, and no report
  arrived in 15 s. Setting the same feature by hand 0.5 s later worked
  every time, which ruled out the part needing longer.
* Hold, reset, Set Feature, resume: the loop absorbed nothing; a hold
  landing mid-staged-reset left NRSTN low. Hold, Set Feature, resume
  with a resume through init: nothing either, the init reset the part
  and threw the feature away. Resume goes back to RUN when the part is
  up and through INIT only when the hold spanned a reset.
* At 388 Hz, 46 frame errors in 30 s, every one id 0x00: a zero byte
  after the last report is padding.
* 130 ms of blocking init inside the main loop was a Modbus request
  that timed out: `fc 0x46: silence`. Draining three 276-byte
  announcements inside `poll_init` was `fc 0x6E: silence` right after
  the rail returned (2026-08-29).
* A cargo per `latest` request cost 45 ms and caught one frame in
  eight; op 8 reads shared memory.
* The pin check reported MISO held by something else: the test's own
  doing, CS floating low asserting the part. Measured 2026-08-29: bits
  11 with CS floating.
* Log ring share: the IMU reports at 50 Hz and the angle loop polls at
  about 24 kHz; with an equal share per armed source the IMU went from
  1 record a second to its full rate.

## The A1335

* Figure 31 names the bit R/W and never says which way round; measured
  on this board, read is 0.
* Two frames per read: asked TSEN, FIELD, TSEN in turn, one frame
  returned the previous register every time.
* TSEN measures its own die and is reset every time AFE_ON breaks:
  2026-08-28 it fell 1.88 K during a run that warmed the board. It
  quantises at 0.125 K. FIELD reads about 2 G with no magnet.

## Thermal

* Camera 2026-08-28, room 20 C, four states held 25 minutes each
  (3.7 tau at 6.8 min = 97 % of the way to equilibrium); the NTC and
  the rises of the bridge, the MCU, the regulators and the AFE:
  passive 30.0 / +15.0 / +8.0 / +1.0 / +1.0; AFE on 31.1 / +14.2 /
  +8.1 / - / +5.9; traffic 31.4 / +13.6 / +7.6 / - / +5.9; switching
  40.0 / +17.3 / +20.0 / +10.1 / 0.0.
* NTC minus TSEN: -0.74 C idle, +10.94 C switching. The NTC overstates
  the switching rise 2.48x.
* A sample is 0.42 s, so every 60 s is 0.7 % of the time in the wrong
  state; four samples 3 s apart spread 50 mK with no drift.
* The camera saw one bridge zone; per leg is three times the lumped
  15.2 K/W and no measurement says otherwise yet.
* The passive state's power came from the supply's own reading,
  0.050 A, which is what `board_to_ambient` rests on.
* The bench suite's regression: the thermal observer reading two ADC
  channels and two SPI transactions on every poll, and before that a
  poll blocking long enough to lose a Modbus character.

### The envelope's arithmetic, 2026-09-03

Computed in `test_thermal_core.py` against the C that will run on the
board - the first time any of it was exercised outside the Python
mirror. Not measured on hardware; the capacities are the calibration
record's and the currents are the rating.

* A 100 A burst in one leg, 48 V, switching: 18.39 W on the driver
  node, 35.00 W on the phase node. **The FET is the binding part, not
  the shunt** - 0.12 J/K against 0.40, so 12.3 J of headroom against
  42.0, and ambient to the 125 C ceiling is **0.67 s on the driver
  node** against 1.20 s on the phase node. The conduction split is what
  made this visible: booked entirely on the phase node, the FET's own
  heat capacity was not in the picture at all.
* `soa_lookahead_ms` is 2000 in the record, and 2000 ms was three
  times the FET node's whole burst budget. **Under the projection that
  first implemented it** - each node stepped forward `lookahead_s` at
  its present rate - it therefore said "over the ceiling" the instant
  full current was asked for, from ambient, and the derate went to 0.00
  before the burst started. The envelope forbade the transient rather
  than shaping it. Changed the same day: the window is time left, not a
  projected temperature.
* Under that projection, what each horizon did to a 35 W phase-node
  burst from 20 C - the clamp from cold, and where it first came off
  1.00. Kept because it is what condemned the shape:

  | lookahead | from cold | first backs off |
  |---|---|---|
  | 0 ms | 1.00 | 1.04 s at 110.2 C |
  | 100 ms | 1.00 | 0.94 s at 101.9 C |
  | 250 ms | 1.00 | 0.78 s at 88.5 C |
  | 600 ms | 1.00 | 0.42 s at 58.1 C |
  | 1000 ms | 1.00 | 0.02 s at 23.5 C |
  | 2000 ms | **0.00** | never runs |

  The knob was not monotone: past about 1 s it stopped being a warning
  and became a refusal.

### The window that replaced it, 2026-09-03

Each node's HOLD - `capacity x (limit - t)` over the net watts, the same
seconds `millis_to_limit` reports - measured against `lookahead_s`. The
fraction `1 - hold/window` joins the temperature fraction and the derate
takes whichever is worse.

* A 100 A burst, every node live, at the record's 2000 ms: the clamp is
  **1.00 from ambient**, starts closing at 0.40 s on `driver_u` at
  83.8 C, and open-loop reaches zero at 0.70 s. The knob is monotone -
  0.5 s backs off at 109.6 C, 1.0 s at 103.9, 2.0 s at 83.8, 4.0 s at
  32.5 - and no window refuses to start.
* CLOSED LOOP, the clamp scaling the current and conduction going as
  I^2, with the firmware's own asymmetric slew (instant down, 0.05/s
  up): 100 A at t=0, driver node to 119.6 C by 3 s, clamp to 0.34, then
  a glide to about **25 A continuous with the driver at 75 C and the
  phase node at 121 C. Never tripped.** That is the burst-then-throttle
  the board is for, and 25 A is what this cooling supports
  continuously - on the model, not on a bench.
* A power the node cannot hold for the window at all is throttled from
  ambient: 200 W into the phase node - some 240 A - starts at a clamp
  of 0.70. That is the rule working rather than a hole in it.
* The threshold is `hold < window x (1 - throttle_at)`, so at 2000 ms
  and 0.85 the ramp occupies the last 300 ms of hold - three
  `THERMAL_STEP_MS` steps. Raising the window lengthens the ramp; it can
  no longer stop the drive.

### A throttle band is only there if something looks inside it, 2026-09-03

Found on the bench, in the rotor observer: the estimator peaked hard and
then collapsed toward zero and never came back. The stage had tripped -
`worst` read 0.756 when the host next looked, because the whole peak
happened between two polls.

* The stand-in integrated a whole poll gap in sub-steps and evaluated the
  envelope ONCE, after the loop. At 10x haste a 0.25 s poll is 2.5 s of
  model time: it ran the lot at full current, went 33 K past a 125 C
  ceiling, and the first evaluation it made had nothing left but the
  trip. Measured: 20.0 C on one poll, 158.6 C on the next, derate 1.00
  then 0.00.
* Three things were wrong with it, and all three had to go:
  * the envelope ran once per gap, not once per step;
  * `STEP_S` was 1.0 s, chosen as a fifth of the fastest node's constant
    - the right rule for integrating and the wrong one for acting, since
    the ramp is 300 ms wide;
  * the drive was sampled ONCE for the gap, so the model went on
    integrating the pre-throttle current after the clamp had closed.
  Fixed, the same 90 A hold gives: clamp 1.00, then 0.52 at 47 A with the
  driver node at 116.7 C, settling at **about 30 A with the phase node at
  119.5 C and no trip at all**.
* **The firmware had the same defect, conditional on main-loop latency.**
  `Board_ThermalPoll` took one step of whatever `since` was - `thermal.c`
  clamps a step at 2.0 s - and evaluated once. At the normal 100 ms it is
  fine; a starved loop loses the band. Measured in the C, a 100 A burst
  over the same two seconds of model time, and where the throttle first
  looked:

  | step | first sees the band at |
  |---|---|
  | 100 ms | driver 81 C, clamp still 1.00 |
  | 250 ms | driver 97 C, clamp 0.65 |
  | 500 ms | driver 99 C, clamp 0.61 |
  | 1000 ms | driver 178 C, clamp 0.00 |
  | 2000 ms | driver 335 C, clamp 0.00 |

  `Board_ThermalPoll` now consumes a late gap in `THERMAL_STEP_MS`
  slices, stepping and evaluating on each, capped at `THERMAL_CATCHUP_MS`
  = 2000. Past that the power sample is too stale to integrate: a model
  fed one reading for two seconds is inventing the heat it did not see.
### The burst budget rests on a number nobody took, 2026-09-03

Traced after reading Silva 2022 (Appl. Sci. 12, 12555) on the transient
response of thermal circuits. `thermal.c` has said it since the campaign:

> Heat capacity. The board dominates: tau 6.8 min against 8.33 K/W is
> about 49 J/K. **The parts' own are not measured** - they respond in
> seconds, below what this rig can resolve, and **only affect the
> settling**.

* The board's 49 J/K is fitted to a MEASURED transient, so it is already
  an effective capacity. The leg nodes' 0.35/3 and 1.20/3 J/K are not
  measured at all - the four camera states were each held 25 minutes,
  which is equilibrium, so nothing in the campaign could see a leg's time
  constant.
* **The last clause is no longer true.** The envelope divides by exactly
  those numbers: `soak_j` is `capacity x (limit - t)`, `hold_seconds` is
  that over the net watts, and the throttle's reaction window is a
  multiple of it. Every burst figure above rests on them.
* Silva's Eq. 12-14 bounds how wrong: a lumped element's EFFECTIVE
  transient capacity is `gamma C` with `gamma = 1/3` less a negative term
  per contact with a better conductor, because heat crosses a distributed
  body in one direction. If 0.35 J/K was a guess at the PHYSICAL
  capacity, the transient one is up to three times smaller; if it was
  already a guess at the effective one, it stands. Nothing on record says
  which, so the honest answer is a band. Measured in
  `test_thermal_core.py`:

  | driver node capacity | soak at 100 A | burst to the ceiling | throttle first acts |
  |---|---|---|---|
  | 0.1167 J/K *(on record)* | 12.25 J | **0.67 s** | 0.40 s |
  | 0.0389 J/K *(x gamma)* | 4.08 J | **0.22 s** | **0.00 s** |

* THE TWO UNMEASURED NUMBERS ARE COUPLED. At gamma the node's whole hold
  from ambient is shorter than `soa_lookahead_ms` = 2000, so the clamp
  closes from a cold board and the 100 A burst is forbidden outright -
  the same failure the old temperature projection had, arrived at from
  the other side. A reaction window is only sane against a capacity that
  is known.
* WHAT SETTLES IT, and it is the only soft number a TRANSIENT can reach
  rather than an equilibrium: a power step and the NTC's slope. With the
  coupling at one the thermistor reads the leg lump, so `dT/dt` right
  after a step is `P / capacity` outright - no camera needed, and
  `tools/pulse.py` already makes the step.

### The NTC coupling is one point, stretched ten times, 2026-09-03

Raised at the bench: the NTC runs away as soon as the stage switches, and
the thermistor is not really that close to the switch nodes. The
arithmetic says the doubt is the right one.

* `NTC_OFFSET` = 36.0 - 30.0 = **6.0 K**, from the passive state where
  nothing was warming anything. That one is fine - it is a mounting and
  channel offset taken where there is no driver term to confuse it.
* `NTC_SEES_DRIVERS` = ((55.6 - 40.0) - 6.0) / 9.1 = 9.6 / 9.1 =
  **1.055**, and that is a slope FITTED FROM ONE POINT, at a driver rise
  of **9.1 K**. Both terms in the numerator are camera readings; +/-0.5 K
  on each gives a slope anywhere from **0.95 to 1.16**.
* The demo, and any real burst, drives the driver node 70-100 K over the
  board - **ten times the rise the slope was fitted at**. Extrapolated,
  the NTC reads +79.8 K over the board at a 70 K driver rise, and the
  camera error alone spreads that over 72 to 88 K. It is the fastest
  moving number on the page and the least supported.
* A slope of 1.055 says the thermistor tracks the driver node one for
  one - thermally ON it, not a few millimetres away on the laminate. The
  note beside the constant rationalises that as "closer to the heat than
  the point the node stands for", which may be true; it may equally be
  the fit absorbing a board GRADIENT, since `board` in that state is the
  camera's reading at one spot and the copper under the thermistor need
  not be that spot. Nothing distinguishes the two from one point.
* Not changed. The constant comes off the campaign's own measurements and
  there is no measurement that says otherwise (invariant 10). What would
  settle it is a camera run at a driver rise of tens of kelvin - the same
  bench day that would span `board_to_ambient` at high dT and `to_board`
  per leg. All three of this model's soft numbers are fitted at one tenth
  of the load the board is rated for.

**Test vectors, 2026-09-03.** Run on the lumped model after the bench saw
the NTC spike ABOVE the switch temperatures.

* The model is `NTC = board + c (driver_v - board) + k` with c = 1.053
  and k = 6.0, so `NTC - driver_v = 0.053 x rise + 6.0` - **positive at
  every rise, by construction**. Swept: +6.0 K over the driver node at
  rest, +6.5 K at the 9.1 K rise it was fitted at, +9.8 K at 70 K, +11.5 K
  at 100 K. A passive sensor cannot be hotter than the thing heating it.
* It is already wrong AT THE FIT POINT. The camera saw the NTC 15.6 K over
  the board in the switching state while the model's driver node rose only
  9.1 K, so the fit says the sensor sat 6.5 K above its own source before
  anything was extrapolated.
* ONE MEASUREMENT, TWO UNKNOWNS. That state fixes only the product:
  `c x to_board = 48.0 K/W`, since the driver's switching share is
  0.20 W. Every pair on that curve fits the camera exactly:

  | to_board | coupling | driver rise at fit | NTC - driver at fit | NTC when a driver is at 100 C over a 45 C board |
  |---|---|---|---|---|
  | 45.6 K/W *(current)* | 1.053 | 9.1 K | **+6.5 K** | 108.9 C (+9 K) |
  | 60 | 0.800 | 12.0 K | +3.6 K | 95.0 C (-5 K) |
  | 78 | 0.615 | 15.6 K | 0.0 K | 84.8 C (-15 K) |
  | 100 | 0.480 | 20.0 K | -4.4 K | 77.4 C (-23 K) |
  | 150 | 0.320 | 30.0 K | -14.4 K | 68.6 C (-31 K) |
  | 250 | 0.192 | 50.0 K | -34.4 K | 61.6 C (-38 K) |
  | 400 | 0.120 | 80.0 K | -64.4 K | 57.6 C (-42 K) |

  **`to_board` must exceed 78 K/W for the sensor to sit below its source
  at all.** The campaign fixed it at 45.6 - itself three times a lumped
  15.2 the camera saw once - and solved for the coupling; fixing the
  coupling at something physical and solving for `to_board` fits the same
  measurement just as well. The bench's own expectation, the NTC 40-50 K
  below a 100 C local hot spot, lands at 250-400 K/W with a coupling of
  0.12-0.19.
* THE LIKELIEST READING: the model has no board GRADIENT. `board` in that
  state is the camera at one spot and the copper under the thermistor need
  not be that spot; a fit with nowhere else to put the difference puts it
  in the coupling. Six kelvin of local gradient at the fit point would
  take the coupling to about 0.4 and leave `to_board` alone.
* Also structural: the modelled NTC has NO TIME CONSTANT of its own. It is
  algebra on the driver node, so it follows a fast silicon node one for
  one, where a thermistor in copper is a low pass with the copper's own
  mass behind it. A coupling well below 1 gives that for free - the
  reading becomes mostly the slow board node - which is the same fix, from
  the other end.
* `DRIVER_RISE_SWITCHING` was a bare 9.1 written beside the two numbers it
  is the product of. Derived now (`DRIVER_SWITCH_WATT * LEG_TO_BOARD`), so
  the dependency is where a reader will trip over it: the coupling is
  solved against a fitted number, not a measured one.

### Conduction was one sample squared, 2026-09-03

Found auditing the model after the bench asked why the switches were not
the hottest thing on the page. They are, at any real current - at 20 A
rms a phase the driver nodes settle at 187 C against the MCU's 118 - and
the demo simply runs a few amps, where 1.33 W of housekeeping genuinely
dominates. But the audit turned up a real defect beside it.

* `load_now` handed `thermal_power_estimate` ONE synced sample per
  `THERMAL_STEP_MS` and it squared it. A single instant of a rotating
  three-phase current says where the vector is pointing, not how big it
  has been: measured in the core, a sample at the peak claims 35.00 W
  where the true loss is 17.50, and a sample at the zero crossing claims
  none at all.
* Unbiased over a uniform phase, and the node's own 5 s constant filters
  50 samples - so the TEMPERATURE was tolerable. The ENVELOPE was not:
  `hold_seconds` divides by that same power, so a sample near a crossing
  reads as "not heading anywhere warmer" and the throttle sees no
  pressure in the step where it matters.
* And the sampler is SYNCHRONOUS - the trigger is a tick inside the PWM
  period - so this is worse than a coin toss. At a speed whose electrical
  period divides the poll interval the alias LOCKS, and a leg carrying
  its peak reads as a leg carrying nothing for as long as the speed
  holds. A leaky average over the polls cannot cure a locked alias; only
  accumulating at the sample rate can.
* `Board_SyncMeanSquare` accumulates sum and sum-of-squares per leg in
  the injected callback - three integer multiply-accumulates, in COUNTS,
  so the interrupt does no floating point - and undoes the affine
  conversion once per read from two evaluations of `Board_PhaseAmps`,
  which keeps what a count is worth in one place (invariant 7).
  `thermal_load_t.phase_sq` carries it; zero means not measured and the
  estimator squares the sample as before. The ISR cost is UNMEASURED -
  `test_bench.py` at the bench is what would confirm it.
* Per leg and not a three-phase sum, deliberately. For a balanced set the
  three squares sum to a constant and could be shared out, but this board
  also drives one leg against another - `tools/pulse.py` is exactly that
  test - and spreading U's heat over an idle W would be a model that
  could not represent its own bench test.
* Two things checked and found NOT wrong on the way: the host's
  `phase_power` and the C agree watt for watt once both are given the
  same rms (the earlier disagreement was peak against rms in the
  comparison itself), and the host's `POWER_SWITCHING` regulators entry
  of 1.134 W is exactly the C's `ldo_watt` 0.534 plus the non-driver half
  of the 1.2 W switching loss. No watts are lost between them. The host
  gives the AFE 0 W where the C gives 0.130, and that is right: the AFE
  only draws with AFE_ON high, and AFE_ON high unpowers the gate drivers,
  so `switching` and `afe_on` are not a state this board can be in.

### Is 70-80 C on the board plausible? 2026-09-03

Asked at the bench after the rotor observer showed it. Arithmetic on the
record's own constants, no measurement.

* `board_to_ambient` is 8.33 K/W and it rests on ONE point: the passive
  state, 1.2 W from the supply's own 0.050 A, the camera at 30 C in a
  20 C room. 10 K over 1.2 W.
* Linearly, then: the board node needs **6.00 W to reach 70 C** and
  **7.20 W to reach 80 C** (10.20 W is its 105 C ceiling). Settled in
  the model that is about 13-15 A rms a phase - and at that current the
  driver nodes are already 120-135 C, past their own ceiling, so the
  throttle acts long before the copper gets there. Switching alone at
  48 V with no phase current is 3.67 W and settles the board at 50.6 C.
* **The linear extrapolation is pessimistic and this is where the 70-80
  came from.** Natural convection has h proportional to dT^0.25, so the
  resistance falls as it heats: solving dT = P * 8.33 * (10/dT)^0.25
  gives dT = (14.8 P)^0.8, which is **56 C at 6 W and 62 C at 7.2 W**,
  not 70 and 80. Radiation is not small at those temperatures either -
  about 0.012 m^2 of board at 60 C into a 20 C room is roughly 3 W at
  emissivity 0.9, comparable to the convection, and almost none of it
  was present at the dT = 10 K where the constant was taken.
* So the model errs toward safety: it will throttle earlier than the
  copper requires and it overstates a reading. It is not a temperature
  to trust as a measurement above about 40 C.
* The softer number is worse. `to_board` at 45.6 K/W a leg node is NOT a
  measurement - the camera saw one bridge zone, and per leg is three
  times the lumped 15.2 K/W, recorded above with "no measurement says
  otherwise yet". It sets every steady-state current figure here: 18 W
  through 45.6 K/W is 821 K over the copper. The transients are better
  grounded, being governed by the capacities.
* What would settle both: a camera run under real load, spanning
  `board_to_ambient` at high dT and `to_board` per leg.

### SWITCH TEMPS below BOARD TEMPS is the label, not the model, 2026-09-03

* Read at the bench as a broken observer. It is not: idle, every driver
  and phase node settles at **31.08 C, which is the board node exactly**,
  and a node below the copper cannot happen - `thermal_step` sheds
  `(t - board) / to_board`, so it takes a negative shed and is pulled
  back up.
* The right gutter is four nodes and its hottest is almost always the
  **MCU at 46.06 C**, 0.666 W through a linear LDO, 15 K over a copper at
  31.08. The caption said BOARD and reported that. The two figures were
  not comparable at all.
* Reporting the copper instead was tried and WITHDRAWN. It bought the
  ordering a reader expects and broke something worse: the figure then
  disagreed with its own gutter, saying 20.9 C under a stack whose
  tallest tube was the regulators at 33.7 C.
* What actually fixed it was the TUBES. Each was a share of its own
  ceiling, and the ceilings differ - the copper's 105 against the
  silicon's 125 - so two tubes at one height were two different
  temperatures and the two gutters could not be compared at all. Height
  is degrees on one scale now and colour is the margin against each
  node's own ceiling, so a copper at 100 C goes amber where a FET at
  100 C has not. With one ruler the surprise stops being one: the MCU
  tube is visibly the tallest and the caption names it.

* A false lead on the way, recorded so it is not chased twice: the
  stand-in's clamp appeared not to bind - `derate 0.25` left the sampled
  current at 77.94 A - and that was the measurement's own fault.
  `Drive.set_params` takes **SI**, so `drv_i_max_ma=90000` asks for
  90 000 A, not 90 A. The units are in the name and the value is not.
* `THERMAL_STEP_MS` is 100, so a horizon under 100 ms cannot see past
  its own step. The band that both shapes a burst and outruns a poll is
  a few steps wide.
* The lookahead is a feedback loop, not a gate: the clamp scales the
  current, which lowers the power, which lowers the projection, so it
  settles where the horizon lands on the ceiling. `board_thermal.c`
  slews the recovery at 0.05/s, which is what stops it chattering
  there - measured on the stand-in at 0.25/s, it oscillated 1.00 to
  0.00 every 100 ms against the node's 18 s constant.

## The DAQ

* The reader thread: 84.4 to 134.6 records/s with 4 ms of work a
  block. At the bottom of the ring, 95 reads/s at 1.00 records each;
  waiting for a reply's worth is 31 reads/s at 4.00 records = 124.8
  records/s.
* A terminal that stopped drawing for six seconds overflowed a 16 K
  ring: 334 records. The ring is 448 KB in AXI SRAM now.
* `interval_us` 0 with `records` 0 took the link down and is the one
  combination refused.
* A tone burst costs 440 cycles a sample.
* The adaptive ladder: with the reader thread draining continuously,
  0 moves where the same run made 6 before the reader existed.
* A ramp's sum over a window is piecewise linear in where the window
  starts, so several starts give the same total - three, over 8192
  searched. The integrity check asks whether there is one place every
  record is exactly right.
* A relative error is meaningless at the filter's first outputs:
  1.3e-3 at record 0 against nothing wrong; the comparison is in codes.
* The 1.5-cycle sampling time is ruled out as a cause on the quiet
  channels: the 15 nF node capacitor supplies the sample-and-hold
  charge. It is not ruled out for Cinj and Clevel, whose apparent duty
  tracks the sample rate.

## Clocks

* A host clock is not a reference, and a Windows one reporting a good
  sync is not either: an offset inside `MaxAllowedPhaseOffset` is
  slewed rather than stepped, so it can sit most of a second out and
  drift on top of that. `set_time_from_pc(reference='utc')` measures
  the host against NTP over the same window and takes out both the
  offset and the rate; with no route it falls back to `'pc'`, which
  ties the board to this machine as it stands, and says which it did.
* CYCCNT wraps every 9.04 s at 475 MHz; the elapsed arithmetic is done
  in raw ticks (invariant 2).
* The stand-in's clock runs 12 ppm slow on purpose, so a sync has
  something to measure.

## Calibration

* 2026-08-30 the DC link was spanned against a DMM: 31.04 V read,
  30.05 V true, -32 418 ppm on channel 5, saved. It is the one number
  measured against an instrument.
* The phase gain was traced off the schematic 2026-08-26, so the
  phases report amperes; they have not been spanned.
* An id added without moving `BOARD_CAL_PARAM_COUNT` is a field the
  board holds and never reports: `deadtime_ns` read back as absent from
  a record that had it.
* Nine ADC rows overflowed 0x42's single reply (seven came to 197
  bytes, nine to 254); the table pages. The reserved pin list grew from
  7 to 19 rows = 418 bytes and the parts list past 253: both page.

## The local model

* Asked for raw codes with the AFE deliberately off, a model wrote
  "Mid-scale ... 25.00 C" out of the warning text itself.
* A local model reported a coaxial cable or connector, twice.
* `ch=['phA']` was guessed; BUS_VOLT and A0 were invented; "vänster
  knä" was sent as `right knee`.
* Asked to measure, gemma4:12b read HARDWARE.md and answered with its
  channel table.
* gemma4:12b denied being able to flash with `build_firmware` in its
  list; the build's first two tries were `python3` and the wrong
  directory.
* `link_diagnose` ran first on three questions not about the link.
* qwen2.5:14b turned the AFE on four times in one turn.
* qwen2.5:14b answered in Chinese, Japanese and Thai; locked to Korean
  it refused to leave, in Korean.
* llama3.1:8b invented tool arguments.
* Asked what had been ruled out about the phase V offset, qwen2.5:14b
  found the entry and reported a dead end, because the hit carried
  the entry and not the chapter; `find` reports both.
* Reloading the weights was most of a run's wall time, so the model is
  loaded once per run and released once. A run killed from outside
  leaves them resident until something releases them by hand, which is
  what the `finally` and `keep_alive` 0 are for.
* The reserve cannot be a flat fraction of the card: a desktop holds
  VRAM at idle, and a quarter of a 16 GB card left too little slack
  behind it. It is the largest of a quarter, 2 GB, or what is already
  in use plus 2 GB.
* llama-server's prompt cache and its context checkpoints each
  allocate hundreds of megabytes beside the weights, and restoring a
  checkpoint threw `std::bad_alloc` and took the runner with it. Two
  copies of the weights on one card is a 500, `cudaMalloc failed`.
  Both allocators off and one model, one context: ten questions,
  twenty-seven calls, no `std::bad_alloc`, one load.

## The renderers

* Decimation cost at 94x36, single process: grid 16 → 7.8 ms, 24 →
  13, 32 → 29, 48 → 37, 64 → 63.
* A shadow map rebuilt every frame cost 14 ms and the frame rate fell
  from 160 to 49.
* The fitted shadow threshold 0.24 exceeded every measured occluder
  gap (max 0.235 across five poses) and cast shadows never fired.
* Moving the camera from 90 to 60 degrees darkened the whole board
  before the light was made the world's, not the camera's.
* In Rec.709 luma against the exporter's screenshots, '.' cells sit at
  93 to 99 and ':' at 128 to 130; the outline lift is 4.5 from 2.5.
* The slab's top is measured from the mesh, not assumed at z 0: the
  export centres on its bounding box, and a gate a millimetre over the
  measured top shows 44 loops wider than 0.12 units. 914 of the 1 458
  edges in the 95 loops drawn were under half a cell face-on.
* Zoom 1.0 fits the bounding sphere at any attitude, 56 % of the box's
  width; 2.0 is the first zoom that reaches every edge.

## Ruled Out

Hypotheses investigated and settled, so they are not investigated
again. A hit from `docs(find=)` carries this chapter's name beside the
entry for that reason.

### PCSEL accumulation explains the Phase V offset

Ruled out. PCSEL accumulation is real (ADC3 PCSEL 0xC03, four channels
live at once) and every read path clears it, but it is not what the
Phase V offset is.

### The NTC channel is not anomalous, it is quiet

The 15 nF node capacitor supplies the sample-and-hold charge, so the
1.5-cycle sampling time is ruled out as a cause on the NTC and the
other quiet channels. It is not ruled out for Cinj and Clevel, whose
apparent duty tracks the sample rate.

### The rest

* The two hot gate driver stages were not a hardware fault: the gate
  pins were at CubeMX's LOW speed.
* JTAG connect-under-reset failing with `Unable to get core ID` is the
  probe firmware, not the board; the cabling was proven fine.
* The BNO085 needing longer after a power cycle: setting the same
  feature by hand 0.5 s later worked every time.
* The four BNO085 hardware hypotheses: none survived a measurement;
  the six causes were firmware.
* MISO held by something else on the IMU bus: the check's own chip
  select floating low.
* The board halted after two sessions held one COM port: it was the
  port, opened twice.
* A gap after the last SHTP report is padding, not a frame error.
* Stopping a reply on a valid CRC: a prefix passes once in 4096.
* The IMU's H_INTN never asserting: it asserts; the 77 highs were an
  artefact of 15 ms round trips.
