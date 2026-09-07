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
### The datasheet was in the tree the whole time, 2026-09-04

Three numbers said to need a bench day are in
`datasheets/mosfet/IAUCN10S7N021-Datasheet.pdf` Rev 1.2, and two of them
change what we thought.

* **Rth JC 0.69 K/W max** (p.4). At 100 A a FET carries its half of the
  period, about 9 W, so the junction sits **6.2 K** over its own case.
  The die the network has no node for is therefore a few kelvin, not
  tens: a 125 C ceiling on the copper is about **131 C at the junction
  against the sheet's 175 C limit**, 44 K of margin. **The ceiling is
  CONSERVATIVE, not optimistic** - the opposite of what was written here
  before the sheet was looked up, and that correction is the point of
  looking things up.
* **Rth JA 25.9 K/W typ**, on a JEDEC 2s2p FR4 board, vertical, still air.
  One FET's WHOLE path to air. The model's own path for one leg is
  `to_board` 45.6 plus `board_to_ambient` 8.33, about 54 K/W - so **the
  model's spreading term alone is 1.8x the datasheet's entire
  junction-to-air**, on a board carrying heavier copper than 2s2p. They
  cannot both be right.
* **Rds(on) 1.8 mOhm typ against 2.1 max** at Vgs 10 V. The model books
  the typical, so the envelope under-books a worst-case part by 17 %.
  Flagged and not changed: the LTspice model this tree traces is the
  typical one, and the two would then disagree.

**AND THAT SETTLES WHICH CAMPAIGN INPUT IS WRONG.** Three lines of
evidence about `to_board`, pulling two ways:

| evidence | says about the leg's spreading resistance |
|---|---|
| camera, one bridge zone at 15.2 K/W lumped, tripled per leg | 45.6 K/W |
| the NTC's own rise, if it is to sit below its source | **above 48 K/W** |
| the datasheet, whole junction-to-air on a lesser board | **well under 25.9 K/W** |

The datasheet and the NTC point in opposite directions, and the
datasheet is a characterised measurement on a defined board while the
NTC constraint rests on the camera's board reference in the switching
state. So the odd input is the one already under suspicion: **the
camera's `board` 40.0 C**, read off mixed copper and soldermask through
an emissivity nobody corrected. If the copper under the thermistor was
really nearer 46 C, the NTC's rise above its LOCAL board is 3.6 K rather
than 9.6, the fraction falls to about 0.24, and the model's `to_board`
can come down toward the datasheet instead of up away from it.

Not retuned here. Moving `to_board` moves every steady-state current
figure and the whole SOA behaviour, and that is a bench decision - but it
is now a decision with three numbers behind it instead of one.

### The placements settle two of the guesses, 2026-09-04

`electronics/Coaxial 63100 Pick-Place.csv` arrived, and it is the
authority on where things are the way the parts list is on what is
fitted. NTC1 sits at (99.62, 79.83) mm; every distance below is to it.

| part | mm | what the thermistor sees of it |
|---|---|---|
| U1V, the V gate driver | **8.2** | 0.50 |
| Q2V, a V half-bridge FET | 15.1 | 0.33 |
| Q1V, the other | 17.7 | 0.28 |
| U1W, the next driver | 28.0 | - |
| RV1, the nearest shunt | 29.7 | - |
| U1U, the far driver | 30.3 | - |

* **`THERMAL_NTC_NEIGHBOUR = driver_v` is CONFIRMED.** U1V is the nearest
  power part by a factor of 3.4 over the next driver. It was an
  assumption until the file arrived.
* **The element fraction is 0.30, not 0.50, and it is geometry now.**
  Two-dimensional radial spreading in a plate gives
  `f = ln(R/r) / ln(R/a)`, with R half the short side of the placement
  extent (46 mm) and `a` a package's own radius (1.5 mm). The old 0.5 was
  right for the DRIVER IC alone - and wrong for the node, because the
  model lumps the driver's switching loss and both FETs' conduction onto
  one lump while the thermistor is 8 mm from one and 15 to 18 mm from the
  other two. At 100 A the FETs make 18.4 W of that node's 18.6, so the
  fraction is theirs: power-weighted, **0.304**.
* What it does to the reading: a leg node 100 K over the board now shows
  the thermistor 70 K BELOW it rather than 50. The campaign residual
  moves 11.04 to 12.86 K, which is the same inconsistency seen from a
  slightly different fraction and not new information.
* THE OFFSET DOES NOT MATTER. The exporter's origin is shifted, and every
  quantity used is either a distance between two parts or the extent of
  the whole set - both differences, so a constant shift falls out.
* R IS A FLOOR. The extent is the parts' bounding box, not the board
  outline, so the real R is larger and f slightly higher: at R = 55 mm
  the weighted fraction is 0.34 rather than 0.30.
* `test_sensorless.py` reads the file and checks both claims, so a board
  revision that moves the thermistor fails the suite rather than the
  bench.

### A more plausible lumped model, 2026-09-04

From `docs/papers/`: Ziegenfelder 2022 (USU) for the heat-transfer form,
and the PCBA compendium for what a lumped R-C network is worth and what
radiation carries.

**THE PATH OFF THE BOARD IS NOT A CONSTANT.** Free convection carries
`h = Nu k / L` with Nu a power of the Rayleigh number, and Ra is linear
in the rise, so h goes as about the fourth root of it (Ziegenfelder Eq.
2.4-2.6: `q = h A dT`, `Gr = (g/nu^2) beta dT P^3`). Radiation carries
`h_rad = eps sigma (T^2 + T0^2)(T + T0)` (Silva Eq. 5), which grows
faster still. The model held both frozen at the one rise the campaign
measured - 1.2 W over 10 K - and then asked about loads putting sixty
kelvin on the board.

* `thermal_board_to_ambient_at` scales the calibration value by how much
  better the two mechanisms carry at the present rise. Everything else -
  area, emissivity, fluid properties, characteristic length - stays
  inside the calibration value, so the measurement is reproduced exactly
  at its own point and only the shape away from it is the correlations'.
* The split at the calibration point is **35 % radiation**, from the
  compendium's "stralning star for 30-40 % av den totala
  varmeavledningen vid passiv kylning och kan inte forsummas". It is
  needed because the two shapes differ, so only their proportion lets
  them be scaled apart.
* What it does:

  | rise | K/W off the board | board temperature | needs, flat | needs, now |
  |---|---|---|---|---|
  | 10 K | 8.33 *(the calibration point)* | 30 C | 1.20 W | 1.20 W |
  | 20 K | 7.30 | 40 C | 2.40 W | 2.74 W |
  | 40 K | 6.28 | 60 C | 4.80 W | 6.37 W |
  | 60 K | 5.68 | 80 C | 7.20 W | 10.56 W |
  | 85 K | 5.24 | 105 C *(ceiling)* | 10.20 W | 16.50 W |

  So the copper needs **8.40 W to reach 70 C** where the flat model said
  6.00, and the earlier hand estimate of "about 56 C at 6 W" against the
  flat model's 70 comes out of the model itself now.
* `steady()` iterates rather than multiplies, since the rise is implicit
  in its own resistance. A handful of passes: the resistance moves as a
  fourth root, so the fixed point is a gentle one.

**AND THE READING IS SLOW.** `NTC_TAU_S` was the leg node's own 5.32 s,
which made the modelled thermistor exactly as quick as the thing it
watches - the one speed it cannot have, since the SOA acts on silicon in
0.22 to 0.67 s at 100 A. It is the GEOMETRIC MEAN of the pair the element
sits between now, 5.32 s and 408 s, so **46.6 s**: the log-midpoint, which
is what "between" means for a time constant. Measured on the stand-in at
a 70 A hold, the hottest switch node reaches 117.5 C in the first model
second while the reading is at 29.9 - **trailing by 88 K** - and it is
still 69 K behind six seconds later.

The compendium also sets the error bar this whole model class carries:
lumped R-C is **±10 %**, against ±5 % for a Fourier hybrid and ±2 % for
full 3D CFD. Our unmeasured constants are far outside that, which is
worth saying beside any number this model prints.

### The thermistor becomes an element, 2026-09-04

Implemented from Silva 2022 (Appl. Sci. 12, 12555), whose form is that
every thermal object is a resistance and a heat capacitor in parallel and
objects join into a network. The thermistor is now one such object, tied
to the leg node on one side and the board on the other.

* **It cannot leave the interval between them, at any parameter value.**
  Its steady state is `board + f (leg - board)` with f clamped to [0, 1],
  which is a weighted average. That is the property the old form could
  not have: `board + c x rise + offset` with c fitted at 1.055 and an
  additive offset put the sensor above its own source at every load -
  6.0 K over at rest, 11.5 K at a 100 K rise - and capping c at one left
  the offset still doing it.
* **The 6.0 K offset is no longer a temperature.** It is the passive
  state's disagreement between a thermistor and a CAMERA, and the camera
  is the instrument reading mixed copper and soldermask through an
  emissivity nobody corrected. It is recorded, not applied, and both
  inversions - `thermal_board_from_ntc` and the NTC anchor in
  `thermal_step` - dropped it.
* f is **0.5 and not measured**, and the campaign cannot measure it: its
  one switching state implies 9.6 K of thermistor rise against 9.12 K of
  leg rise, a fraction of 1.05. A point sensor soldered to FR4 a
  centimetre from the pad is somewhere between a tenth and two thirds of
  the way; this is the middle of that, and no value of it can produce an
  unphysical reading.
* THE PRICE, and it is recorded rather than hidden: the campaign's
  switching state now misses by **11.04 K**. The inconsistency was always
  there - it had been living inside the coupling, which is what made the
  coupling impossible. Which of the three inputs is wrong is still open:
  the leg's spreading resistance (three times a lumped figure the camera
  saw once), the driver's share of the switching loss, or the camera's
  board reference.
* Measured on the stand-in, 60 A hold: the hottest switch node settles at
  116.5 C, the board at 33 C, and **the NTC reads 58 C - 57 K below the
  switches and 25 K above the board**, which is the picture the bench
  described from the start.

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

## The SOA envelope

* **A throttle weighed nodes it could not cool.** `thermal_budget`'s
  worst node ran over all ten, and three of them - MCU, regulators, AFE -
  draw the same watts at zero duty as at full. Measured on the stand-in
  2026-09-04, an idle board with nothing switching settles at 49.1 C on
  the MCU and 51.1 C on the regulators; against a 125 C ceiling from a
  20 C ambient that is 0.30 of the budget spent before the stage has done
  any work, and no derating can lift it. The page read 38 % of the
  board's SOA gone on a lukewarm bench. Fix: `soa_undriven_mask` in the
  calibration record (CAL_VERSION 11) marks the three, and the throttle,
  the ramp and `millis_to_limit` skip them. `used`, `soak_j` and
  **`tripped` still span every node** - a regulator at its ceiling is a
  stop whatever a clamp could have done about it. Idle worst is now the
  laminate at 0.003; at 30 A rms it is `phase_u`, as it should be.
* The laminate is NOT masked: the legs are most of what heats it, so the
  clamp moves it and it belongs in the throttle.
* Steady state at 5.30 mOhm a phase, from `coaxial.thermal`: 20 A gives
  board 71.7 C / driver 97.5 / phase 110.9; 30 A gives 105.8 / 156.8 /
  194.0. Continuous against a 105 C laminate is 19.1 A, against a 125 C
  junction 22.0 A. The shunt node binds first in steady state - the
  board's ceiling is not what limits a continuous rating.
* **The throttle point moved from 85 % to 90 % of the span, 2026-09-05,
  on the bench's word.** `THROTTLE_AT`, the record's `soa_throttle_ppm`
  (900 000), the stand-in and the notebook builder all carry it. With
  the two-second lookahead the ramp is now the LAST 200 ms of hold
  rather than 300 (`lookahead x (1 - throttle)`), which is what moved
  three checks in `test_thermal_core.py` - none of them a change of
  rule, all of them a scenario that sat inside the old band and under
  the new one. Measured on the core (phase node 0.400 J/K): 200 W from
  ambient holds 0.21 s and just clears a 0.2 s ramp, clamp 1.00, where
  it was throttled at 85; the fault check is 300 W now, clamp 0.70.
  35 W on a node placed hot: the window binds from about 110 C
  (100 C before) - 108 C still 1.000, 110 C 0.944, 112 C 0.820 against
  a present-only 1.000, so the check sits at 112. And an unmasked
  regulator at 110 C is 0.857 of its span, under the point; the check
  places it at 116 (0.86 unmasked, 1.00 masked). **The one that was
  wrong before the move:** `screen.gauge` turned sodium at a literal
  `hot=0.85` of its own, a second copy of the throttle point that
  would have stayed at 85 while the board acted at 90 - it defaults to
  `THROTTLE_AT` now. What the notebooks print (`foc_montecarlo`,
  `thermal_model`) still quotes 85 until they are re-executed; the
  builder says 90.

## The thermal identification, 2026-09-05

Built and measured on the host against a ground truth the identifier
does not know: the same twenty-node graph with its air path scaled -
2.0 for a board in a box, 0.5 for a fan - read through the NTC and the
two dies with ±0.05 K of noise every thirty seconds of every cooldown
and never during a run (AFE_ON is low while anything switches). Three
cycles of a ten-minute 20 W run at the legs and twenty minutes of
cooling. `test_thermal_core.py` holds the result; `python
tools/thermal_ident_trace.py [cycles] [air]` prints every innovation
with its regressor and the scales after it. Nothing below was found by
looking at the estimate alone.

* **The observer ran open loop on the board, and had since the sampling
  went to thirty seconds.** The anchor's pull was `THERMAL_ANCHOR_HZ *
  dt_s` - 0.05/s sized as if a reading came with every step - and the
  board reads its three thermometers every 30 s into a 100 ms step:
  0.5 % of the residual per sample, "a wrong initial guess gone in a
  minute or two" was a hundred minutes. Found because the identifier
  inherited the observer's state and charged its error to the scales.
  Now `1 - exp(-HZ * seconds since the last sample)`, 78 % at thirty
  seconds, 5 % per step when a reading comes every step - the old
  figure in the old case; `since_seen_s` in `thermal_t`.
* **The thermistor's anchor inverted a lagged element as if it were
  instantaneous.** `over = reading - board`, divided by `ntc_sees` to
  place the V patch: the element lags the laminate by 215 s, reads
  4 K above the average during a cooldown at these rates, and the
  patch was placed 14 K hot at every sample. Now the miss is against
  the MODELLED element, the element is anchored to what it reads (it
  is its own temperature), and the patch takes the miss through the
  share AND the lag - `0.5 tau / interval`, one to eight - so 39 % of
  the patch's error a sample at thirty seconds where the share alone
  took a ninth (leg patches 20 K cold at the third sample). Held at
  the leg - the element within 2 K of the V patch, read or modelled -
  the miss is the patch's one for one; the share's inverse there was a
  gain of 2.6 a sample on a reading that IS the patch and the patch
  swung ±25 K sample to sample. Only a miss that grew over one
  interval (≤ 90 s since the previous sample) is read as the patch's:
  after a blind run it is the element's own state, 33 K of it, and
  inverting that put the V patch at 293 C on a 220 C truth.
* **The laminate the thermometers do not reach moves with the ones they
  do.** The mean of the dies' patch corrections goes to every board
  node no thermometer anchors, and the V patch's correction to U's and
  W's - one layout mirrored. Without it the legs' patches kept a run's
  whole error through the cooldown (obs 178 C against 242 true at the
  end of a run on the wrong air scale) and the identification read
  their flow into the centre as a parameter: 3 to 5 K of innovation at
  the first samples, absorbed to the clamps.
* **A die's node lags its patch, and the patch was anchored hot by the
  lag.** `patch = node - P R` is the steady algebra; the package's is
  `C dT/dt = (patch - node) / R + P`, so `patch = node - P R + C R
  dT/dt`, and at 0.08 K/s the MCU package (τ 20 s) rides 1.6 K above
  the steady figure. Anchor and seat both placed the patch there; the
  identification saw −0.8 K of MCU die innovation an interval that no
  scale could fit while the NTC read +0.2 - the compromise was air
  1.75, capacity 0.79 for a truth of 2.0, 1.0. With the rate term:
  1.98, 1.05.
* **The shadow must read the element by the observer's rule, bound
  included.** The truth's element hits `never past either patch` 250 s
  into a cooldown from 250 C patches and then tracks the V patch down
  - a 7.6 K drop in one interval; the shadow integrated the lag
  without the bound and predicted a slow fall: −5 K once, then −0.9 K
  every sample charged to the laminate's capacity (0.34). Now
  `thermal_ntc_follow` is one function both call, and the shadow's NTC
  sensitivity is the held patch's while it is held.
* **The identifier's own rules, all measured in:** the seat is
  measurement-consistent (the element set to its reading, each die's
  node to what it implies, the patch under it by the lag algebra) so
  an innovation is the reading's change over the interval against the
  model's and a slow state error cannot enter - subtracting the seat's
  residual was tried first and a fraction of tens of kelvin is still
  kelvin; the seat's own dependence on the spread goes into the
  regressor (`s[SPREAD][patch] = per_r * R_base`) or each seat moves
  the patch and nothing is charged for it; the sample that ends a
  blind gap (> 90 s since a reading, measured from the last READING -
  the shadow's horizon is reset at 600 s and a ten-minute run ends
  exactly there) seats and is not judged, and two more pass while the
  anchors work; an innovation beyond 3 σ of `hᵀPh + R` has R inflated
  to sit at 3 σ; while UNCERTAIN the online variances are floored at
  half the prior, or a handful of state-error samples collapsed the
  covariance around 0.32 for a truth of 0.5 and the filter was certain
  of it.
* **SPREAD and NTC are not observable from a cooldown and are held.**
  With only the air path changed in the truth, the spread ran to a
  clamp - 0.25 or 4.0 - in every arrangement tried, and the NTC's
  share followed: a cooldown puts no power through the legs' edges,
  the dies' own edges are seen only through the seat's relation to
  neighbours no thermometer reads, and a scale the data cannot see
  absorbs what the others leave over. `thermal_ident_online` says which
  move (air, capacity); the other two stay on the wire and in the
  record for a bench to set. What would free the spread: a static
  regressor at idle - the MCU die against the thermistor at rest IS the
  MCU's edge (0.666 W × 22.5 K/W = 15 K per unit).
* **What it does now, from the defaults, three cycles:** box - air
  2.01 → 1.92 → 1.93, capacity 1.03 → 1.07 → 1.06, CONVERGING after
  the first, STABLE after the second, innovation 0.05-0.06 K; fan -
  0.79 → 0.53 → 0.50, capacity 0.65 → 0.97 → 1.02, STABLE after the
  second. Box then fan (the suite's check): 1.93 ± 0.07 after three
  box cycles, UNCERTAIN within the first cooldown after the fan, 0.49
  ± 0.06 four cycles later, CONVERGING. 111 updates over the three
  cycles; the first judged sample of a cooldown is the fourth.
* **On the board:** the identifier steps beside the observer in
  `Board_ThermalPoll` on the same power and slice; a sample that moves
  the scales re-applies them to the observer's network at once. The
  record is the base - `network_from_cal` - and what runs is base times
  scales, so a setter changes the base and never the scale. Saved to
  the record by the board itself: at most every thirty minutes, on a
  disarm if an online scale moved 2 %, never while armed (flash is not
  programmed under a closed loop) and never while UNCERTAIN. The margin
  policy trims every ceiling's span over 25 C by 0.80 / 0.90 / 1.0 for
  UNCERTAIN / CONVERGING / STABLE - a 105 C laminate ceiling is 89 C
  while the model is not trusted. CAL_VERSION 14 appends the four
  scales; a stored 13 is taken up as a prefix with them zero rather
  than refused, because that record holds the DC link span. Firmware
  builds at 0 warnings, 194 364 B flash, 38 348 B DTCM (the shadow
  and its 4 × 20 sensitivities). Not yet run on the board.

## The motion verbs, 2026-09-07

* **`app_robot_arm` had failed two runs in three since 2026-09-04, and it
  was two defects stacked - neither in the notebook.** Measured with the
  notebook's own three poses on the stand-in at three commits: at 8241489
  (the notebook's last pass) the elbow's error after a move stayed under
  0.32 deg and no correction was needed; at 3463277 (the stand-in's rotor
  of 2026-09-03) single corrections of +1.65 and -0.71 appeared; at cbbfaa5
  (the servo's two-reading arrival) the corrections alternated and grew -
  +0.62, -1.03, +0.70, -0.57 - until `tries` ran out, "the shaft stayed
  -0.7 deg short of 20.0 after 4 corrections". A 2 ms trace of the shaft
  found the ring: +-6.6 deg at 28 Hz right after the energise, before any
  slew - a rotor released into the weakest current step under 0.01 N.m
  swings to twice its static deflection, 5.6 deg at 0.33 A - and it had
  not decayed two seconds later. (1) `Servo._measure` took nine reads over
  0.2 s, 22 ms apart: against a 28 Hz ring that aliases to 17 Hz and
  leaves up to a degree of the ring in the "mean"; `to()` corrected that
  degree, the correction re-kicked the spring, and the corrections
  pumped. A shaft seen moving more than the slew's pitch across the first
  0.2 s is now read for a full second, 10 ms apart - some thirty periods,
  under a tenth of a degree of ring left in the mean - `Servo.swing` says
  how far it moved, and the refusal names it. (2) The stand-in's rotor was
  integrated with a sub-step re-sized every poll (dt / n) at a twentieth
  of the spring's period. The symplectic step conserves an energy that
  depends on h, so every re-size moved the rotor between energies: the
  ring after the energise decayed with a 16 s constant where 2j/b is 4 s.
  Fixed at that coarse step it GREW 6 % a second and the elbow ran away
  by thousands of degrees - a held rotor is a pendulum, sin(cmd - theta),
  and +-7 mechanical degrees is +-49 electrical; at a hundred-and-twentieth
  of the period, fixed and the remainder carried to the next call, 4.6 s.
  After both: the elbow within 0.09 deg on every pose, no correction
  beyond the move itself, three runs of three - and a run takes 30 s where
  it took 15, the measurement's price. The nine notebooks that ride the
  rotor re-executed clean; test_sensorless 138/138, test_simulated
  247/247. The bench's report was "app_robot_arm verkar ha fel i koden".

## The views

* **A view that reports the mouse cannot be selected from, and the
  default was to report it.** Asking for SGR reports and clearing
  QUICK_EDIT is exactly what a terminal uses to let a reader left-drag
  across a line and copy it, so no number and no braille cell on any of
  these pages could be marked. **Two wrong fixes first, both shipped:**
  a key that handed the mouse back, then that key ALSO stepping the page
  out of the alternate screen and printing it plainly - each reported
  still broken, because the common case (read the page, copy a figure
  off it) stayed behind a keystroke nobody had reason to press. The
  answer was the DEFAULT: the terminal keeps the mouse, `F` lends it to
  the view for the wheel and the trackball, and left-drag marks text
  everywhere with no key at all. The alternate screen was never the
  problem - a terminal's selection is anchored to the buffer and
  survives the cells under it being rewritten. Zoom moved onto `+ -` in
  the two views that had it only on the wheel; C was not free (the
  attitude view's frame, the menu's direct entry), which is why the key
  is F.
* **The kW foot gauge, 2026-09-05.** Linear over 0-2 kW it showed
  nothing for the loads the bench actually runs - a few tens of watts is
  a sliver of a 2 kW bar; log-scaled it was the other failure, the top
  half of the bar spent between 200 W and 2 kW with the interesting end
  compressed. The bench asked for small draws visible at the start,
  the middle of the scale near 500 W, the last stretch up to 2 kW and
  deep red past it. That is a power law with its exponent fixed by the
  midpoint: `share = (W / 2000) ^ p`, `p = ln 0.5 / ln (500 / 2000)` =
  0.5, a square root. 50 W sits at 16 % of the bar, 125 W at 25 %,
  500 W at 50 %, 1125 W at 75 %, 2 kW at the end; above 2 kW the bar
  is full and takes `SOA_TRIP`'s red, and the foot label wears the
  same ink as the bar. `WATTS_MID` is the only knob - move it and the
  exponent follows. `test_the_power_face_has_its_middle_at_half_a_kilowatt`
  pins the midpoint, the monotone rise, and the red.
* **DRIVE's mode row carries the envelope, 2026-09-05.** It said
  `RUNNING SENSORLESS`, and whether the board was clamping the current
  it ran under sat three rows down as `throttle 63 % of the clamp`. Now
  `HOLD (NORM)` / `SENSORLESS (NORM)`, and `(THR)` in `THROTTLE_RED`
  (xterm 124, a red darker than the trip's 196 and the pulse's 210)
  while the board holds it back - on the board's own `throttling` or
  `tripped`, the same verdict the SOA gauge pulses on, now one
  predicate (`envelope_acting`) for both. The darker red is deliberate:
  the envelope working is not a fault, and the page has already been
  taught once (FLASH_HZ, down from 3) not to shout about routine work.
* **One ruler for every thermometer, 2026-09-05: -35 to 130 C, the
  room at 25.** The gutters ran 125 C from the REPORTED ambient, the
  winding at the foot 150 from a literal 20, and the NTC's colour ramp
  -20 to 100 - three scales on one page, one of them moving with the
  room. `TEMP_FLOOR_C, TEMP_SCALE_C` and `temp_share` are the one
  definition now; the NTC ramp's ends are the scale's. The floor is
  under a winter bench so a cold board starts on the tube, the top is
  past the record's highest ceiling (125) so a node at its ceiling is
  seen short of the top. The stand-in's `AMBIENT` is 25 (it was 20):
  every idle figure recorded above from the stand-in - 49.1 C on the
  MCU, 51.1 on the regulators, 0.30 of the budget spent idle - reads
  about 5 K higher on it now, and the C core's test keeps its own 20.
  The room sits at 36 % of the tube, and MOTOR SOA at rest reads 64 %
  left rather than 100: a margin against the top of a scale that
  starts under the room.
* **CI was red for four pushes on one flag, 2026-09-05.** The demo
  test launched the rotor observer with `python -P`, and `-P` arrived
  in Python 3.11: on the 3.10 runner the workflow declares as its floor
  it is "unknown option", exit 2, and `the view ran two hundred frames
  simulated` failed while 2346 checks passed around it. The 3.12 leg
  was green, so the tree looked right from the bench. Read off the
  commit comment the failure step posts - the public API serves it
  without a login, which is what it is for. The flag is gone; nothing
  in `tools/` shadows a module the view needs, which was the only
  thing it guarded against on the bench.
* **One instrument on every page, 2026-09-05.** MOTOR CONTROLLER drew
  its levels in dots - the tube with the mercury at dot resolution and
  the grey track - and the session, the thermal observer and the
  meter bridge each drew their own: `====----` in ASCII, `█` on `─`
  with `╵` and `│` for the marks. The bench asked for the same
  thermometers everywhere. `coaxial/gauges.py` is the instrument on
  its own, drawn by THE SAME CODE as the machine's gutters
  (`machine._level`, `machine._tube`, extracted so a level is drawn
  once): `gauge` a row, bipolar with a centre and ticks for a meter
  bridge, `tubes` a rank of thermometers with labels. `screen.gauge`
  delegates to it, so the session's THERMAL box and the desk's buffer
  gauge changed without a line in their views; the session's ANALOG
  box gained the bridge's gauge beside each number; the meter bridge's
  bars are the gauge with its centre marked and the burst extremes and
  held peaks as ticks; and the thermal observer has a TUBES box - the
  ten nodes and the NTC beside its map, on the shared scale, coloured
  by each node's margin. The scale and the two colour bands
  (`temp_share`, `margin_class`, `thermometer_class`) moved into
  `gauges` with them; the rotor observer imports them under the names
  its tests use. Marks wear the palette's orange, not white: one ink
  for the thing to be found.
* **The bead has a wake, 2026-09-05.** A bead alone says where the can
  is; a bench watching a sensorless start could not tell from it
  which way or how fast, because a mark moving a cell a frame looks
  the same either way. `_wake` draws TRAIL_S (0.1 s) of travel on the
  rim behind the bead - 36 degrees at 60 rpm, a few dots at a crawl,
  capped at 120 so a fast can does not wear a ring - in three inks
  fading from the bead's orange to the south pole's brown, drawn over
  the rim it rides as the truth stroke is (`Frame.put`'s MARKS). The
  rate is the loop's speed over the pole pairs, the same number
  `travel` integrates, so the wake and the travel figure agree. The
  bead itself is the palette's orange now, the north pole's, not
  white.

### The identification on the pages, 2026-09-05

* **ROTOR OBSERVER's instrument column did not scroll.** The bench:
  "no arrow up/down for more in the right column". `run_view` scrolls
  every page on the console's own state and `frame_of` pages the boxes
  from it - and this page handed `frame_of` the boolean every view
  calls `console` (`board_view.is_terminal`). The comment above that
  line said exactly this had happened once and been fixed; the fix was
  the comment, the call was not changed. `compose` takes the console
  now. The thermal page passed `board_view` all along.
* **The policy on both pages.** THERMAL OBSERVER's SENSE carries
  `model  UNCERTAIN  margin 0.80` as a chip in the margin's colour and
  `scales air 1.00±0.50  cap 1.00±0.20` - the two the samples move,
  with their sigma; read every 5 s, since it moves once a sample.
  ROTOR OBSERVER's foot, at the bench's placement "between WINDING and
  kW": `▴ WINDING 25.0 °C   TH OBS UNCR   POWER  0.02 kW ▴` - TH OBS
  in the leaders' grey and the state's word in the margin's ink, UNCR
  red, CONV yellow, STABLE green, the bench's abbreviations. THERMAL
  OBSERVER with a policy word was 57 to 60 cells against fifty-two;
  the first cut put the word on a second foot row under the title and
  the bench abbreviated it back onto one. `POWER %5.2f` so a negative
  kilowatt does not shift the row (the bench's word) - to ±9.99, and
  100 A of link is 6.3. And `WINDING %5.1f` after the bench asked
  whether three digits push on TH OBS: measured at 100.0 and 123.4 C
  the head grew a cell and took it from its own gap, TH OBS and POWER
  keeping their columns - by the parity of the centring, which the
  fixed width no longer relies on.
* **The stand-in identifies its own ground truth**, 2026-09-05, after
  the bench's "how else would you even simulate that it works": a
  HYPOTHETICAL BOARD - the same graph with a situation laid over it,
  box 2.0 / fan 0.5 / heat sink 0.35 and 1.6 / stuffy 1.5 on the air
  path and capacity - heats on the losses the observer estimates and
  is read through its NTC, MCU die and A1335 die with ±0.05 K every 5
  model seconds; the observer anchors on those readings by
  `thermal.c`'s rules and the same identifier, mirrored in
  `thermal_ident.py`, runs beside it. Measured (`test_simulated`, 52
  model minutes in 6.5 s): six minutes at 30 A in a box then a
  cooldown - UNCERTAIN, CONVERGING at six minutes, STABLE ten minutes
  into the cooldown, air 1.92 ± 0.05 for 2.0, capacity 1.02; the fan
  on under that trusted model - UNCERTAIN at once (innovation 2.1 K),
  CONVERGING at twelve minutes of cooldown, STABLE at sixteen, air
  0.50 ± 0.05. The record is a file (`COAXIAL_SIM_NVM`, the temporary
  directory for the pages): written on the disarm after the run, and a
  new stand-in on it resumes CONVERGING at the saved scales.
  **One number moved in the C by this:** the covariance floor while
  UNCERTAIN was a quarter of the prior for every scale, which put the
  capacity's floor (0.05) exactly on the STABLE threshold (0.10 -
  sigma, so 0.1) and a board could never be STABLE again after a
  switch; and floored alike, the capacity took a third of the fan's
  correction and was left at 0.65. Half the prior for the air path,
  which is what a situation changes, a quarter for the rest
  (`FLOOR_SHARE`): capacity 0.67 to 0.72 after a fan, STABLE reached.
  The margins are the bench's numbers since: 80 / 90 / 100 %.
* **An idling board stays UNCERTAIN**, the bench's rule the same day:
  "there are no hot switches burning energy and moving the board's
  temperature - that is why one keeps to 80 % of the SOA when switching
  starts, or lower, with the thermal situation unknown." Before it, an
  idle board GAINED confidence: at equilibrium the readings agree with
  the shadow whatever the air scale, because the observer's ambient
  estimate absorbs the difference (the board has no ambient sensor), so
  the innovation was zero, the regressor was not (the shadow's
  equilibrium moves with the scale - about 1 K per unit at the idle
  rise) and the covariance narrowed on nothing. Now a sample whose
  seated thermometers all moved less than three floors (0.3 K) since
  the seat is a STILL board: it moves neither the scales nor their
  covariance (`IDENT_STILL_GAIN`). Its prediction error still feeds the
  innovation filter - the first cut skipped the sample whole, and then
  the filter froze on the last big innovation of a fan's cooldown and
  the stand-in never left UNCERTAIN (26 minutes, sigma pinned at the
  floor); a model that has learned a cooldown and then sits quietly on
  the readings is not held UNCERTAIN by the quiet, and one that has
  learned nothing keeps its prior sigma and stays. Measured: forty idle
  minutes in a box at equilibrium, sampled every thirty seconds - zero
  updates, UNCERTAIN, on the C and on the stand-in alike; the cooldowns
  that identify move 1 to 7 K an interval and are untouched. A warm-up
  from a cold start is a transient of its own and does count: the
  housekeeping's 1.5 W is a step whose answer the thermometers see.
  And the trim is where the bench put it after two tries. First on the
  foot; then as red caps on the SOA tubes, the span from the ceiling in
  force to the record's drawn in the trip's red - twelve percent of a
  tube that runs -35 to 130 C, a row or two, and rejected: "instead of
  making the tops red, start flashing SWITCH SOA red at 80 % already,
  and throttle down; then 90; and do not throttle until 100 %." So the
  SWITCH SOA and MOTOR SOA legends (and their gutter tubes) read the
  spend of the WHOLE SOA - the board's `used`, which is against the
  ceiling in force, times the margin - and flash red where the board
  acts: at 80 % UNCERTAIN, 90 CONVERGING, 100 STABLE. Read raw, `used`
  had said 100 % at three different temperatures. The ramp is the
  board's own: the derate begins at THROTTLE_AT of the span in force
  (72 % of the SOA UNCERTAIN, 81 CONVERGING, 90 STABLE) and the ceiling
  is the trip. The motor's legend pulses too since the winding is a node
  the envelope acts on. Also on the foot - "make it
  visible that it throttles at 80 % of the SOA already, then 90, then
  100 as the model's uncertainty goes to zero": `TH OBS UNCR 80%`,
  `TH OBS CONV 90%`, `TH OBS STABLE` alone at the whole span, since
  `STABLE 100%` is 52 cells with the two gauges' names and nothing
  between them. The spend bars and the throttle already ran against the
  trimmed ceiling on the board and the stand-in alike; the foot says so.
* **The room is the fifth identified quantity**, 2026-09-05, after the
  bench's "take the motor and the electronics from 25 C indoors to
  -20 C outdoors and back in" and "a robot from a 20 C warehouse into a
  -25 C freezer and out into 45 C". The board has no ambient sensor.
  THREE ESTIMATORS, TWO OF THEM WRONG. (1) `thermal.c`'s anchor had
  inferred the room as the mean patch less the laminate's losses through
  the bulk path: an identity when the patches are evenly warm and a
  downward drift when they are not - the fourth-root law evaluated per
  patch on one side and at the mean on the other - and it carried
  nothing about the room. Ported to the stand-in with the anchor's new
  78 % pull it walked to -173 C for a room at 25 within an hour and drove
  the air scale to its clamp behind it; on the board it had been pulling
  0.5 % a sample and nobody saw. (2) An integral of the dies'
  common-mode correction, gated on quiet legs: reads a step in the room
  in a couple of minutes but cannot tell a cold room from a good air
  path - both make the board colder - and overshot to -44 C for -20
  while the air scale went to 2.4 for 0.8; the fan case broke too. (3)
  The room beside the scales in the Kalman step (THERMAL_IDENT_AMBIENT,
  MINOR 15 on op 10), its sensitivity by finite difference like theirs:
  the room's is the same at every rise, the air path's grows with it, so
  a cooldown separates them, and an idle board tells neither - the
  still rule already skips it. Its prior is ten kelvin AND IT IS A
  WEIGHT: at three the room moved 0.6 K a gated sample and the air
  scale took the rest (2.9 for 0.8, both mirrors); at ten the room is
  found within 5 K both ways with the air scale near the truth, at the
  price of some of a cooldown's early state error landing in the room -
  19.6 C for a bench at 25 after the first cycle, which the anchored
  nodes do not feel and the next cycles correct. The room's process
  noise is 22 mK a sample, a couple of kelvin an hour at the board's
  cadence; at 0.14 K the room's variance never closed and the air
  scale's sigma floored at 0.14 through their correlation. Measured: C
  ground truth, a run and a cooldown at 25 then forty idle minutes at
  -20 - room -18.8 C, air 0.74 for a truth of 1.0 there, CONVERGING;
  back in, 23.0. Stand-in, at the board's thirty-second cadence (it
  was five, and judged every twenty seconds a late cooldown's samples
  moved under the still rule's 0.3 K, so the room never separated from
  the air path and a box stayed CONVERGING where the board goes
  STABLE): six minutes at 30 A and eight cooling on the bench, then
  outdoors idle - UNCERTAIN, CONVERGING at fourteen minutes, room -27
  for -20 and air 1.3 for 0.8 on idling alone; a run in the cold
  settles both - room -19.2, air 0.73, STABLE; back in, 25.3, air 1.0.
  With the room beside the scales one cycle leaves the air scale known
  to 0.12 and CONVERGING and the second takes it under a tenth: a box
  is 1.89 ± 0.12 after one, 1.98 ± 0.09 and STABLE after two. The
  room is never saved: it is where the board is, not what it is. The
  rooms a situation can lay on the stand-in: bench 25, outdoors -20,
  temperate 20, cold -25, toasty 45 (warehouse, freezer and thai until
  the bench renamed them neutrally, 2026-09-06). And the bench's word on the
  record: a good observer earns STABLE within a few cooldown samples,
  so the flash save is not needed for safety; it stays because a
  resumed record starts at 90 % instead of 80 and costs nothing.
* **"Now it never goes STABLE on the rotor page"** (bench, 2026-09-06),
  after the room joined the step. Measured: with the room beside the
  scales ONE transient leaves the air path known to 0.12 and the room
  to 2.3 K from its ten-kelvin prior - the two share a cooldown's
  evidence - and the STABLE thresholds were a tenth and 2 K; the
  second transient took both under (0.10 and 2.0 at the second
  cooldown's eighth minute). The rotor page holds a steady current, so
  after a situation switch there is one warm-up and then equilibrium,
  which the still rule rightly ignores: one transient, never STABLE.
  Two things were tried and rejected on measurement: the room's
  UNCERTAIN floor at a quarter of its prior helped nothing on a fresh
  start (the prior is what is wide there) and broke the return indoors
  (room 10.9 for 25, air 2.85); the still threshold at 1.5 floors
  admitted the cooldown's tail and the noise in it, and the C fan case
  went from 0.46 ± 0.04 STABLE to 0.43 ± 0.20. What holds: STABLE at
  0.15 on the air path and 3 K on the room - what one transient gives,
  and as much as an envelope acting on anchored nodes can use. A tenth
  stays on the capacity, which converges to 0.03.
* **The margin is continuous and the states are words** (bench,
  2026-09-06: "take out the three discrete steps 80/90/100 and scale it
  with the innovation normalised between 0.8 and 1, adjustable; keep
  the states as pure display; remove the writing and reading to
  disk"). `thermal_ident_margin` is `floor + (1 - floor) * (1 - doubt)`,
  the doubt the worse of two normalised measures: the filtered
  innovation from the thermometers' floor (none) to three floors (all -
  the ratio that says UNCERTAIN), and each online quantity's sigma from
  its STABLE threshold (none) to its prior (all). The innovation alone,
  which is what was asked for, fails by arithmetic before it is run: a
  fresh board's innovation starts AT the floor and idle - which the
  still rule rightly leaves alone - never raises it, so a board would
  have had its whole span at boot, against the bench's own cold-start
  rule. Measured on the stand-in's walk with both beside each other:
  they agree wherever the board has cooled down and part where it has
  not - twenty idle minutes outdoors with the air scale at 1.27 for a
  truth of 0.8 and the innovation back at the floor, innovation alone
  1.000, with the covariance 0.945. Along the box walk: 0.80 fresh,
  0.88 three minutes into the first run (UNCERTAIN still), 0.94 at six,
  1.00 by the first cooldown's fourth minute - two minutes ahead of the
  word STABLE, which needs five runs; the fan on, 0.80 within a minute
  (innovation 0.97 K), held thirteen minutes, 0.87 to 0.99 over the
  cooldown's eighth to fourteenth minute; STABLE dips to 0.97 on an
  innovation blip (0.13 K) and comes back. The floor is the record's
  (`soa_margin_floor_ppm`, CAL_VERSION 15, thermal op 12, MINOR 16) and
  the bench's 80 % by default; refused at zero, which would trip the
  stage at boot. Gone with it, all of it: the board's flash save and
  resume (`Board_CalSetThermalIdent`, the save policy,
  `thermal_ident_resume`), the record's four scales - the floor stands
  in their place and a stored 14 or 13 is taken up as a prefix, two
  back this once because the bench board's 13 holds the DC link's span
  - the stand-in's file (`COAXIAL_SIM_NVM`) and the pages' hooks for
  it. Op 10 writes `saves` 0 and `since_save_s` never, since a wire
  field is never removed, and op 11 no longer refuses while armed -
  there is nothing to write. On the rotor foot the percent rides every
  word now, `STBL 97%` under a percent since `STABLE 97%` is a cell
  wider than the row has between the gauges' names. Firmware 0
  warnings, 194 912 B flash, 38 456 B DTCM. thermal_core 134, simulated
  236, views 173.
* **The THERMAL OBSERVER page runs a load cycle on the stand-in and
  wears the evidence under the board** (bench, 2026-09-06: "make the
  page show the board's temperatures from a simulated load cycle, so
  one sees how the regions warm and cool; a scale under the object like
  the rotor observer's, where one sees the innovation vary over the
  cycle; red to yellow to green"). `SimulatedThermal.load_cycle`: six
  model minutes at 30 A and fourteen idle from the model's own clock -
  the suites' walk, two minutes of wall time at HASTE - laid on in
  simulated mode beside the random situations; `Thermal.load_cycle`
  refuses on a board in words, the drive and `tools/switch.py` being
  what put current through one. UNDER THE ENVELOPE, found on the first
  walk: the cycle's 30 A ignored the clamp and the trip, and driver U
  reached 212 C with the stage nominally tripped - a load the envelope
  cannot clamp would cook the hypothetical board past the ceilings it
  is there to act on. Now the amps are the cycle's times the applied
  derate, worked out whether or not a drive is wired to take it, and a
  trip ends the run until the next on-phase (the re-arm a bench would
  do). Measured live in a box: driver U 89 C after one minute, the
  throttle point inside two, then 95 to 105 C on 12 to 19 A of the 30
  asked for; idle, 62 C by the twelfth minute. THE BAR: `TH OBS` and
  twenty cells of the span the model has EARNED - empty at the floor,
  full at the whole span, one minus the doubt - red below a third,
  yellow to two thirds, green above, and the label in the bar's ink so
  the floor reads red with nothing filled. The innovation in kelvin and
  the margin rode to its right first; the bench: "remove the text to
  the right of the scale, move it to HEADROOM" - so the bar stands
  alone and HEADROOM carries margin, floor and innovation under the
  soak, the envelope's figures in the envelope's box. Rastered at the
  fourth, tenth and twenty-fourth minute: the legs' patches red under
  load and orange in the cooldown, the bar yellow at 91 % and green at
  100 %. And "the page does not seem to run a load sequence": both
  pages' stand-in hooks - the random situations, the cycle - were keyed
  on `--simulated`, and a bench with no cable reaches the stand-in
  without it, through the rig's fallback; keyed on `origin.real` now.
  Then the bench, on the pushed page: "make the colour of TH OBS
  constant, only the thermometer changes colour; loop the load cases a
  bit faster so the temperatures on the board pulse a bit faster -
  otherwise damn good". The label is the leaders' grey since, and at
  the floor the bar is its tip and the track alone. The page's cycle
  is two model minutes at 30 A and four idle (`PAGE_CYCLE_ON_S`,
  `PAGE_CYCLE_OFF_S`), thirty-six seconds of wall time, the stand-in's
  default still the suites' six and fourteen. Measured live in a box
  over thirty minutes, three cycles tried: 120/240 swings driver U 71
  to 109 C, CONVERGING by the fourth minute, margin 0.98 by the
  eighteenth; 90/210 swings 72 to 112, CONVERGING at the fifth; 180/300
  swings 63 to 112, CONVERGING at the fourth, 0.99. None reaches STABLE
  in thirty minutes - the air path's sigma wants the walk's long
  cooldowns to get under 0.15 - and the margin says what has been
  earned either way, which is the point of it being continuous.
* **The meter bridge's peak hold is released toward the bar** (bench,
  2026-09-06: "make the decay meter not lag behind the value in the
  bar; it should be a typical peak hold that decays toward the current
  value - the value simply pushes the peak hold, which then falls back
  toward the current value"). `Desk._hold` fell a FIXED 1.5 % of full
  scale an update whatever the distance - eight seconds for the whole
  bar - so on the stand-in's phases, which swing 0.27 of full scale at
  0.14 Hz, the caret was at most 0.20 of full scale above the bar: 7.6
  of the 38 cells, where the bar had been seconds ago. Now each end is
  pushed out at once by the window's extreme and RELEASED toward the
  bar's level by `RELEASE` of the distance an update, the old 1.5 % the
  least it moves so it lands; it never falls below the window's own
  extreme, which is the tick beside it. Measured on the same sweep at
  eight updates a second, the caret's worst height above the bar: 0.15
  a fifth, 5.1 cells; 0.25, 3.2; 0.35, 2.1; 0.5, 1.1. A quarter ran
  first - a time constant of half a second - and the bench, on the
  pushed page: "make the decay in METER BRIDGE a bit slower"; 0.15
  since, eight tenths of a second, 5.1 cells at the sweep's worst,
  more of a hold. The legend's held lo / hi keep their own
  slow memory (`PEAK_DECAY`), a figure to read rather than a mark to
  watch.
* **THERMAL OBSERVER tours the rooms on earned margin, and SENSE says
  `sim`** (bench, 2026-09-06: "make truth in SENSE another name, since
  it is only in simulated mode that the thermal situation is known";
  "run the predefined temperature cycle, +20 to -25 to +45 and back to
  +20, in a loop, so one sees the innovation vary; make it more
  dynamic - switch the outdoor temperature after it has run with a
  stable innovation for a while"). `SimulatedThermal.situation('tour')`:
  temperate 20 C, cold -25, toasty 45, round again - "it is not
  called thai; call it toasty, cold and temperate, or something more
  neutral", the bench, on the first names - and the
  move is the identification's to earn, not the clock's. First cut: the
  margin held at 0.95 for three model minutes, measured under the
  page's two-on four-off cycle over 150 model minutes - moves at 8, 30,
  54, 72, 89, 113 and 137 minutes, every leg earned; after each the
  innovation swung to 2.1 to 3.0 K within two minutes, the state
  UNCERTAIN for fourteen to sixteen, the room re-found within 5 K in
  ten to twelve, the margin back at 0.95 in eighteen to twenty-four;
  the cold room found faster than the toasty one (17 to 22 against 22
  to 24), whose 45 C pulls the air scale to 2.2 on the way before it
  settles at 1.6 - and never the word STABLE before a move, since 0.95
  is a leg short of it (air sigma 0.157 against the 0.15 threshold,
  room 3.4 K against 3). The bench: "do not switch the outdoor
  temperature until it has run in STABLE for ten seconds or so". So
  the trigger is the state: STABLE held a hundred model seconds - ten
  of wall time at HASTE - no sooner than five minutes after the last
  move, or fifty regardless (`TOUR_*`; forty-five first, and on CI's
  slower 3.12 the toasty leg ran to it a minute short of the suite's
  literal allowance, so the check reads the cap off the constant and
  the cap gives the leg's 38 to 48 minutes room - and then red once more
  on 3.10, the toasty leg at 47 by the loop's count without STABLE: the
  stand-in also advances on the WALL clock inside `identification()`
  and `state()`, the live path, so a walk driven by `fast_forward` drifts
  by however slow the machine is, and the state was read after a wall
  advance had already moved the tour; the suite's walks run the live
  path off since, model time only, and five suites racing here pass -
  and the box and fan walk the same the next day, red on 3.12 with the
  fan's air scale at 1.34 for the same reason, every walk in the
  identification test on model time only since);
  a named situation ends
  the tour. ROTOR OBSERVER tours too since the same evening - "now
  ROTOR OBSERVER never switches to cold, hot, back to temperate", the
  bench, once its demo could reach STABLE under the judge on the
  movement - where a random situation every three to six minutes had
  stood. Measured, the cycles a leg could run: 120/240 reaches STABLE
  at the tenth minute from a fresh warehouse and holds it 51 of 60,
  and again 22 minutes after the cold room; 120/360, 12 and 22; 120/480,
  14 and 19; 180/420, 14 and 19; 240/600, 10 and 17 - the longer
  cooldowns buy little, so the page keeps its pulse. And the tour with
  no cap, 240 model minutes: STABLE 25 into a cold leg, 38 into a
  toasty one (its 45 C pulls the air scale to 1.5 on the way in and
  the room to 47 before both settle), 25 into a temperate leg from the
  toasty room; a thirty-minute cap forced the toasty move short of the
  word, so it is forty-five. The SENSE row is `sim
  cold  4 min`, its second row the laid scales and room. And above
  the board a hint, the bench's: "show 😓🔆, 🥶❄️ and 😌🌤️ on top of the
  board as a little emoji hint, depending on the estimated ambient
  temperature" - on the identification's room, not the truth's, so a
  board wears it too: under 5 C the shiver, from 35 the sweat, mild
  between, in the blank row `picture` leads with, centred over the
  board's field on the bench's next word ("centre the emojis above the
  board") - the map's narrowest row less the rail is the field, and the
  board sits centred in it. Rastered over the
  cold, temperate and toasty rooms at -24.5, 20.0 and 44.0 C
  estimated: each row wore its hint, though `ansi2png`'s font has no
  colour emoji and drew the shiver as boxes. Then the bench: "see if
  you can scale the emojis up, or think of something symbolic in
  braille, so it does not break with the rest - the retrofuturism /
  cyber / Blade Runner". A terminal cannot scale an emoji, and the
  pages draw in braille: so the hint is three PICTOGRAMS, seven cells
  by three rows of dots, in the thermometer ramp's own inks - a
  snowflake in its blue, the sun behind a cloud in its green, the sun
  in its red (`ROOM_DOTS`, `braille_icon`) - two rows more in the
  reserve, and the raster can judge them. Seven cells by three rows
  first; the bench: "tidy up the symbol - hot and cold are a bit ugly
  and asymmetric - and make the symbols a bit smaller too". Five cells
  by three since, the axis through the middle cell's two lanes, every
  dot of the snowflake and the sun mirrored into the other lane of the
  mirror cell, so the terminal's own spacing of the lanes within a cell
  cannot skew them; rastered again over the three rooms. And then the
  bench: "switch back to the emoji for cold, mild, hot - ❄️🥶, 🍃😌,
  🔥🥵 - and 🌡️🤔 when the innovation is large; see if you can scale
  the emoji up beyond the standard size". Back to one row, the four
  pairs, the thermometer thinking while the filtered innovation is
  three floors or more (0.3 K, the ratio the state calls UNCERTAIN),
  and the board has its two rows back. They cannot be scaled: a
  terminal draws an emoji two cells wide at its font size, Windows
  Terminal and VS Code's xterm.js both leave DECDHL double height
  undone, and a sixel image would need an emoji font rasterised on the
  host and a terminal that shows sixels - none of which the chooser's
  terminal has. A space between the two since, the bench's "so it does
  not go wrong in the terminal": back to back, one with a variation
  selector, the pair can be shaped or mis-measured - and one cell to
  the left of that, half the five rounded up, on the bench's eye:
  "shift it one space left so it is centred again". views 180,
  simulated 244.
* **The innovation is judged against the reading's own movement**
  (bench, 2026-09-06: "it never seems to converge during the run cycle
  in ROTOR OBSERVER, and SWITCH and MOTOR SOA seem stone dead - something
  breaks"). Nothing broke: with no board attached the page is on the
  stand-in, and headless five minutes of the demo end with both legends
  at 85.9 %, TH OBS at CONV 97%, the winding at 106.6 C with 12 %
  headroom, no trip; a three-minute replay of the burst and the load
  holds CONVERGING at 0.95 throughout, the derate never below 1.00.
  What was true: it never reached STABLE, because the prediction error
  was judged against a fixed 0.1 K floor while the demo moved the
  readings several kelvin a sample - a two-percent model error on a
  four-kelvin swing is 80 mK, and the filtered error sat at 1.5 to 2
  floors for ever, where STABLE wants under two for five runs. The
  thermal page's cycle gave it quiet cooldowns and got there; the rotor
  page never stops loading. Now the error fed to the judgement is the
  raw error times the floor over the floor plus five percent of the
  reading's movement since the seat (`IDENT_MOVE_SHARE`, the mirror's
  `MOVE_SHARE`): at rest unchanged, a 0.3 K miss on a still board is
  still three floors; under a live load a model that predicts the swing
  to a few percent is predicting. The Kalman step still takes the raw
  error. Measured: the replay of the demo reaches STABLE at 1.00 within
  eighty seconds of wall time (thirteen model minutes), judged
  innovation 0.04 to 0.06; the box walk goes STABLE at the first
  cooldown's sixth minute as before; the tour's legs to STABLE are 10
  fresh, 23 cold, 38 to 48 toasty, 21 temperate - the toasty leg still
  at the forty-five minute cap now and then. The core suite's box, fan,
  room and idle cases pass unchanged: a fan's first cooldown still says
  UNCERTAIN (its 1 to 2 K errors on 5 K moves are four floors judged),
  idle still teaches nothing. Firmware 0 warnings, 195 016 B flash.
  And the envelope not acting is the demo, not the observer: the burst
  was sized to nine tenths of the budget on the ten-node graph and
  reaches 0.57 of the span in force on a cold board at the floor and
  0.82 of the whole span over four minutes, warm and STABLE, no trip;
  1.4 and 1.8 s of it were tried over two minutes and reached 0.68 and
  0.71, the leg node saturating against its patch in seconds. Harder or
  longer would meet the throttle only by
  latching a trip while the model is doubted, which in a demo is a dead
  stage, so the burst stays and its comment says what it measures.
* **The room is reset when the model stops predicting** (bench,
  2026-09-06: "maybe just give it a bit more time, the dynamic model
  takes a while to settle and you have surely put Gaussian noise on the
  NTC"; then "see if you can speed up or reset when the innovation runs
  away from hot to cold or the other way"; and "the important thing is
  that the SOA level goes down when the ambient temperature or heat
  capacity becomes unknown"). Measured first on model time alone - the
  suite's walk had been drifting on the wall clock, FINDINGS above -
  the toasty leg took 41 to 56 minutes to STABLE: a room step arrives
  whole, and the Kalman step shares an innovation out by covariance,
  so with the room narrowed by the last leg and correlated with the
  air path the 70 K step was charged to both, the air scale ran to 1.8
  for a truth of 1.2 and crept back while the room crept up. Now the
  transition to UNCERTAIN, from STABLE or CONVERGING, puts the room's
  variance back to its whole prior and cuts its correlation with every
  scale (`room_reset`, the mirror's `_room_reset`): the next samples
  are charged to the room first and the scales keep what the cooldowns
  taught them. Measured on the tour, model time only, no cap: toasty
  22 to 27 minutes to STABLE, cold 13 to 20, temperate 11 to 32, and
  the margin at the 0.80 floor within a minute of every move - the SOA
  down while the room is unknown, which is the point. The cap stays at
  fifty. Firmware 0 warnings, 195 184 B flash.
* **The mirror is held to the C, number by number** (bench, 2026-09-06:
  "check that the C on the target is a one-to-one mapping of what is in
  Python - without the ground truth, of course, which comes from real
  sensor values"). `test_the_mirror_carries_the_cs_numbers` reads
  `thermal_ident.c`, its header and `thermal.c` and compares every
  `#define` and every `static const` table the identifier and the
  anchor run on with `coaxial/thermal_ident.py`'s attribute of the same
  meaning: twenty-five scalars - the slice, the three finite-difference
  steps, the noise gain, the excitation floor, the innovation filter,
  the settle count, the gate, the still gain, the movement share, the
  two ratios, the stable runs, the two horizons, the blind gap, the
  scale and room clamps, the record's four, the anchor's rate and the
  thermistor's two rules - and seven tables over the five quantities.
  All equal today, and a number tuned on the page and not carried to
  the C now fails a suite instead of drifting. What is NOT one to one,
  and cannot be: the stand-in's ground truth (its thermometers are the
  board's real ones), the integration (the C steps float32 in
  `thermal_step`'s slices, the mirror float64 through `net_flows`), and
  the still rule's readings on the C come off `thermal_sense_t` where
  the mirror reads a dict - the same rules, not the same bits. The
  walks are held separately: the core suite drives the C, the stand-in
  suite the mirror, through the same box, fan, cold and idle cases.
  thermal_core 137, 2910 in all.
* **And on a walk** (the same evening; the bench: "fix what you want,
  the electronics and the motor are not available"): the spread's idle
  regressor was the pick and waits for the board - at rest the MCU die
  sits about three kelvin over the thermistor on the stand-in, and the
  STM32's die sensor is good to a kelvin or two absolute, so without
  that offset measured the regressor would identify the sensor and not
  the edge. Instead `test_the_mirror_walks_with_the_c`: one tape of
  watts and readings - a box, two cycles of a ten-minute run and a
  twenty-minute cooldown, the C truth's three thermometers every thirty
  seconds of the cooldowns with its own noise - played to the C chain
  (`thermal.c` stepped and anchored, `thermal_ident.c`) and to the
  mirror chain (`net_flows`, `anchor`, `Identifier`, as the stand-in
  runs them). Measured, the worst difference over eighty readings: air
  scale 0.003, capacity 0.002, room 0.05 K, judged innovation 0.003 K,
  margin 0.000, the observers' thermistor, MCU and centre within
  0.05 K, and the state the same at every one of the eighty. What
  keeps it from the bit is the integration - float32 in the C's own
  slices, float64 in one-second steps here - and nothing else. Held at
  a hundredth, a fifth of a kelvin on the room and a tenth on the
  nodes. thermal_core 140, 2913 in all.
* **A trip shrinks the envelope, and the spend is measured from the
  room** (bench, 2026-09-06: "now the SOA values died again in ROTOR
  OBSERVER"; "still bugs that send the values negative or divide by
  something small"; "it should trip the limits and push the SOA limit
  down to maybe 70 %, or some other graceful degradation"). Two things.
  THE DEATH was the envelope working: on the tour the margin is at the
  floor after every room change, and the demo's burst, 0.82 of the
  whole span warm, is over a ceiling in force at 0.8 - the stage was
  dropped and latched, the drive stopped making current and the legends
  froze at the housekeeping's level, the same as the morning's "stone
  dead" now that the tour puts the floor under the demo. THE NEGATIVE
  was the stand-in's spend: `_used` measured every node from a fixed
  25 C and did not clamp above one, so in the cold room a node at -20 C
  spent a negative fraction and a tripped node read 103 % - "headroom
  -3 % left". `thermal_budget` in the C measures from `th->ambient`,
  the identified room, and clamps to 0..1; the stand-in does the same
  since, line for line. And THE DEGRADATION: a trip is the ceiling
  reached with the throttle already acting, the model or the load
  wrong by more than the derate could take back, so after one the
  margin is held at 0.70 of every span, recovering a percent a minute -
  half an hour to the identification's own - and every trip starts it
  over (`THERMAL_TRIP_MARGIN`, `THERMAL_TRIP_RECOVER_PER_S`, the
  stand-in's `TRIP_*`; op 10's `margin_micro` carries it, being what
  the board acts on). Re-arming stays the host's: on a board a person;
  on the stand-in the rotor page itself, once no node is at its ceiling,
  saying so on the page - the demo has no other operator. Measured on
  the stand-in: 200 A into the cold room trips once, the margin reads
  0.72 two minutes on, 0.80 at fifteen where the identification's own
  is the lesser, the identification's alone at thirty-five; and the
  demo replayed fresh in the toasty room reaches STABLE at 1.00 inside
  a minute of wall time, the worst node 0.88 of the span in force at
  the bursts, no trip - the room read 55 C on the warm-up under load
  and settled to 46 in three minutes, so a fresh start in a hot room
  spends from a room ten kelvin too warm for a while, which is what
  the covariance term's margin is for. Firmware 0 warnings, 195 412 B
  flash. simulated 246, 2915 in all.
* **Two small ones, the same evening.** The meter bridge's legend held
  its lo and hi on a slow memory of its own, two percent of the
  distance a frame, and said a peak the caret had let go of seconds
  before; it releases at the caret's `desk.RELEASE` since, one memory
  for one idea. And HEADROOM says what holds the margin down: a `doubt`
  row naming the largest of the doubt's terms - `innovation 0.35`,
  `air 0.37`, `room 0.17`, `none` when the span is earned - off
  `thermal_ident.doubt_terms`, the C's `thermal_ident_doubt` taken
  apart, which the mirror's own `doubt` now reads too; at rest the bar
  sat at the floor on the covariance with the innovation quiet, and
  nothing on the page said which. The board's noise floor is not on the
  wire, so the page uses the 0.1 K the board is built with.
* **The emoji broke the frame** (the bench's screenshot, 2026-09-06:
  "some graphics error from those emoji, see if you can fix it"): on
  the hint's row the SENSE box's edge sat a cell off. ❄️ and 🌡️ are
  U+2744 and U+1F321, narrow characters made emoji by U+FE0F, which the
  layout counts as one cell and Windows Terminal draws as two, so that
  row ran a cell long and everything after it on the row moved. The
  other six are single code points with emoji presentation, two cells
  to both. So 🧊 for the cold and 🤒 for the thinking thermometer, and
  the suite holds every glyph in the hint to `east_asian_width` W with
  no selector. And then, the bench: "maybe move the emojis to the SENSE
  block on the right, a bit more uniform" - so the pair sits on SENSE's
  `room` row after the figure, and the row above the board is the
  blank it always was. And "put in some hysteresis so the emoji do not
  flutter near the limits": a held word stands until the room is two
  kelvin past the threshold it crossed, and the thinking thermometer,
  on at 0.3 K of innovation, stands until 0.2 (`ROOM_HYSTERESIS_K`,
  `ROOM_SURE_K`); the page keeps the word shown last and hands it back
  in. views 181, 2917 in all.
* **The thermal observer is a tool for the local model** (2026-09-06,
  my own list: the rule sends "how hot is the board, what does the
  envelope keep in hand" to `board_chat`, and neither the MCP server
  nor the local model's tools could reach device 8). `thermal` in
  `coaxial_mcp.tools`, the fifteenth: op=state the measured NTC and
  every node's estimate by group with the identified room, op=budget
  the worst node against the ceiling in force, the clamp and the joules
  left, op=ident the state as a word, the margin the envelope acts on,
  the scales, the room and the innovation - rendered with the one
  measurement named, and on the stand-in the truth beside it. In the
  `read` and `code` sets, not `pins`. Measured on the stand-in: 127, 71
  and 81 tokens a call; the description held to the suite's 140
  characters. mcp 50, ollama_tools 219, 2922 in all.
* **MAP, under SENSE** (bench, 2026-09-06: "an explanation for U, V, W,
  REG, MCU, HS and AFE, in SENSE maybe, you decide"): a box of its own
  between SENSE and HEADROOM, one row a mark - the label, the
  references its frame is drawn round off the pick and place
  (`thermalmap.MARKS`), and what they are in words - so the letters on
  the picture are read off the same column as the numbers. views 182.
* **The stand-in's clock is the wall's or the caller's, never both**
  (2026-09-06, after CI's second red on the same cause). `state()`,
  `budget()` and `identification()` advanced the model by the wall
  clock inside the reader, which is what a page wants and what a walk
  driven by `fast_forward` did not: on a slow machine the walk got
  wall time it never asked for between its steps, and the suite had
  grown seven `_advance = lambda: None` hacks to hold it still. From
  the first `fast_forward` a caller makes the stand-in is DRIVEN and
  its readers stop advancing on the wall clock; the pages, which never
  call it, keep the live path. The seven hacks are gone, the walks read
  the same on every machine, and the tour lands at 12, 40 and 75
  minutes here on model time alone. simulated 247.
* **`thermal_identification.ipynb`**, the same evening: the stand-in's
  tour walked 150 model minutes under the page's cycle and plotted -
  the margin earned and lost a leg at a time against the state as a
  word, the room tracking every step within its band, the air scale
  kicked by the reset and settling, driver U pulsing with the load -
  then a trip in the cold room and its cap climbing back. Executed
  and checked in like the others, from `make_notebooks.py`. Measured:
  the room within 3.3 K at every leg's end, the moves at 12, 40, 70,
  95, 117 and 143 minutes, the trip's margin 0.73 the minute after,
  0.90 at twenty, 1.00 at forty. Executing it twice moved the tour at
  different minutes the first time: the stand-in's derate recovery
  slewed on wall seconds times HASTE, model time on the live path and
  noise under `fast_forward`; it slews on model time since, and two
  executions agree to the minute. structure 614, 2926 in all.
* **The stand-in starts in its room** (2026-09-06, off the toasty
  replay's room at 55 C for 45 on the warm-up). Both boards started at
  25 C whatever the situation's room, so a fresh start in the toasty
  room was a cold board carried in, and the identification, starting
  its room at 25 as well, charged the warm-up under load to a room
  hotter than the truth for three minutes. `Board_ThermalInit` starts
  the observer and the room on the thermistor's reading, and the
  stand-in does the same now: switched on in a room it reads that room
  on every node, and a situation laid on later is a carry-in that moves
  nothing. The rotor page's THERMAL box says the room since, identified
  and, on the stand-in, the truth's beside it, so the tour reads off
  that page too. simulated 247, 2916 in all.
* **The W frame drew wider than U's** (bench, 2026-09-06: "the W area is
  a bit larger than the U area, looks a bit odd, but maybe you have
  other information from the pick and place?"). The file says the
  opposite: U's frame is 15.6 mm wide and W's 14.5, since RU2 and Q2U
  sit 0.4 and 0.8 mm further out than their W twins, RW1 and Q1W - the
  legs are not exact mirrors on the board. The drawing was the odd
  one: `_cell_rect` floored both edges of a frame to cells, which
  rounds a left edge outward and a right edge inward whenever its dot
  falls in a cell's right lane, so a box on the right of the board
  could lose a cell its twin on the left kept, and at some widths did.
  Now each side is the nearest dot, rounded half away from the board's
  centre, the cell that dot is in and the lane it is in, top and bottom
  mirrored about the centre row the same way: a box at +x is drawn as
  its twin at -x would be, and neither is a dot wider than its
  millimetres. Measured over 30 to 88 cells: U 10 to 29 dots wide, W 10
  to 27, never W the wider.
  AND SENSE ONE FACT A ROW - the bench: "the boxes on the right are
  messy, lots of text run together": `sample every 30 s - last 0 s ago`
  and `truth heatsink  air 0.35  cap 1.60  room 25 C  0 min` were
  single rows of up to fifty-two cells into a forty-two-cell panel,
  cropped at its edge; now NTC, err, open, sample, last, model, air,
  cap, room, truth (two rows) and load, each a row. views 178,
  simulated 240.

* **BOARD ATTITUDE's board sits one row lower** (2026-09-07, "den är
  lite högt uppe"): `orientation.LIFT` 0.39 to 0.42 - a row is 0.028 of
  the view's 36, and 0.36 was the fit, 0.39 the row asked for on
  2026-08-30. Rasterised before and after at 110 by 36: the board's top
  moved from the Z label's row to one below it, and its feet still clear
  the frame's bottom. `test_render`'s shipped-board check reads the lift
  off the constant instead of carrying its own 0.39.

## The renderers

* **A per-cell grain is what made the board blocky.** The tone ladder
  picked among the 28 patterns that carry six dots by a per-cell hash,
  uniformly and then cubed toward the even end, and either way a flat
  face wore a different pattern in every cell: 107 distinct glyphs on
  the board's top at one pose against 79 with it off, and the 79 are
  real edges. A flat surface is a flat pattern; the block is spent where
  the LEVEL changes. And the mono ladder sat two rungs heavy - class 2
  at six dots read as a slab where the exporter's `:` is two; four now,
  2.8 dots a cell measured against 3.6.
* **Still blocky with the grain off, 2026-09-05: one rung per cell was
  the block.** At the attitude page's own size and pose (100x34, the
  board near flat) the face was a carpet of `⢕` - the desk lamp's
  gradient is a fraction of a rung a cell, and rounded once per cell
  the whole face rounded to the same rung, with the parts' outlines
  drawn on a tile. Two things, measured: the ladder's window was
  FITTED ON ONE FRAME (368 cells, zoom 1, one pose) and at the page's
  size (78x30, zoom 0.88) at the attitude the stand-in reports - the
  board lying on a bench, rpy -5.6 +2.8 -0.6 - it put 299 of the
  face's 561 lit cells on rung 1; and at other poses the face fell
  under the floor and half of it sat at exactly `DIMMEST` (p50 0.40)
  - one tone, one rung, a slab. (Poses here are quaternions in the
  page's own `(i, j, k, real)` order; a first survey passed `(real,
  i, j, k)` and named its poses wrong.)
  Three changes. The light is SAMPLED PER DOT: each of a cell's eight
  dots is lit where the heat at the dot's own position - the cell's
  heat plus its gradient from the lit neighbours - clears that dot's
  rung, the rungs being the order the ladder adds dots in
  (`LADDER`, read off `raster.SHADE` so the flat glyphs and the
  sampled ones are one alphabet). A flat cell draws exactly the rung's
  glyph as before; where the light changes inside a cell the glyphs
  between the ladder's eight appear, in order - which is what "use the
  whole braille block where it fits" means, and the bench asked for.
  The window is EXPOSED PER FRAME on the frame's 5th-95th percentiles,
  never narrower than the fitted window's share of the ladder so a
  flat frame does not stretch its grain, followed at a third a frame.
  And the floor is a roll-off (`KNEE`), not a clamp, so a face the
  lamp barely reaches keeps its order and the dots can grade it. At
  the page's pose the face's 561 lit cells went from
  299/67/63/44/33/17/26/12 on rungs 1-8 to 158/83/73/76/73/36/29/33 -
  the top three rungs from 9.8 % of the face to 17.5 %, and 17 distinct
  glyphs on the face against the ladder's 8; the board turned 45
  degrees about Y, the top three from 14.5 % to 28.9 %. WHERE THE
  GLYPHS BETWEEN APPEAR, measured on synthetic fields: a linear
  gradient under about 1.3 rungs a cell reproduces the ladder exactly
  whatever its direction - the eight dots' thresholds are a rung apart
  and a gentle slope never straddles one inside a cell - so they come
  only where the light changes steeply inside a cell, an edge or a
  part's relief, which is where they belong; the histogram's gain is
  the exposure's and the knee's. The mono path is untouched: no light
  to sample.
* **Seen in a raster, the sampled ladder was the artefact, 2026-09-05.**
  The glyph counts above said the ladder was spent; a PNG of the frame
  (Consolas for text, Segoe UI Symbol for the braille Consolas lacks -
  the fallback the terminal uses) showed what the bench saw: a sparse
  lattice, eight dot positions in one fixed order so every cell at a
  rung put its dots in the same places, rows of dots a cell apart on
  the dark half, brightness encoded twice - few dots AND a dim tone -
  so that half dissolved into speckle. Three changes, each judged in
  the raster and not in a count. THE FACE IS A HALFTONE: an 8x8 Bayer
  matrix over the DOTS, four cells wide and two tall - square on
  screen - sixty-four densities, fixed in screen space so a turning
  board moves the density and not the dots, on the light sampled per
  dot as before. A DENSITY FLOOR of 0.3: with the tone carrying the
  light the dots need only carry the shape, and three dots in ten
  keeps the dark side a surface (0 was scattered dots, 0.5 flattened
  the shading, in the raster). And THE RIM, CLIPPED AND LIT: the fold
  now keeps which of a cell's four quadrants the 2x2 raster reached
  (`engine.fold`'s `quads`); a dot in a quadrant the model missed is a
  dot outside the board and stays dark - the spill the bench asked
  cut - and a part-covered cell, or one with an uncovered neighbour,
  is drawn solid in its reached quadrants at the outline's tone: a
  hard line of light round every silhouette and every hole, one cell
  thick, the Nostromo look the bench named. The coverage thinning is
  gone with nothing left to do. Measured at the page's pose: the face
  0.59 lit with no cell under two dots, thirteen distinct glyphs (a
  Bayer tile over 2x4 cells repeats its cell patterns - that is what an
  even lattice is), no dot past the rim over its cells, the rim's
  median luma above the face's; 17 ms a frame against 12.
* **"Blocky as hell" - the Bayer's own hierarchy, at a real window,
  2026-09-05.** The 78x30 test frame hid it; a raster at 150x44, the
  attitude page's size on the bench, showed the 8x8 matrix's two-by-two
  clusters as small square blocks across the whole face, and the raised
  parts saturated to solid `⣿` rectangles. Four threshold fields
  rastered side by side at that size: Bayer (the blocks), interleaved
  gradient noise (a regular diagonal screen), the R2 sequence (half
  structured), and a 64x64 void-and-cluster BLUE-NOISE mask (no
  structure at any density, and fixed in screen space so it does not
  crawl as error diffusion would when the board turns). Blue noise
  ships as `coaxial/bluenoise64.bin`, 4096 ranks, with
  `tools/bluenoise.py` as its reproducible, seeded generator - numpy
  and a second for the tool, a file read for the renderer. With it a
  density CEILING of 0.85: the brightest face keeps a dot or two dark
  a cell and stays a texture, the rim and the outline still draw solid.
  And the mesh is earned BY SIZE ON SCREEN as well as by zoom
  (`_earned`): at zoom 0.88 a 150-column window drew from the
  12-division decimate whenever no crew was about - the menu's front
  page, a short run - and the parts came out as chunky boxes; now a
  board 62 cells across earns the 32 grid on its own. Measured single-
  process: 67 ms a frame at 150x44, 20 ms at 78x30 (the 16 grid its 42
  cells earn, 12 before).
* **"Extremely blocky", with a screenshot - the DENSITY was the block,
  2026-09-05.** The bench's picture showed what no raster here had: in
  the terminal the braille glyph box is narrower than the character
  cell (the fallback font's metrics, since Consolas has no braille),
  so from about three dots in ten upward every cell is a brick with
  dark mortar round it, and the cell grid itself is what reads as
  blocks - whatever pattern the dots are in. Rastered at 120x40 at
  zoom 1.5, the bench's framing: 0.30 to 0.85 a brick wall, 0.15 to
  0.50 a stippled surface, 0.08 to 0.35 a light dusting with the shape
  carried by the rim and the outlines. The window is 0.12 to 0.45 now:
  the tone carries the light, a few dots carry the shape. And the rim
  pass counted a whole cell beside an uncovered one as an edge, which
  where the parts crowd the board's far edge made every cell an edge
  and the region a solid bright blob - blocks by another route, also
  in the screenshot; it draws part-covered cells only now, the
  silhouette at quadrant resolution, thin enough to be a line. And
  "the highlight and the edges in another braille character": a rim
  cell was solid in its reached quadrants, and a solid cell beside a
  stipple is a block whatever it means. It is a braille LINE now -
  `EDGE_GLYPH`, one of `⠃ ⠘ ⠒ ⡄ ⡇ ⡜ ⡔ ⢠ ⢣ ⢸ ⢢ ⠤ ⠣ ⠜` by which
  quadrants the model reaches: of the reached quadrants, the dots that
  border a missed one - the whole column across from it, the one row
  above or below it. Along a rim the cells join into a drawn line that
  follows the silhouette, in the bright edge tone, and the face inside
  it stays a stipple in the light's tone: two vocabularies on one
  board.
* **"A shade pixelly" - the face is scanlines now, 2026-09-05.** A
  stipple of scattered single points on black is grain however evenly
  the mask spreads it, and no density inside the window smooths it
  without the bricks coming back. Rastered at the bench's framing: the
  stipple as shipped, the same with a cap of four dots a cell, and the
  same dots confined to alternate dot rows (`SCAN_ROWS`, rows 0 and 2,
  the density doubled along them so a cell carries the same ink). The
  lines were the smooth one: fine broken horizontal lines that close up
  in the highlights, the coherent structure a stipple has none of and
  the retro terminal's own - and with two rows of four always dark a
  cell cannot fill, so the bricks stay gone by construction. The face's
  alphabet is the sixteen patterns two rows make, plus the rim's lines.
* **"Some smart trick" - the phosphor, built twice and taken out,
  2026-09-05.** The one channel a braille cell has that nothing had
  used: its BACKGROUND. Every lit cell's background painted with a dim
  copy of the face's own tone, so there is no terminal black between
  the dots, and a dot is a highlight on a surface rather than a point
  in the dark. Flat, it showed the tone field's own cell steps - a
  raised part was a hard bright rectangle of background, "a bit too
  blocky with those shapes as background". Blurred two cells each way
  by two sliding passes, and weighted by the cell's coverage from the
  fold so the disc's silhouette anti-aliased in colour instead of
  stepping by whole cells, it was smooth - and "a light haze over the
  board": the surface took the crispness the dots had. The bench's
  verdict, kept as the rule: the crispest picture is the braille alone,
  rendered with the edges drawn. Both variants are gone, the escape
  emitter's background pair with them, and the watch on the bench was
  138 ms a frame at 150x44 for the blurred one against 67 without.
  Rasters of both are the reason not to build a third.
* **The bead runs in the can's wall, 2026-09-05.** It rode on the
  outer rim, half in and half out; the bench asked for it between the
  two outer circles, and the air between the can's edges is a race for
  it to run in - `POINTER_SEAT` is the wall's middle as a fraction of
  the can's radius now, and the wake runs the same race behind it, on
  the side it came from, so the rings stay whole and the smear is
  inside the rotor rather than across it.
* **"Still a bit pixelly" - the breaks in the lines, 2026-09-05.** With
  the face scanlines, what was left to be a pixel was every gap in a
  line. The tone carries the light, so the lines can run nearly whole:
  the density window is 0.42 to 0.5 - 84 % of a lit row at the floor,
  a whole row at the ceiling - and only the darkest of the board keeps
  a gap here and there. Rastered at 0.35, 0.42 and 0.5 at the bench's
  framing: whole lines were the smoothest; 0.42 keeps a trace of the
  light in the dots on top of the tone's.
* **The shaft angle's face was the second oval, 2026-09-05.** The same
  geometry as the can, the same fault: drawn at an assumed 2.0 cell on
  a terminal whose cell is taller, and flattened across. The probe the
  rotor observer grew for its can is `screen.aspect_of` now, one
  definition for both views; the angle page asks it once, draws the
  face round, and says in its box whether the cell was measured, given
  or assumed. And a notch smaller - 58 by 21 from 64 by 23 - on the
  bench's word, once it was round.
* **The motor a fifth bigger, 2026-09-05.** "A shade bigger, and scale
  everything after it": the rotor observer's box is 52 columns from
  46. The width is what sizes the can - the gutters take their columns
  first and the machine gets the rest - so six columns are six dots of
  radius; the rows follow through `fit_rows`, the legend's runs and the
  foot's rules through `machine.gutters`, and the instrument column
  keeps its forty. Nothing else on the page is placed by a number of
  its own, which is what made it one line.
* **The instrument column scrolls on every page, 2026-09-05.** It was
  the rotor observer's alone: seven boxes did not fit its column, so it
  grew a window, an arrow row to click, a drag and the up and down
  keys, all in the view. The bench asked for the arrows on every
  submenu. The paging is the template's now - `stage.paged` windows
  the boxes to the terminal's height and packs the last page from the
  end, `frame_of` calls it for whatever view draws through it and adds
  the SCROLL chip to the key bar only while there is somewhere to go,
  and `run_view` takes the arrows, the clicks on the markers and a drag
  over the column for every view at once, with the state kept on the
  console the view draws through. The session keeps its arrows for the
  duty (`scroll_keys=False`) and has no column anyway. The rotor
  observer lost 125 lines and nothing it had.
* **The wake, a shade narrower and shorter, 2026-09-05.** Shorter is
  the shutter and the cap - 0.07 s and 90 degrees from 0.1 and 120 -
  and narrower is the dots a dot apart along the arc rather than half
  a dot: a lighter line, the same fade.
* **The thermal observer is a graph of twenty nodes that follows the
  copper, 2026-09-05.** The bench asked for the modelling taken to ten
  of ten given the target. Judged against the lumped-network class the
  papers in docs/papers put at about ten percent, the star was a six:
  one board node for a disc with a seventeen kelvin gradient across it
  in the camera's switching state, six leg nodes that could not warm
  each other except through that average, the switching loss a point
  scaled with voltage alone, the junction a constant, still room air
  for a board behind a stator. Each is a coarser graph than the
  physics, not a wrong equation, so each is closed by adding to the
  graph: SEVEN LAMINATE PATCHES whose areas come off a quarter-mm
  raster of the outline over the thermal picture's own partition (7776
  mm² in all) and whose in-plane conductances are one sheet figure -
  0.020 W/K per L/d, chosen so the V patch's three neighbours in
  parallel are the 15.2 K/W the camera measured lumped, and what 2 oz
  on two layers at 40 % coverage computes to - times shared boundary
  over centre distance off the same raster; the legs warm each other
  through them; THE HOT SWAP A NODE (35 W at 100 A had been booked on
  the regulators, and its FETs are Q3 and Q4, the bridge's part, 3.6
  mΩ in series - the plausible 5 mΩ was for "a hot-swap FET" before the
  pick and place said which); THE MOTOR THE BOARD'S BOUNDARY - winding,
  stator, rotor, the mount and the faces open on a bench, forced
  convection per sqrt(krpm) off the drive's own speed; LOSSES AS
  FUNCTIONS - the no-load switching scaled by the C_oss energy's own
  law (near V^1.55: 4.3x at 63 V where a line gave 2.6x), the overlap
  from Q_gd 18 nC against the driver's 3.4 A on and the resistor's 2 A
  off, the body diode across both dead times off the record's
  `deadtime_ns`, the gate charge 81 nC from a buck at 85 %; JUNCTIONS as
  R_th times the part's watts, so the MCU's campaign 27 K is 0.666 W
  through 40.5 K/W and rises with load. The integrator sub-steps to a
  quarter second whatever the caller's gap: a leg's silicon is 1.4 s
  now and an explicit 2 s step oscillated. The wire follows the count
  (twenty, MINOR 13; ops 7, 8, 9 carry the table), CAL_VERSION 13 holds
  the network with zero meaning the derived default, so an
  identification has somewhere to keep what it learns, and the record's
  winding fields feed its node as they fed the element. The host mirror
  (`coaxial.thermal`) carries the same tables, relaxes the graph to a
  steady state in 2 ms, and the stand-in integrates it; the thermal
  picture reads its laminate off the patch under each point. Held on
  the core through the host gcc, thirty checks: the patches sum to the
  measured bulk, a leg warms its neighbour and the face loses what the
  leg makes, the C_oss law and the four switching terms, the junction,
  the mount open and closed, the rotor's air at speed, a 2 s step
  landing where a 0.1 s one does. EVERY NEW NUMBER IS AN ESTIMATE WITH
  A DERIVATION and no measurement, said in `thermal_defaults` beside
  each; nine remains the papers' own numerical methods, which do not
  belong in a 10 Hz loop on the target. Firmware 183.8 kB, 0 warnings,
  not flashed. Two test assumptions went with the star: the leg's 28
  K/W no longer exceeds the datasheet's 25.9 whole path on its own (the
  graph's whole leg path, 35, still does, by a third), and lifting every
  board ceiling now lifts the winding's too since it is a node.
* **The stage throttles on the motor's SOA as well as the switches',
  2026-09-05.** THE MOTOR HAD NO ENVELOPE: ten nodes, every one on
  the board, and the winding was the rotor observer's own estimate
  that nothing acted on. The bench asked for the stage to back off on
  how close BOTH are. The winding is one more element in `thermal.c` -
  `3 i_rms^2 R` off the phases' mean squares through the record's
  `motor_r_uohm`, shedding to the air it turns in rather than the
  laminate, stepped on the same slice by `board_thermal.c` - judged by
  `derate_of`, the one ramp both envelopes use; its K/W, J/K and
  ceiling are CAL_VERSION 12's three fields, the motor profile's
  placeholder pair and 120 C until a thermocouple writes real ones.
  ONE CLAMP, TWO ENVELOPES: the stage gets the smaller factor and
  either ceiling trips it through the same path. Op 4 appends the
  winding's estimate, spend and OWN factor (MINOR 12), so the page can
  say which envelope holds the stage back; op 6 sets the three. The
  stand-in mirrors it and the page draws the board's winding when the
  budget carries one, its own estimate marked `est` when it does not.
  Held on the core through the host gcc: 15 W settles the winding
  33 K up; at the same spend a node and the winding get the same
  factor to within the wire's byte - the node's spend crosses as
  `u8`, the winding's was compared at what the byte said, 0.945 not
  0.947, after the first draft failed on exactly that; a cold winding
  whose hold has fallen into the window is throttled on the hold; a
  zero ceiling disables it. On the stand-in, 60 A rms with the
  board's ceilings lifted out of the way: the winding throttles first,
  the stage gets its factor, and the ceiling drops the stage through
  the gate the nodes use. Built, 0 warnings; not flashed - no motor is
  on the bench to check a winding against.
* **The thermal observer's board is a braille halftone with its parts
  on it, 2026-09-05.** It was half blocks - one palette stop a cell, the
  nearest, two field rows a character - and the bench's word was
  "pixelly": the rim stepped at the cell and the field banded at the
  stops. Now each of a cell's eight dots is lit where the temperature
  under it clears the blue-noise mask the attitude face uses
  (`raster.NOISE`, moved there to be shared), denser as it is hotter,
  in `ansi.thermal_rgb` - the same stops, blended to 24 bits between
  them. The rim and the bore's edge are one dot wide. THE PARTS ARE ON
  IT, from the pick and place: `thermalmap.PLACED` copies the file's
  coordinates and `PNP_CENTRE` (106.25, 74.2) is the board's centre in
  the exporter's frame - the placements' extents' midpoint, and the U
  and W switch pairs sit 28.7 mm either side of it; `test_sensorless`
  holds both to the file to a hundredth. The heat blobs moved with
  them: the tape measure had the switches 12 mm too high and the hot
  swap 15 mm too far out. Marked as blocks with edges: MCU, REG (the
  two bucks and the two LDOs), U V W, AFE, HS and NTC - every dot
  inside a package lit in the field's colour, a two-dot white edge
  round it, a label beside or inside. Asked in four steps on the
  bench, each after seeing the last: braille and anti-aliased; then
  "clearer blocks, or with edges" (the dot-wide outline alone read as
  a faint frame); then the scale in braille too - two cells of the
  same halftone at each temperature, so the rail is the legend for the
  dots as well as the colour; then "not black at -20, a shade of blue"
  - the cold stops went from xterm 17 and 19 to 19 and 20. The
  halftone's floor stayed at four dots in ten: five was tried first
  and rastered, and it put an idle board at seven in ten, inside the
  brick range the attitude page had measured. The same raster showed
  the two-dot edge turning a cell column white each side, so a package
  smaller than the MCU was all frame - one dot now, like the rim. REG
  and NTC were the last two asks.
  The page asks the terminal for its cell aspect like the other round
  pages (`--cell-aspect`, the SENSE box says measured or assumed).
  Measured: 88 cells draws in 44 ms once the geometry mask is cached
  (73 the first time), against 40 for the half blocks.
* **The thermal map's regions are frames, and the rim is a dot wide,
  2026-09-05.** The blocks with white edges were "grey areas" on the
  bench, and the rim a thick band: a cell is one colour, so a cell an
  edge crossed lit its field dots white too, and every line was a cell
  wide. Now a cell with a mark in it draws the mark's dots ALONE - the
  rim, the bore's edge, a frame - so a line is one dot with a dot of
  dark beside it, "ideally one pixel". And the marks are frames round
  the GROUPS rather than boxes round the packages: `thermalmap.frame`
  is the bounding box of the members' bodies a millimetre out, the
  field's halftone untouched inside, the label on a side of it or
  inside where there is room. Each phase's frame takes its shunts, on
  the bench's word - RU1/RU2, RV1/RV2, RW1/RW2, the WSHM2818 7 mOhm
  pairs from the pick and place, up at the rim by the terminals, 35 W
  between a pair at 100 A - so the U and W frames run into the rim,
  which is where those parts are; the phase blobs took a second point
  on the shunts, since the node's watts are the FET's and the shunt's.
  Rastered before landing (`tools/ansi2png.py`): eight frames, a
  dotted rim, no solid cell in the mark ink. Held: no marked cell is
  solid and the marks are under a fifth of the lit cells; each phase's
  two shunts sit inside that phase's frame, U left of V left of W.
  AND THEN RIGHT ANGLES, the bench's next word with its own glyphs:
  "braille with just a border and right angles ⠒⢲, ⡖⠒". A frame
  sampled from its millimetres put each line on whatever dot row the
  edge fell and left the corners ragged. `_draw_frame` snaps the box
  to the cell grid and draws it through the cells' own dots - the top
  across dot row 1, the bottom across row 2, the sides down a lane -
  so a frame is `⡖⠒⠒⢲` over `⠧⠤⠤⠼` with `⡇` and `⢸` between, box
  drawing in braille; never under two cells each way, and it still
  stops at the rim. Held: five of each corner glyph and ten of each
  run on the demo board.
  THEN FOUR MORE, in one message: the labels INTO the bottom line
  (`⠧⠤⠤MCU⠤⠤⠼`, `label_at('bottom')`); the hot-swap frame round
  the controller WITH its back-to-back FETs and the fuse - Q3 and Q4
  at 14 and 15 mm just right of the bore, U12 at 23, the fuse RTS1 and
  the varistor V1 at 31, the terminals at 40: the chain runs from the
  bore's edge to the rim, which is why the bench thought the area sat
  further right than a box round U12 alone; REG and the MCU sharing
  one edge - `_share_edges` draws the second frame's side on the
  first's column and lane where they land within a cell of each
  other, either side, since the MCU's frame now stands 3 mm off its
  package (`MARKS` carries a margin per frame) so the temperature
  inside it shows. The first draft caught only a frame landing right
  of the other's edge, and the bigger MCU landed a cell inside REG's:
  two lines a cell apart, seen in the render before the raster.
* **The thermal observer's BUDGET is HEADROOM, and its spend is a
  solid bar with an orange tip, 2026-09-05.** The bench's word: rename
  the box, label the level `soak`, lose the square brackets, "something
  nicer, three braille rows tall". Built as a block three rows tall -
  `⣶` over `⣿` over `⠿` with a dot of air round it, the throttle point
  a mark through all three - rendered through the stage's theme,
  rastered, landed; and the answer was "no, one row of ⣿, not triple
  rows, terminated with an orange ⢸ or ⡇". `gauges.bar` is that: one
  row, every dot of every cell to the level in the margin's colour, a
  tip column in the mark's amber in whichever lane the level ends -
  its cell holding the tip alone, so it is a line and not a coloured
  cell of level - and the track beyond. No throttle mark: the margin's
  colour says where the throttle point is, as the shared gauge does.
  Then "the grey rows four tall too": the track was the gauge's
  three-dot `⠇` a cell and is a full-height `⡇` a cell in the track's
  grey, so the scale stands as tall as the level that fills it. The
  old one-line `budget_line` and the `summary` line went - both dead
  since the boxes replaced them.
* **The clock sync guarded its first NTP query and not its second,
  2026-09-05.** CI on Python 3.10 crashed the whole DAQ suite with
  `TimeoutError: timed out` out of `ntp_offset` - a runner that reached
  time.google.com at the start of a 0.2 s sync and not at the end.
  `Clock.sync` fell back to the PC clock, said, when the FIRST query
  failed, and raised on the second; a rate against UTC needs both ends,
  so now the second failing is the same fall-back with its own note.
  The 3.12 job passed the same commit: the network, not the code
  under test, and the suite that crashed was the stand-in's, which
  reaches for NTP because `set_time_from_pc` does - honest about this
  machine's clock, and now honest about a server that answers once.
  Held on the stand-in with NTP stubbed to answer once and time out.
* **BOARD ATTITUDE holds its face at rest and caps itself at 30 Hz,
  2026-09-05.** The bench's word: "the laptop's fans run away". Profiled
  at 108x40, zoom 1.27: a frame cost 104 ms in one process - 75 of
  them `engine.raster` - and 52 with the eight-worker crew, the parent
  waiting 24 ms on the bands and then spending 28 of its own on the
  glow, the dots, the outline and the rows. At the 20 Hz default a
  52 ms frame never sleeps, so the view held a core and half of eight
  others for as long as it was open, and the board was NOT MOVING: the
  deadband had already made its pose bit-identical frame to frame, and
  every one of those frames rastered and shaded it again. Every pass in
  `_paint` writes cells and reads none, and the ground under it is the
  one thing that moves at rest, so `_face_layer` paints the face once
  onto a blank grid, holds it in `persist` under everything it depends
  on - size, zoom, colour, the pose to six places, the solid - and lays
  it over each frame's ground; a new pose is drawn in full eight times
  first (`FACE_SETTLE`) so the exposure has glided. Measured: a resting
  frame is 3.3 ms with the crew and 4.2 without at 108x40, 4.4 and 5.8
  at 150x44, from 50 and 103 - and the rest of the sleep is the loop's.
  Moving, 50 ms with eight workers and 59 with four, so the crew stays
  eight; `_dots` lost a third of its pass to a precomputed SCAN_DOTS
  and the mask's rows hoisted per row. `HZ_CAP` clamps `--hz` at 30
  (`period_of`); the default stays 20. Held on the render suite: the
  drawings counted through `_cells`, the held frame equal to the drawn
  one cell for cell, the ground still moving under it, a turn drawing
  again.
* **The can is seated at the top of its band, not centred.** Centred,
  whatever the band had over the can's height was split above and
  below - and on a terminal whose cell the view could not measure,
  drawn at an assumed 2.0, that was a row of air under the legend that
  nothing explained. Seated, there is nothing between the last legend
  and the motor at any aspect (measured 2.0 and 2.3), and the leaders'
  hop row went with it: the corner glyph turns each run down in its
  last cell and the tube it lands on is a column. STATUS now says
  `cell 2.31 tall measured` or `2.00 tall assumed`, because a number
  that was not measured is worth a word on the page.
* **The bead is U+0298, drawn by Consolas itself.** The circled bullet
  is narrow and still came out squeezed to half its width, three times
  reported: Consolas has neither it nor a single braille cell, so the
  whole drawing goes to the fallback font and the fallback draws the
  bullet into a cell that is not its own. The bilabial click is the
  same mark - a ring round a dot - and the terminal's own font carries
  it. Narrow round marks Consolas has, for the record: `◦` `◌` `∙` `ʘ`.
* **The foot gauges' track is the gauge's own height**, `⠇` a cell in
  the track's grey, as the tubes' track runs the tube's whole width. A
  single dot on the middle row beside a level three dots tall was a
  scale a third the size of what it measured.
* **The thermistor can read a few tenths above a leg that has just
  stopped, and that is physical.** Measured on the stand-in: under
  25 A the NTC sits at 28 C against legs at 83; load off, the legs fall
  to the board in five seconds and the NTC, with thirty-six seconds of
  its own mass on the FR4, follows behind. A leg cannot fall below the
  board it sheds to, and the NTC sits on that board - so it leads only
  while the leg is hot and lags only while the leg cools. Not clamped:
  a display that forbids the sensor to read above the source would be
  lying about the sensor.
* **Wrong, and bounded now, 2026-09-05.** The bullet above argued from
  a rod with its ends held, and the leg is not held: `thermal_step`
  sheds it through ONE path, `(t - board) / to_board`, the copper the
  thermistor sits on. A node that drains only into its neighbour cannot
  fall below that neighbour, and a passive link in a chain fed from one
  end cannot read above that end - the series network of the thesis in
  `docs/papers` (Ziegenfelder 2022, 2.3, fig. 2.3), which the bench
  pointed at when the page showed an NTC warmer than the switches that
  heat it. The lagged state hung off the side of both nodes with its
  own 47 s and could: measured on the core, 25 A on the V leg for two
  minutes then off, the reading was 5.96 K over the leg 13.8 s after
  the stop; 28.8 K at 60 A; 80 K at 100 A. Under load it never was
  (0.65 K under, 2.9, 7.9) - the defect was the way down alone. The
  reading is bounded by the pair it sits between now, in the core and
  in the stand-in's own copy of the lag; after the stop it is +0.00 K
  at every current and it still lags on the way up. The lag is the
  patch's; the bound is the chain's.
  `test_the_thermistor_never_reads_above_its_source` holds both.

* **The thermometers went dead because the demo lost its load, not
  because the model changed.** Reported three times as "nearly static"
  and blamed on the thermal recalibration. Measured: the stand-in warmed
  identically at `60ae1f3`, at HEAD and in the working tree under the
  same drive (`driven.py`, 12 A: driver_u 0.12 to 0.15 of its ceiling in
  3 s). The VIEW differed - at 600 frames HEAD had the winding at 98.8 C
  and SWITCH SOA 50 %, the working tree 22.9 C and 20 %. `main` had been
  split into `_link`, and `demo_defaults` - which puts the model's load,
  `args.b`, onto `args` - landed after the `preflight` that hands the
  model its parameters. Unloaded, the model drew no current. Order
  restored, 99.2 C and 50.8 %; `test_views.py` runs 200 frames and holds
  the winding above 45 C. The recalibration IS a smaller change: at
  12 A the leg node reaches 0.146 of its ceiling in 3 s where the old
  `LEG_TO_BOARD` of 45.6 gave 0.220 - deliberate, and documented under
  the thermal findings.
* **A cell is eight dots and one colour, and three rules for which
  colour were tried.** Highest RANK among lit dots: a tooth outranks
  the yoke, so the yoke ring came out chopped into phase-coloured
  segments that changed with the drive. MOST DOTS: mended the yoke and
  broke the can - the magnet band's outer edge and the can's inner ring
  are 0.10 of the radius apart, 3.3 dots against a cell four tall, and
  at twelve o'clock the shared cell is mostly magnet; the ring went
  amber in three places, ringed in red on the bench. LINES BEAT AREAS,
  then most dots: a line that loses its cell is a broken line, an area
  that loses one is a dot short at its edge. Yoke ring wholly its own
  colour (0.80 under the vote), no can-ring cell lost to a magnet
  (three under the vote).
* **The air gap is under a cell tall, and no colour rule makes that
  right.** 0.08 of the radius is 2.6 dots against a cell four tall, so
  at twelve and six o'clock one cell holds a magnet's inner edge and a
  tooth's tip - 240 such cells over 48 poses - and a cell is one colour.
  Three answers, each measured and each seen on the bench: the gap held
  open to a cell's diagonal - no shared cell, and the teeth 1.9 dots
  short, "the slots are too small"; the tooth given the cell - green on
  the band, "colour faults in the rotor"; the magnet given the cell -
  amber on a tooth tip, "the rotor bleeds into the stator". The drawing
  keeps the teeth at their full fraction and gives a shared cell to
  whichever has more of it, which is the magnet in all 240. The south
  arc is a magnet, not a line: counted as a line its fringe took 46 of
  those cells. **Which fault to carry is the bench's choice, not the
  drawing's**, and the other two are one constant away.
* **The shaft sensor's stroke is drawn through the magnet band, and it
  wins its cells outright.** Three placements before that. In the AIR
  GAP it stood over the slot mouths where the teeth show their current
  and read as a second indicator drawn across the magnetisation -
  trimmed to a dot clear of both sides and made to yield in shared
  cells, it was reported there still, because its dots were still
  there. OUTSIDE THE RIM, beside the bench's own mark, it reached the
  gutter at three and nine o'clock (one column of air) and found no
  empty cell at some angles (the rim's fringe). THE BAND has room, is
  the rotor, and is what the sensor's angle is compared with: a slipped
  pole is the stroke standing off a magnet's edge. Yielding to the
  rings it owned no cell in some poses - the can's inner ring's fringe
  reaches the band's outer cells - so it takes everything: at that one
  angle a ring cell goes white and the stroke reads as reaching the rim.
  Measured over 48 poses: no cell shared with a tooth, none in a
  gutter, at least one cell its own in every pose, at most one rim cell
  taken.
* **The mercury's top is drawn at the dot.** A track dot inside the
  cell the level ended in took the level's colour, so every bar's top
  read `⣿` and a tube that fills in cell steps barely moves. The end
  cell holds level and nothing else: `⣀`, `⣤`, `⣶`, `⣿`, one dot a step
  (the foot gauges `⠇`, `⠿`, one lane a step).
* **The box is sized to the can on this terminal.** The band was a
  constant fifteen rows; the can is 13.5 rows at a two-by-one cell and
  11.7 at 2.3, and the spare was split above and below - 1.5 rows of
  air over the motor on the bench's terminal. `fit_rows` measures the
  aspect once and sets the height; `_Radii` measures its height in the
  same units as its width (it assumed a square dot, the gap flagged
  earlier), with `+ 2` in the diameter for the dot it keeps off each
  edge. 14 rows at two, 13 at 2.3, 12 at 2.5.
* **Consolas has neither U+29BF nor braille.** On a terminal at VS
  Code's default font the whole drawing is rendered by the fallback,
  Segoe UI Symbol, which is why the bead reads oval: the glyph is drawn
  by a font whose cell is not the terminal's. It is not a width flag -
  U+29BF is unambiguously narrow. Round marks Consolas draws itself and
  that are narrow: `◦` `◌` `∙` `ʘ`; `●` and
  `○` are in the font but ambiguous. A choice for the bench, not
  made here.
* **The runner must survive what it reports.** A failing check whose
  detail held eight braille cells raised UnicodeEncodeError inside the
  summary on a console at its codepage, and the tally never printed.
  `run_tests.py` reconfigures its own stdout to replace.

* **EAST ASIAN AMBIGUOUS WIDTH is what shears a terminal drawing.**
  Reported from the bench as the composite bleeding colour inside its
  own box and the indicator reading oval. Unicode does not decide the
  width of `◀ ▶ ▲ ▼ °`: a terminal set for East
  Asian text draws them two columns wide and every other one narrow, and
  it is a SETTING, not a font. Wide, the mark doubles, everything after
  it on the row slides a column, and the colour runs slide with it. The
  small triangles `◂ ▸ ▴ ▾` and `ᵒ` are the same
  marks and unambiguously narrow. **The bead was not the culprit** -
  U+29BF is narrow - and the fallback built for it chose U+25CF, which
  IS ambiguous: the safe substitute was the only unsafe character in the
  pair. Braille is narrow by definition, so only the furniture was ever
  at risk. `test_views.py` holds the rule.
* **An ordered dither on a fringe crawls and does not help.** A
  quarter-covered dot lit at a quarter of the positions sounds like more
  resolution; the threshold is fixed in SCREEN space, so a shape moving
  across it has its fringe pop on and off in a standing pattern. On a
  still picture it only fattened lines here and there - measured on a
  ring, `⣄⣄⣀` against `⣄⣀⣀`. Coverage
  alone, thresholded at half a dot, is smoother and cheaper.
* **A ring wants an analytic coverage, not a band test.** `abs(radius -
  at) <= line` is a yes, and four corner samples then quantise a stroke
  to fifths - on a thin ring that is solid or nothing per dot, which is
  the staircase. A ramp across the stroke's own edge grades it, and the
  grading is what draws `⣀` where an arc grazes the bottom of a
  cell, `⣤` halfway in and `⣶` nearly through. 17.7 ms a frame
  became 22.1.
* **A rotated sampling grid beats the four corners at no cost.** Four
  samples on a square give a near-horizontal or near-vertical edge only
  two distinct coverages - both samples of a row cross it at once -
  which is exactly where a circle looks worst. The four-rooks pattern
  gives those edges five.
* **The instruments are not dead, they are honest.** Five thermometers
  reading the bottom tenth of their tubes looks broken; measured on the
  stand-in at 12 A the spends are 0.03 to 0.17, which on a fifteen-row
  tube is one or two rows. A board at a tenth of its ceiling should read
  a tenth (invariant 10), so the scale stayed and the margins gained a
  decimal instead - whole percent stood still while a node climbed four
  degrees.
* **Not fixed:** `_Radii` sizes the can with `min(width * 2, height * 4)`,
  which assumes a square dot. Only bites where a cell is LOWER than two
  by one, and then the can can overflow its box vertically.


* **`coaxial/braille.py` holds the block and the vocabulary.** Glyphs
  picked by hand at call sites (`chr(0x2824)` for a run, `chr(0x2847)`
  for a drop) stay a handful and the corners come out wrong: a run
  ending against a column under it is two marks that happen to touch,
  and `chr(0x28A4)` has to be decoded before it can be reviewed. A
  corner the line ENDS at is a hook (`⠲`); one it falls THROUGH reaches
  the cell's floor (`⢲`) or it breaks against the row below.
* **The dimmed track was drawn at two different rates.** The gutter
  tubes put a dot every other row in ONE lane, so the empty half of a
  thermometer was narrower than the mercury under it; the flat gauges
  put one every FOURTH dot column, which is a dash in every other cell.
  Both read as some bars having a scale and some not. One dot a cell,
  both lanes, everywhere.
* **The pointer bead has been wrong three ways.** A radial spur read as
  a tick at the top of the can and a dash at its sides. A square of dots
  centred on the rim was clipped to the silhouette and what survived was
  a crescent, cut differently at every angle. A sampled disc seated
  inside the rim was round and spread over three cells - a smear on the
  band. A dot is square, so FOUR IN A SQUARE are round: always the same
  four, over one cell or two or four depending only on where the block
  falls across the grid.
* The two gutters must be the same width or the legends are not
  symmetric: a legend's arrowhead sits on the machine's own edge, so a
  left gutter of eight columns against a right of seven put nine
  columns of leader on one side and eight on the other.

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
* **The glyph ramp was two steps above blank and the 3D lived in it.**
  ' .:' gave a leaning face one step to fall through, so a board drew as
  a flat carpet with a rim and the lighting could only change its colour.
  `raster.SHADE` is nine rungs by dot count holding all 256 patterns of
  U+2800 - 1, 8, 28, 56, 70, 56, 28, 8, 1 - ordered smoothest first, the
  phase cubed off the grain hash so a flat face mostly wears the even
  arrangement. With colour the rung comes off `heat`; fitted on the
  shipped board at zoom 1, 515 lit cells span heat 0.92 to 4.70 with p5
  and p95 at 1.87 and 3.83, which is 0.22 to 0.52 of DIMMEST to the top
  of the glow.
* **A dot is one bit and `SUBDOT` samples four corners.** Read as "any",
  a shape covering a quarter of a dot lit it whole: every arc in the
  rotor and the protractor came out a dot fatter than it is. Read as
  coverage against a 4x4 Bayer - half a dot or more always lit, the
  fringe dithered - the rotor draws 97 distinct patterns and the
  protractor 72, where both were a handful.

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
