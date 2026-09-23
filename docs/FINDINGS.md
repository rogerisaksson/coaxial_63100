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
| --- | --- |
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
  | --- | --- | --- |
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
  - the envelope ran once per gap, not once per step;
  - `STEP_S` was 1.0 s, chosen as a fifth of the fastest node's constant
    + the right rule for integrating and the wrong one for acting, since
    the ramp is 300 ms wide;
  - the drive was sampled ONCE for the gap, so the model went on
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
  | --- | --- |
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
| --- | --- |
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
| --- | --- | --- |
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
  compendium's finding that radiation carries 30 to 40 % of the total
  heat dissipation under passive cooling and cannot be neglected. It is
  needed because the two shapes differ, so only their proportion lets
  them be scaled apart.
* What it does:

  | rise | K/W off the board | board temperature | needs, flat | needs, now |
  | --- | --- | --- | --- | --- |
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
  | --- | --- | --- | --- |
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
  | --- | --- | --- | --- | --- |
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
  resistance falls as it heats: solving `dT = P * 8.33 * (10/dT)^0.25`
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

* **A block read entirely after a wrap came back 9.02 s in the past**
  (2026-09-07, the bench finding daq_live_plot's phase plots wretched):
  `_timed` unwrapped each block's stamps on their own, which orders a
  wrap INSIDE a block and does nothing for a block whose every stamp lies
  past one - `to_host` then placed it a whole wrap early. Measured on the
  stand-in at 50 records/s: every record's `since` climbed for nine
  seconds and fell 9.02 s at the wrap, and the plot drew its window back
  over itself, the traces crossing the axis both ways. The acquisition
  carries the wrap count from block to block now (`_unwrapped`), the
  first stamp of a run picking its epoch from the host clock through the
  sync - twelve seconds through a wrap, 600 records, none backwards, the
  worst stamp 82 ms from the wall.
* **The stand-in invented records at the read's pace, not the clock's.**
  A clock-closed task answered fifteen a read whatever the interval: 245
  records a second from a 50 Hz task, their stamps running 3.4 s ahead of
  the wall per second, which is how the index reached +6 s "before now".
  A read answers what the interval produced since the last one, the
  fraction carried: 49.7 a second from 50, four a read at the reader's
  pace. `read(-1)` of a 5-record run at 500 Hz therefore arrives over
  more than one read, as it does from a board; the finite-run check reads
  until it has them.
* **The stand-in's phases swept +-57 A on a stage that was down, and a
  tare could not zero them.** `values._sweep` turned the three phases
  +-9000 codes at 0.14 Hz "so the meters had something to show"; a record
  used it whenever the drive carried no current, and a tare through the
  analog path stored that moment's value as the zero - the two paths could
  never agree, and a live plot of a machine at 4 A sat under 50 A of
  fiction. The phases carry the machine's current and nothing else now,
  on both paths from one `phase_codes` - the DAQ's own angle per record,
  the drive's command angle for a read - over the rest offsets NOMINAL
  keeps ("roughly what a live board reads"). A tare then zeroes the
  records: 1431, -7989, 396 codes stored; the currents +-4 A about -0.1
  to -0.3 A after, seven rising zero crossings in 2.16 s at 3.5 Hz, the
  gates 0.46 to 0.54. The meters lost their invented motion and were
  given a machine instead: the bridge page on the stand-in holds a
  current vector turning at 0.14 Hz electrical and runs it 0 to 30 A and
  back over 45 s (`show_desk.demo_machine`, the stand-in's own record
  clamp lifted to 30 A for it); on a board the page opens onto whatever
  the drive is doing. The phases' burst ripple went from 2600 codes -
  +-16 A of invented noise at rest - to 60, the quiet channels' order.

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

## The device layer as a declared model, 2026-09-12

The bench asked for a systems-engineering model of the code, as clean
as it can be made: no nested ifs, no constants in the code, decorators
and multiple inheritance where they serve. Measured over
the device layer - `subsystem.py`, `board.py`, `protocol.py`, `wire.py`
and the sixteen subsystems, twenty modules - with an AST walk that
counts an `if` inside an `if` (an `elif` ladder is not a nest) and a
numeric literal in a function body other than 0, 1, -1 and 2:

* Nested ifs 16 -> 0; numeric literals 321 -> 83, of which 29 are
  `clock.py`'s NTP arithmetic and 16 `analog.py`'s call defaults, both
  left alone. The whole library's top-level modules, renderers
  included: 111 -> 95 nested and 1186 -> 948 literals - the rest is
  `thermal_ident.py` (16, the identifier's own arithmetic), `rig.py`
  (13), `machine.py` and the raster engines.
* Ten subsystems carried the same three-line `_op` with one byte
  different. `Device._op` frames it once from the byte the class
  statement declares - `class Thermal(Device,
  device=protocol.DEVICE_THERMAL)` - and a `Device` that declares none
  is a TypeError at import.
* The op codes were eleven sets of module constants, four in
  `protocol.py` and seven beside their subsystem. They are `IntEnum`s
  in `protocol.py` now (`ThermalOp.STATE`, `DriveOp.MODE`), which is
  what let `request_length` become two tables keyed on them instead of
  a four-deep `if` mirroring `cmd_length.c` by hand: `DEVICE_REQUESTS`
  for the fixed shapes, `GROWN_REQUESTS` for gate op 2 and DAQ op 4.
  The suite's prefix sweep drives both oracles and they still agree.
* The wire's scales - centi, milli, micro, nano, Q16.16, a fraction of
  255 - are named once in `wire.py`, and a reply is read in its unit:
  `r.centi()`, `r.milli('u32')`, `r.q16()`, `r.fraction()`; a request
  is packed with `milli(k_per_w)`. `r.maybe('u32')` is an appended
  field or None; `pages()` walks a paged reply, and five copies of the
  `total, first, count` loop - the map's two pin sections, the parts,
  the record's tail, the thermal graph - are one each.
* Three decorators say what a method wants: `afe.powered` (refused
  while AFE_ON is off - `read_all`, `noise`, the burst behind the
  cooked readings, and `tare`), `subsystem.remembered` (the channel
  table, the map, the scaling and the record, cached until
  `refresh=True`) and `subsystem.forgetting('read')` on every writer
  of the record.
* FOUND BY THE DECORATOR: `Calibration.defaults()` never dropped the
  cached record, so a `read()` after it answered the record the board
  no longer held until another writer ran. Every writer forgets now,
  `defaults()` included.
* FOUND BY THE MANIFEST: the stand-in's broadcast node listed the
  subsystems it refuses by hand, and the list had missed `thermal` and
  `power` - on unit 0 those two answered AttributeError instead of the
  broadcast refusal. `SimulatedBoard.__getattr__` refuses every name on
  unit 0 now, and the real board's composition is its class-level
  declaration (`system: System` ... `observer: Observer`), read back by
  `Board.parts()` for `__init__`, the rig's early handles and the
  structure suite - none of which lists a name any more.
* `PolledSensor.configuring` is concrete on the interface: the IMU, the
  shaft sensor and both stand-ins carried the same seven lines.
* Tried and taken out: a registry filled by `__init_subclass__` (the
  board would then import sixteen modules it never names, which the
  unused-import check refuses - the manifest is the annotations); a
  `remembered` descriptor with `forget` on the method object (Pylance
  basic: "Object of type remembered is not callable" at 27 call
  sites); `functools.wraps` on the cache wrapper (pyright reads the
  wrapped signature through it and refused `refresh=`); an untyped
  wrapper (pyright treats an untyped decorator as identity and refused
  it too). The wrapper is annotated, and the tree reads 0 errors in
  basic mode.
* The offline gate afterwards: 2947 checks, 2625 passed, 322 skipped
  (parity and bench want a board), 0 failed.
* THE FRONT DOOR NEXT, the same day: `rig.py`'s thirteen nested ifs
  -> 0, each lifted into a named step - `_take_afe`, `_later`,
  `_release_stage` and `_release_afe` (close() is one loop of three
  steps with one try each, which is what its own comment already
  said), `_pin_called`, `_window`, `_split`, `_queued`, `_ended`,
  `_epoch_of` - and `pick()` is three comprehensions where it was a
  three-way ladder in a loop. Its refusals come in a fixed order now:
  a misspelt name before an unselectable one, where the ladder
  answered whichever came first in the caller's list. The pauses are
  named beside the constants they pace - `AFE_SETTLE`, `TAKE_PAUSE`,
  `RETRY_PAUSE`, `EMPTY_PAUSE`, `DONE_LOOKS` - so 12 literals became
  8, the remaining ones the pandas index arithmetic. Five suites
  through the rig (structure, daq_api, simulated, sensorless, views)
  and pyright unchanged: 0 failed, 0 errors.
* THE LINK, THE READER, THE STAGE AND THE VERBS (2026-09-13): the
  broker's eleven `if op == ...` were one ladder in `_do`; they are
  eleven functions and one table, `OPS`, and an op it does not list
  is refused in words as before. `frame_length` sizes an `ack` reply
  through `_ack_length`, and a frame's fixed bytes are named where
  the transport frames them - `HEAD_BYTES`, `CRC_BYTES`, `EXCEPTION`
  - where the 2s, 4s and 5s were spelt out per line. `transmit` pays
  its gap in `_pay_gap`; the reader counts its rate in `_count` with
  the window and the smoothing named (`RATE_WINDOW` 0.5 s,
  `RATE_MEMORY` 0.7); the stage's interlock is `_require_interlock`;
  the servo's unwrap is `_turned` with `TURN` and `HALF_TURN`; the
  bench table's closing line is `_verdict`, the gauges' label row
  `_label_row`. Fourteen nested ifs -> 0 across the seven files;
  structure, broker, daq_api, simulated, sensorless, views and the
  Modbus core: 0 failed.
* THE STAND-IN (2026-09-13): twenty-two nested ifs across six of its
  files -> 0. The DAQ's sensor words split by part (`_shaft_words`,
  `_imu_words`) with the Q points taken from `imu.SCALE` where 16384,
  256, 512 and 16 were spelt out; the gates' duty is `_gate_duty`;
  the read's pacing against the wall is `_pace`, its measured comment
  now its docstring; the capture's source mask is `_source_mask`
  over a named `SOURCES`. The thermal stand-in resolves 'tour' and
  'random' in `_resolve`, its load cycle answers idle by two early
  returns, and the gate drivers' counted hold is `_periods_left`
  against a named `PWM_HZ` where 50000 stood three times. The drive's
  eighty-line mechanics block is `_spin`, lifted whole with every
  measured comment, and the polarity pulse settles in
  `_settle_polarity` with its two invented readings named; the live
  motor parameters are a class table, `LIVE`. The GPIO stand-in's two
  witness pins are a table too, and the calibration's zero is one
  `next()`. Structure, simulated, daq_api, sensorless, views, mcp and
  the thermal core: 0 failed; pyright 0 errors.
* THE MCP SERVER (2026-09-13): seventeen nested ifs across its four
  files -> 0. The tools' argument coercion is a table looked up at
  the call (`_coercer`); a spelling's note is `_note`; `self_test`
  filters once; the IMU's feature write is `_imu_feature` and the
  orientation's rotation vector `_rotation_vector`, with its interval
  and its looks named where 20000 and 20 stood; `afe_power` splits
  into the broadcast order and the switch; `devices op=use` is `_use`
  with `_named` for the node a name picks. The renderer's identity
  lines and the IMU's state block are their own functions; the
  session's label and its discovery read flat. FOUND ON THE WAY:
  `open_session` checked `port is None` four times in a row, the
  same raise each time - a replace that matched more than once and
  nothing said so; it is one check now. Structure, mcp, the four
  ollama suites that run offline, simulated: 0 failed; pyright 0.
* THE MODEL RUNNER (2026-09-13): forty nested ifs across eight files
  -> 0; `replies.json_objects` keeps its five, a character state
  machine the structure suite exempts by name. The REPL's eighteen
  slash commands were one ladder in `command`; they are one method
  each and a table, `COMMANDS`, with the verb's aliases as keys. The
  turn loop's stale-answer path is `_stale`, a fresh call `_fresh`,
  the link's verdict `_note_link`; the prompt tag strips a side in
  `_without_side`. The capability probe measures Windows cores and
  RAM in their own functions, the POSIX load in its, the fitting
  choice in `_fitting`; the CLI's auto model and one-shot question,
  the client's local-only refusal and its out-of-memory note, the
  pull's layer change, the runner's prose stop, the sandbox's board
  hint and the tools' no-board answer and other-ports step are each
  a named function. TRIED AND TAKEN OUT: the sandbox's evaluation as
  a method of its own - the traceback then blamed sandbox.py, since
  `tb_next` drops exactly one frame and the test that says so
  failed; it is inlined flat instead. And `command()` had an
  UNKNOWN return type to pyright before the table, which is why
  seventeen test lines treating it as a string had never been
  flagged; with the type known they were, and they hold the result
  as a string now (`or ''`), the None contract unchanged.
* THE IDENTIFIER, THE DIAL, THE ENGINE'S FOLD AND SHADE, THE ATTITUDE
  FIT (2026-09-13): thirty-one nested ifs -> 0. `thermal_ident`'s
  `_judge` is a transition rule per state - `_judge_stable`,
  `_judge_converging`, `_judge_uncertain`, `thermal_ident_judge`'s
  switch - with `_doubt_whole` the way back to UNCERTAIN; `anchor`
  pulls the laminate, corrects the thermistor and degrades in three
  functions of their own; `step` takes a sample in `_sample`, whose
  thermometers are `_channels`, the update `_take`, the variance
  growth `_drift`. The C's parity suite still walks with the mirror:
  thermal core 144, 0 failed. The dial's classifier tests the bead
  and the needle in `_on_bead` and `_on_needle`, its raster takes
  the four corners as one list; the engine's art-plane hit is
  `_art_hit` and its shadow test one boolean. MEASURED, since the
  engine's inner loop was the risk: the attitude frame at 150x44
  over twelve poses, plus six dial faces, digest 9753d65d99c7 before
  and after - identical - at 132 ms a frame both ways (the same
  tree, the four files stashed and restored; a copy of the package
  elsewhere on disk had measured 177, its caches cold, and would
  have read as a 45 ms gain). render 79, views 206: 0 failed.
* THE RASTER ENGINES (2026-09-13): `wireframe`, `ascii3d` and
  `machine`, thirty-eight nested ifs -> 0, the last in the library
  outside `replies.json_objects`. The hot loops were folded, not
  called: the rasteriser's one-pixel triangle is three flat tests on
  `tiny`, the resolver's supersample takes its deepest hit with one
  conditional expression and its band pass is a comprehension, the
  gauges' owner is `max()`. What became a function sits outside the
  pixel loop or once per cell: the key light `_key_lit`, the ground
  `_segment`, the three-frame `_vote`, the wire cell and the
  deepened colour, the tooth's stub, the missing decimates. The
  caches' sizes are named (`MESHES_KEPT`, `OUTLINES_KEPT`). MEASURED
  the same way as the engine: twelve attitude frames, six dial faces
  and five machine sections digest 90b88ecc2153 before and after, at
  132 ms an attitude frame and 57 ms a section both ways. render 79,
  views 206, structure 621, simulated 254: 0 failed; pyright 0.
  NOT A MEASUREMENT WHILE SOMETHING ELSE DRIVES THE BENCH, again:
  the views suite run beside the offline gate and pyright's node had
  `show_orientation.py --simulated` die with MemoryError - the crew
  it forks for the decimates - and the structure suite count 624
  where it counts 621; alone on the machine, exit 0 and 621.
* A TIMED-OUT SUITE TOOK 106 MINUTES TO DIE (2026-09-13). The gate
  on the raster engines ran every suite in its usual seconds and then
  sat: `test_views.py CRASHED exit=None 6354.2s, TIMEOUT after 300s`.
  `run_one` used `subprocess.run(timeout=300)`, which kills the child
  alone and then waits for the captured pipe to close - and the view
  the suite had spawned, and the crew workers the view had forked,
  held that pipe until they died of their own accord. Standalone the
  same suite passed 206 three times that hour, 77 s a run, so which
  view hung in the gate the log cannot say. `run_captured` in the
  runner kills the whole tree - `taskkill /T` here, the session's
  process group elsewhere - and the views suite runs each view
  through it with its own 120 s, so a hang costs two minutes and
  names the view.
* THE TEST RUNNER AND THE ROTOR OBSERVER (2026-09-13): thirty nested ifs
  -> 0. The runner's smart plan is `_smart` with `_commits`, `_chosen`
  and `_tiered` under it; a changed path meets its rule in `_touched`;
  the crash report is one print. The view's eighteen keys are one
  function each and a table, `KEYS`; the foot's policy word is
  `_policy_word`; the stand-in's demo numbers are `DEMO_*` constants
  with their reasons beside them and the model's defaults
  `_model_defaults`. FOUND ON THE WAY: `demo_defaults` set `view_step =
  0.01` for the model stand-in and returned 0.1 regardless - a dead
  local - so `+` and `-` walked a 0.06 A torque current in tenths; it
  returns the step it computed, `DEMO_STEP`. The runner drove the
  structure suite (621) and the smart dry run through the new plan; the
  rotor view exits 0 on the stand-in; views 206; pyright 0.
* `Import "matplotlib.pyplot" could not be resolved from source` (the
  bench, 2026-09-13, every notebook). Pylance's wording for a module it
  has stubs for and cannot find in the SELECTED interpreter. This laptop
  has two CPython 3.14s: python.org's (`%LOCALAPPDATA%\Python\
  pythoncore-3.14-64`, the one `python` on PATH launches and setup.ps1
  installs into - matplotlib 3.11.1, pandas 3.0.5, numpy 2.5.2,
  pyserial, PyYAML, mcp, rich all present) and a uv-managed
  `cpython-3.14.7` under `%APPDATA%\uv` with none of the seven. The
  Python extension had picked the uv one. `.vscode/settings.json` names
  python.org's launcher as `python.defaultInterpreterPath` for the .py
  files; an interpreter already chosen by hand outranks the setting, so
  the click there is "Python: Select Interpreter" -> the python.org
  3.14. THE NOTEBOOKS NO LONGER DEPEND ON THAT CHOICE (2026-09-13, the
  bench: app_fixed_wing still flagged after the setting): an editor
  matches a notebook to a kernel by the kernelspec's NAME first and by
  the language's version second, and both interpreters are 3.14.7, so
  the name is what settles it. Every notebook now names
  `coaxial_63100`; `make_notebooks.py --kernel install` registers that
  kernelspec on the interpreter it runs under, argv its absolute path,
  and `--kernel status` says which python it starts, exit 1 unless it
  is the one asking - the line setup.ps1 reports as `notebook kernel`
  and registers on the python it fills. Measured: the uv python asking
  status answers that it needs ipykernel, exit 1; python.org's, after
  install, its own path and exit 0; `setup.ps1 -Check` prints the line
  ok; daq_session executed through the named kernel in 8 s, 18 cells,
  10 outputs, no error. The notebooks themselves were executed on the
  right one all along - none holds an error output.
* THE ROTOR OBSERVER'S MACHINE SCALES WITH THE TERMINAL (2026-09-13,
  the bench asking for the machine to scale with the terminal's size).
  `fit(aspect, size)` sizes the box every
  frame from the console's size: the width the page leaves beside the
  forty-column instrument column and the viewport's four columns of
  frame, the band it leaves under the five caption rows, the foot row
  and the page's four; `machine.layout` bounds the can by whichever
  binds, and `_width_for` inverts it so a machine bound by the rows
  is drawn in a box its own width, the thermometers against it
  rather than at the far edges of a wide terminal. Piped - the
  suites, a log - the nominal fifty-two. `--width` and `--height`
  fit a piped run to a terminal that size - the console's own width
  and height too, since a pipe is assumed eighty columns and the
  frame cropped the foot's WINDING to DING at the first try - so a
  raster is one redirect and `tools/ansi2png.py` away. MEASURED on
  the stand-in,
  the page composed on a recording console at three sizes and read
  in the raster: 100x30 draws the art 54 wide over a 20-row band,
  140x45 84 over 35, 220x60 114 over 50 - the can filling the height
  at every size, the legends' leaders landing on their gutters, the
  foot's three words on one row. views 206, pyright 0.
* ONE LANGUAGE IN THE FILES (2026-09-13, the bench asking for the whole
  tree in English rather than a mix, and for its own remarks not to be
  echoed back like a teleprompter). Every quoted request of the
  bench's, in Swedish - in FINDINGS, MODELS, TODO, CLAUDE.md, the chat
  page, the runner, the dial and the views' docstrings - is now what
  was asked, in English; the measured model answers and the questions
  that provoked them are described the same way; so are the render
  bench's two panel titles and its two messages, the Build-and-flash
  task's question to the local model and the thermal compendium's
  sentence on radiation. WHAT STAYS IN SWEDISH IS DATA: `BOARD_WORDS`,
  `_BOARD_VERBS`, `_QUESTION_WORDS`, `_OTHER_ACTIONS` and `SIDES`, the
  language module's lists and phrases, the classifier's measured
  questions in the tests and their expected lines, and the cp1252
  mangling of `läge` that is itself the measurement. Two passes over
  twenty-four files; a scan over the tree for Swedish letters and the
  fifty commonest Swedish words finds nothing else outside those.
  structure 621, views 206, the six runner suites 524, render 79.
* THE VIEWS AND THE STAGE READ FLAT (2026-09-13): thirty-two nested ifs
  across screen, menu, the chat page, the session, the desk, the thermal
  and rotor pages, the render bench, the angle page, the stage and
  rendershow -> 0. The shapes: a table where a chain of ifs was one -
  `SCROLL_STEP` for the arrows, `DRAG_REPORTS`/`BUTTON_REPORTS` for the
  console's mouse record, `ENTER_KEYS` where three files spelt the pair
  out, the chooser's typed choice as three lookups; a callback nobody
  passed as `_ignore` rather than a None checked at every call; a
  helper with one job where an if sat inside an if - the turntable's
  swell and breath, the chat's `_sent`, the desk's `_line_share` and
  `_clocked`, the orientation page's `_taken` with the deadband's
  measurement on it, the angle page's `_face`, the stage's `finish`;
  `rpartition` for the partial escape sequence; an early return in
  place of an else. The render bench's panels are ENGINE, ORACLE and
  EXPORTER now. structure 621, views 206, pyright 0; the menu answers a
  piped `3` with 103 and draws its frames as before.
* THE BENCH TOOLS READ FLAT (2026-09-13): thirty-two nested ifs across
  thirteen tools -> 0, and host/tools/ holds none. The Monte Carlo's
  job is a `Run` - the drive, the loop and the tallies one object,
  `tick` and `tally` its methods - and the run's clock is named
  (`LOCK_S`, `RISE_S`, `HOLD_S`, `FALL_S`, `RUN_S`) where the profile's
  defaults and the tallies both read it; `find_board`'s answers share
  `_reported`; tonecheck's two copies of the SGR-to-luma walk are one
  `_tone` with the reset rule passed in, its PNG header is judged once
  after the chunks, and its fit speaks English; build_and_flash's
  extension binaries are a table, its regions have bases beside their
  sizes, its warnings are a filter; deadtime_trim's rails are a
  comprehension; pulse names its PWM, the protocol its counted pulse
  arrived in and its slack, and hands the hold to `_held`;
  thermal_identify logs a reading in `_logged`; commission imports
  math rather than `__import__`ing it; switch's sweep ends are one pair
  either way; the rest early returns and helpers. Measured: structure
  621, drive core 81 - the Monte Carlo job through the Run, sigma_theta
  and i_peak as before - pyright 0 on the thirteen, and each answers
  --help.
* ONE SESSION SURFACE, DECLARED (2026-09-13): `port`, `baud`, `unit`,
  `bus`, `simulated` and `attached` on `coaxial_mcp.session.Session`,
  the stand-in's session and dbg.py's NoBoard, listed once in
  Session's docstring; the MCP tools, the runner's tools and the chat
  read them as attributes where twenty getattr defaults stood -
  `session.bus` is the port on a real bus, `attached` the board only
  when the link is already open. The tests' doubles carry the surface
  too: the ollama suites' scripted-board session says it has no port
  and is simulated, and the held-port doubles are the real Session
  with the link handed in. THE CODE HALF WENT UP FIRST: the tree was
  committed mid-patch (d471c26) and CI crashed six suites on
  `'SimulatedSession' object has no attribute 'bus'` - the ollama
  suites' own double, not the library's stand-in, which had `bus` all
  along. ONE FLAKE FOUND ON THE WAY: test_simulated's AFE-on check
  asserted the string 32768 absent from the analog table; MCUdie's
  nominal sits near mid-scale and a live reading walked through it
  once (CI 3.10, 02827e2). Frozen is every channel at the unpowered
  value - 32768.0 single-ended, 0.0 differential - and that is what it
  checks now. structure 621, tools 219, link 109, bus 28, runner 223,
  prompt 113, board 28, reply 23, render 32, language 12, mcp 50,
  broker 33, simulated 254; pyright 0.
* THE CHAT'S STATE IS DECLARED, AND THE MODEL CLIENT HAS A BASE
  (2026-09-13). `Chat` carried fifteen `getattr(self, name, default)`
  reads - `origin`, `io_log`, `tool_names`, `schemas`,
  `prompt_history`, `intent`, `compile_intent`, `_said_node`,
  `_traced`, `client` - because the suites build one bare with
  `__new__` and set what the method under test reads. Those are class
  attributes now, typed the way they are used: `tool_names` a tuple,
  `schemas` the specs or None (what set_tools leaves with no tools),
  `prompt_history` a tuple grown by rebinding, `origin` a pair or
  None; a method reads them. Six more reads asked the client whether it
  had `model`, `options`, `truncated` or `notes`: `client.Model` names
  that surface once, `Ollama` derives from it and so does every double
  in the tests - ScriptedModel, Dead, Recorder, Narrator, Flaky - two of
  which lacked `options` and would have crashed the runner's budget if
  a test had walked that path. The chat page reads `chat.toolbox` and
  `chat.close()` as the attributes they are, `_Claude` declaring it has
  no toolbox. structure 621, tools 219, link 109, bus 28, runner 223,
  prompt 113, board 28, reply 23, render 32, language 12, mcp 50;
  pyright 0; the chat page draws its frames.
* THE STAND-IN'S STATE IS DECLARED, AND SO IS WHAT A TRANSPORT CAN DO
  (2026-09-13). Twenty more `getattr(obj, name, default)` reads: the
  acquisition stand-in grew its noise pool, its ladder and its wall
  clock on first use and asked itself each time whether they were
  there - they are set in the constructor now, beside the parts the
  board wires; the drive's hand-set PWM and its accumulated shaft, the
  power stand-in's counter, likewise. A transport declares `address`
  and `stream` - only a broker's has either - so `connect` and the
  rig's streaming read them rather than probing; the stand-in board
  declares it has no transport; the observer reads `board.rig`, which
  both boards declare. The key reader counts its mouse reports from
  zero, the render page reads the count, the desk reads the rig's
  baud, the runner reads a client's tag. The stage kept the instrument
  column's scroll by setattr on the console, a try/except around it;
  it is a weak table keyed by the console now, gone with it. The first
  cut put the counters on the capture stand-in - the first `__init__`
  in the file, not the acquisition's - and `show_capture` and
  `show_desk` said so in the simulated suite. What stays as a probe is
  the I/O edge: a stream's `reconfigure`, `encoding` and `isatty`,
  `os.sysconf` on POSIX, a name looked up on the errors module, and a
  repr that must not raise inside a debugger. structure 621, daq_api
  75, simulated 254, views 206, broker 33, mcp 50, tools 219; the
  offline gate 2947 checks, 0 failed; pyright 0 on the fourteen files.
* A PLAIN MAPPING IS LIFTED TO A RECORD AT THE RIG'S BOUNDARY
  (2026-09-13). `columns()`, `series()` and `channel_names()` accept
  what `board.daq` hands out - a dict - beside the front door's
  `Record`, and carried twelve `getattr(record, field, None)` reads to
  tell the two apart at every field. `_lifted()` makes the dict a
  Record on the layout once, its start time and gap what it carries
  under those names or none - the same samples, in the same order, as
  the plain-mapping branch listed - and everything below reads
  `record.samples`, `.digital`, `.sensors`, `.start_time`, `.dt`. The
  tree's getattr-with-default count is fourteen now, from ninety-seven
  this morning, and every one left probes the I/O edge or the
  platform. daq_api 75, simulated 254, structure 621; pyright 0.
* NO `global` STATEMENT IN host/ (2026-09-13): fourteen, in five
  shapes. A once-built cache - the wire model, the parametric board,
  the face art, the toon and photographic meshes, the exporter's cube
  - is `functools.cache` on its loader, and the shadow casters ask the
  mtime-keyed mesh cache each frame the way their docstring always
  said, the parametric fallback cached beside it. A pool worker's
  state - the crew's solids and art, the farm's model, the Monte
  Carlo's library - is a `_Worker` holder the initializer fills, and
  `montecarlo.hold(lib)` is how the drive-core bench hands in the
  library it built. The key reader names the instance holding the
  mouse (`Keys.holder`). The rotor observer's box - width, rows,
  height - is one `Box` object `fit` sets from the terminal each frame
  and every drawing function reads, thirty sites, where three module
  globals were rebound. The thermal calibration's port is `hold`'s
  first parameter rather than a global main assigned. structure 621,
  render 79, views 206, drive core 81, mcp 50; pyright 0 on the eleven
  files; the render bench and the calibration tool answer.
* THE STANDARD LIBRARY IS IMPORTED AT THE TOP (2026-09-13). Eighty-three
  of the tree's 247 function-level imports were `import time`, `json`,
  `subprocess`, `ctypes`, `glob`, `shutil`, `argparse`, `math`, `re`
  and the like inside the function that used them - or a name the
  module already bound at the top, imported again - or `import time as
  _time` where the plain name was free. A scanner judged each: twenty-
  one redundant ones dropped, fifty-five hoisted into the file's first
  import block (a `rich` one beside the file's other `rich` lines, with
  the block's noqa), seven aliases rewritten to the plain name. What
  stays where it is: a `msvcrt` behind a try (the guard IS the import),
  the optional pandas/numpy/PIL/nbformat at their call sites by the
  tree's rule, and the relative and tree imports inside functions -
  164 - which are import cycles (wireframe and orientation, board and
  the stand-in, broker and board) or a page kept instant by loading a
  heavy module on first use. Those are the next item, one at a time,
  each tried and reverted where the cycle bites. structure 621, pyright
  0 on the thirty-three files; the offline gate 2947 checks, 0 failed.
* THE TREE'S OWN IMPORTS ARE AT THE TOP WHERE NO CYCLE BITES
  (2026-09-13). The 128 relative and tree imports left inside functions
  were tried one at a time: hoist it, import the module and the three
  package roots in a fresh interpreter, keep it if that works and put
  it back if not. Ninety-three moved - every view's `from screen
  import ...`, the runner's `find_board`, `pick_tests` and client, the
  chat's client and capability, the stand-in's `..drive`, `..clock` and
  `..imu`, the library's `.motor`, `.mesh`, `.commission`, `.protocol`
  and the rest. Thirteen bit and stayed: `coaxial_mcp.session` and
  `find_board` each way, the wireframe and orientation pair, board and
  the stand-in, broker and board. Also left: ten a comment beside them
  calls lazy or instant, four aliased, the chooser's five (a page kept
  instant by design), and the optional pandas, PIL and notebook
  libraries - the first cut hoisted PIL and nbformat, and the tree's
  own rule put them back. ONE REAL BUG ON THE WAY: the session page
  had a local `stage = steady(rig.gates.state)` in the function that
  later called `stage()` - the function import inside it had shadowed
  the variable back; hoisted, pyright said `None cannot be called`, and
  the local is `gates` now. TWO MORE THINGS THE GATE AND A BARE CLONE
  FOUND: the test runner's and the picker's `from coaxial_ollama.client
  import Ollama` inside a function was a seam - the runner suite swaps
  `sys.modules['coaxial_ollama.client']` for a broken module to stand a
  missing daemon in - and hoisted, the picker reached the real daemon
  (which answered 500, out of CUDA host memory) instead; the seam is
  the module attribute now, `clientmod.Ollama` read at the call and
  patched by the suite. And the editable install masks a tool's
  import order: twelve tools had their tree imports placed before
  their `sys.path.insert` root-finder and imported fine here, and
  would not on CI or a bare clone - measured with the editable finder
  stripped off `sys.meta_path`, 12 of 42 tools failing, 2 after the
  move (screen and stage, library modules the views import after their
  own insert, as before). Every tool's tree imports sit after its
  root-finder now, wearing the block's noqa - 42 more `E402` markers,
  the root-finder decision's price. Function-level imports 247 -> 69
  over the two passes, `import screen` 0.18 -> 0.32 s, the chooser's
  first frame 1.91 -> 1.86 s (noise). structure 621, views 206, runner
  223, pyright 0 on the thirty-nine files; the offline gate 2947
  checks, 0 failed but the seam, then fixed and its suite rerun.
* THE CYCLES, TAKEN APART - AND THE RED RUN THE LAST PUSH WENT UP WITH
  (2026-09-13). CI failed e9e9669 twice over. One: `_switch_board` in
  the chat kept a local `import find_board` in one branch after the
  pass hoisted the other, so the whole function's `find_board` was a
  local and the "nothing answered" path raised UnboundLocalError - the
  gate here had passed it because the ollama suites narrow themselves
  and that check was not in the draw. Two: the pass had hoisted the
  rig's `from coaxial_mcp.session import open_session` to the top, and
  with the session imported first - CI's order, not this machine's -
  the library's own import walked back into a partially initialised
  session. The tool's import check ran each module and the three
  package roots in one order; the fix runs both orders. What changed:
  the rig imports the MCP session at the call and says why, the chat
  and the CLI read `sessionmod.open_session` off the module at the
  call - the link suite swaps it, and a name bound at the top would
  not see the swap, the same seam as the picker's client - and the
  MCP session imports the broker and the stand-in at its top. The
  wireframe and orientation import each other as modules and read
  `orientation.OUTER`/`BORE` at call time, so neither touches the
  other's attributes while importing; the broker's error classes come
  from its top import; the port finder imports pyserial and the
  library at its top, keeping `build_and_flash` lazy - it imports
  `_text` back. Six more local imports whose name the module already
  bound are gone. What is lazy by design says so beside the import:
  `find_board` under a path insert in three places, `coaxial_mcp.session`
  in the rig, `build_and_flash` in the finder. Function-level imports
  247 -> 40; the ten shapes the scanner still counts are the chooser's
  five, the boot strip's, the optional libraries and two aliases.
  Import order checked both ways for the session, the library, the
  rig, the wireframe and the orientation without the editable
  install; structure 621, link 109, tools 219, prompt 113, runner 223,
  bus 28, mcp 50, broker 33, render 79, views 206; pyright 0.
* SILENT SWALLOWS SAY SO (2026-09-13). Forty-seven `try: X / except E:
  pass` in the host became `with suppress(E): X` - the same behaviour,
  the intent on the line, the handler's reason kept beside it; the
  converter's first cut passed a tuple of exceptions as one argument
  and pyright said so eight times, and the one `except QUIET:` tuple
  constant became `suppress(*QUIET)`. The library's two asserts - the
  CRC catalogue check at import and the dial's face-to-scales row
  count - are checks that survive `-O`, raising RuntimeError and
  ValueError; the blue-noise tool's likewise. Two lambdas bound to
  names are functions. The board's `hasattr(transport,
  'proven_dispatch')` is an attribute both transports declare. What is
  left of that scan: two swallows with a second handler or a finally,
  one `hasattr` in `Device.__init_subclass__` that IS the declaration
  check, and 927 numeric literals inside functions - the next host
  pass, one file at a time, by what each number means. structure 621,
  pyright 0; the offline gate 2947 checks, 0 failed.
* THE FIRMWARE, SAME STANDARD (2026-09-13, the bench: the target code
  too). A scanner for C - comments and strings stripped, braces
  counted, Allman style understood - over board/, comms/, modbus/,
  drive/, shtp/, thermal/ and filter/: 58 nested ifs (modbus_map 11,
  board_daq 8, board_thermal 6, board_pwm 4, board_imu 4, dev_uart 4),
  13 functions past eighty lines, 645 numeric literals inside
  functions (the thermal network's table and the test harnesses hold
  most), 167 file-scope statics - the `s_` state every module keeps,
  which is the embedded norm and not a finding. Baseline build: 0
  warnings, flash 195 532 B, DTCM 38 540 B. THE REGISTER MAP FIRST:
  `modbus_map.c` said the input-register layout twice - once in
  `input_reg_mapped`, once as an if-chain over address ranges in
  `read_reg`, 121 lines, eleven nested ifs - and a span added to one
  was a hole in the other. It is one table of spans now, base, width
  and reader per row; mapping, the extent and reading walk it; the ADC
  span's width is the board's count, asked at run time; the two clamps
  use stdint's limits; the high-first word split is one helper;
  `read_bit` is two conditions. Same wire behaviour by construction -
  the same addresses answer the same words and the same exceptions -
  but the map is not in the host-built core, so THE CONFORMANCE SUITE
  AT THE BENCH IS WHAT PROVES IT (`run_tests.ps1 -AutomaticHigh` with
  the board on). Build: 0 warnings, flash +344 B for the table and its
  readers; the map's nested ifs 11 -> 0, its longest function 121 ->
  22 lines. THEN THE SMALL SITES, thirteen across the command handlers
  and the smaller board modules: a guard inside a condition is one
  condition (`rd_left(in) > 0U && !rd_ok(in)`, `loop == OFF &&
  !Board_AngleInit()`), a repeated shape is a helper (`angle_held`,
  `imu_held`, `scan_mode_on` for the two ADCs CubeMX generated without
  scan mode, `power_lost`, `own_pwm`), the thermistor's clamp is
  `fminf`, and board_adc names its unit factors (`MILLI_PER_UNIT`,
  `CENTI_PER_UNIT`) where five literals said 1000.0f. Left alone, and
  why: `thermal_ident`'s two - a counter in an else-if ladder, and a
  floor inside a loop over the identified parameters - and the
  thermal network's calibration values, set once in a builder with a
  comment each, which are the model's definitions, not magic. Build:
  0 warnings; thermal core 144 (the thermistor path is host-tested);
  the rest is the bench's parity and conformance suites.
* THE FIRMWARE'S BIG MODULES, AND TWO THINGS SAID ONCE (2026-09-13).
  Nested ifs 29 -> 2 - the two in `thermal_ident` left by design. The
  DAQ engine: the rung's fall is an early return, the record into the
  ring is `put_record`, a sweep into the sums is `accumulate`, the
  clock-closed window is one condition, the tone's renormalisation is
  `tone_renormalise` behind `TONE_RENORM_MASK` and `TONE_TINY`, and the
  count-closed trigger is `trigger_due`. The thermal glue: the laminate
  share is one condition per field, the observer's borrow is a ladder
  of early returns over one `due`, the winding's ceiling is one call
  chosen by the node. The IMU: `power_lost` once, the init-or-note one
  condition, the wake ladder `woken_by_retry` with `IMU_WAKE_RETRIES`,
  `IMU_WAKE_RELEASE_MS` and `IMU_RESET_DRAIN` named. The PWM update
  interrupt: `hold_counted_down`, `hold_expired` and `land_next_triple`
  - no side effect in a condition, the else-if chain's fall-through
  kept. The UART driver: `ring_push` and `ring_pop`, the fault flag two
  conditions. The RTU byte handler: two conditions, host-tested. SAID
  ONCE: PRIMASK held and given back was the same four lines eighteen
  times in five modules - `board_irq.h`'s `Board_IrqHold` and
  `Board_IrqRelease` now, every site through them; and the unit
  factors - `board_units.h`'s `MILLI_PER_UNIT` and `CENTI_PER_UNIT`
  where board_adc and board_thermal wrote 1000.0f and 100.0f
  twenty-nine times between them. The first cut put the PWM helpers
  inside the interrupt's doc comment - the insertion walked back over
  ` *` lines and this comment uses `  *` - and the compiler said so;
  the tool walks back to the `/*` now. Build: 0 warnings, flash
  196 564 B; modbus core 77, thermal core 144; functions past eighty
  lines 13 -> 9. The rest of the proof is the bench's: parity,
  conformance and the live views over a flashed board.
* THE FIRMWARE'S LONG FUNCTIONS (2026-09-13). Nine past eighty lines;
  seven were several jobs in one body and are now the jobs, named. The
  DAQ engine: `Board_DaqConfigure` is `refused_before_fields`,
  `select_fields`, `refused_with_fields` and `begin_task` in that order,
  and reads as the four lines it always was; `feed`'s filter tail is
  `shaped_ready`, and the three-line window clear that stood in three
  places is `clear_window`. The thermal glue: the network from the
  record is `lay_bulk`, `share_bulk`, `lay_nodes`, `lay_edges` and
  `lay_winding` in the order the overrides stack, the winding's quarter
  into the iron named `WINDING_INTO_IRON`; the poll's slice is
  `step_slice` (the losses, the step, the identification, the room, the
  budget) and the one place the file acts is `hold_envelope`. The IMU:
  the feature re-apply is `reapply_due` and `reapply_one`, the two
  early returns at the top of the poll one condition; the write's drain,
  wake and gate are `wake_for_write`, the full-duplex transfer with the
  part's tail discarded is `transfer_frame`, and the earlier pass's
  doubled braces around the NOWAKE note are gone; the SPI timeout and
  the pre-write drain are `IMU_SPI_TIMEOUT_MS` and `IMU_WRITE_DRAIN`
  where four sites wrote 100U and one 8U. The RTU: `mb_rtu_service` is
  `take_frame`, `frame_ours` (counted, CRC-checked, addressed) and
  `answer`, the service loop itself thirty lines - host-tested, 77.
  LEFT AS THEY ARE, BY DESIGN: `thermal_defaults`, 195 lines, is the
  network's table with its measurements beside each number and splitting
  it would hide which measurement set which value; `h_gate_drivers_state`,
  88 lines, is the wire layout of one reply and the offsets a decoder
  depends on are only readable in one sequence. Build: 0 warnings, flash
  197 068 B; modbus core 77, thermal core 144; functions past eighty
  lines 9 -> 2, the scanner's literals 599 -> 593. The board modules'
  proof is the bench's after a flash: parity, conformance and the live
  views.
* THE FIRMWARE'S LITERALS, BY MEANING (2026-09-13). The scanner's count
  593 -> 406 across board, comms, modbus and drive, and the binary image
  is byte-identical before and after but for the build stamp - the
  committed tree built in a second worktree, both ELFs through objcopy,
  197 080 bytes each, four differing, and those four the minute in the
  two copies of `FW_BUILD_STRING` - which is the whole proof for the
  board modules, since they have no host tests. THE
  UNIT FACTORS ONCE: `board_units.h` moved beside board.h, where both
  the board layer and the command handlers see it, and grew
  `MICRO_PER_UNIT`, `PPM_PER_UNIT` (the same 1e6, a different meaning -
  a micro-quantity against a ratio), `PPM_WHOLE`, `MICRO_PER_MILLI`,
  `US_PER_S`, `MS_PER_S` and `KELVIN_AT_ZERO_C`; 130 sites in twelve
  files wrote the numbers. THE NAMES THE HEADERS ALREADY HAD: the
  request-length oracle reads as the protocol now - `MB_FC_*` from the
  slave's header, `DEVICE_*` and the ops from cmd.h, `DEVICE_HEAD` for
  the three bytes every 0x6E request starts with; the thermal handler's
  thirteen file-local ops moved to cmd.h as `THERMAL_OP_*` beside every
  other device's, which is what let the oracle name them. ONE ANSWER
  WHERE THERE WERE TWO: the ADC burst's sample bound was 1000U in
  cmd_board and again in board_adc - `BOARD_ADC_BURST_MAX` in board.h.
  THE REST: the ADC's `ADC_CODES`, `ADC_HALF_CODES`, `ADC_MID_CODE`;
  the A1335's `ANGLE_TEMP_LSB_PER_K`; the PWM's `PS_PER_S`, `PS_PER_NS`
  and `PROBE_SETTLE_SPINS`, and the BSRR reset half by its CMSIS name
  `GPIO_BSRR_BR0_Pos` where two sites wrote 16; the slave's
  `MB_COIL_ON`/`MB_COIL_OFF`, `MB_WRITE_SINGLE_LEN`, `MB_SERVER_ID_HEAD`
  and `_MAX`, and the specification's two user-defined function ranges;
  the drive's `PI` and `HALF_PI` beside its `TWO_PI`, and the model's
  noise generator as `MODEL_SEED`, `LCG_A`, `LCG_C`, `LCG_TOP_ONE`.
  LEFT AS NUMBERS, BY DESIGN: tables and defaults - the RCC prescaler
  dividers, the gate pin table (PE8..PE13, said in its comment), the
  thermal network with its measurements, the motor model's and the
  tune's defaults, the sine polynomial's coefficients - and the two
  wire lengths whose comment counts the bytes in prose (the duty op's
  9, 10 and 13). Build: 0 warnings, flash 197 068 B as before; modbus
  core 77, drive core 81.
* THREE THINGS THE FIRMWARE SAID TWICE, AND THE LAST LITERALS
  (2026-09-13). SPI2's four pins were a table in the IMU command and a
  bare 12 in the driver's pin check, both beside board_io.c's rows -
  `BOARD_IMU_SPI_PIN_FIRST` and `_COUNT` in board.h now, the command
  walking from the first, the driver comparing against it. A record's
  shape on the wire - the start time, a sum a field, a byte a pin, four
  words a sensor, the count - was written out twice, for the ring's
  buffer and for the stride: `DAQ_RECORD_BYTES` once. The ladder's
  climb and fall thresholds are eighths of the ring, and the 8 stood at
  four sites: `BOARD_DAQ_RUNG_EIGHTHS` beside them. THE REST: the
  A1335's word width, word mask, register mask, CRC mask, temperature
  mask and SPI timeout; the IMU wake test's two answers that are not a
  time (`IMU_WAKE_NOT_READY`, `IMU_WAKE_BUSY`), its drain and the three
  empties that mean quiet; the IMU command's tries, drain limit and
  default wake; the drive's `UINT16_MAX` where 65535 stood twice,
  `LOG_EPS_SCALE`, and `NANO_PER_UNIT` in the units header for the
  inductances, the inertia and the friction. THE ONE VALUE-LEVEL CHANGE:
  those six sites multiplied by 1e-9f and now divide by 1.0e9f - the
  same conversion within an ulp, the divide the more exact of the two.
  PROOF, two builds against the committed image: with the six divides
  and the pin loop put back, four bytes differ, the build stamp's
  seconds, so everything else is a rename; the final form is 8 bytes
  shorter - the divides and the table gone - and the pin loop and the
  conversions are read, not measured, until the bench flashes it. Left
  as numbers by design: the SHTP report field offsets, the report
  lengths, the log word counts, the RCC divider table. Build: 0
  warnings, flash 197 060 B; the scanner's literals 406 -> 370.
* THE HOST LIBRARY'S SECOND ANSWERS, SAID ONCE (2026-09-13). The
  scanner counts numeric literals inside functions - 927 across the
  four packages, 500 in the library - and most are one of two kinds
  that stay: a keyword default in a signature (an API's documented
  default is a definition) and the stand-in's fixture values (a
  hypothetical board's readings are a table). What went were the
  conversions and the physics written out in more than one file. The
  ADC's code scale - 65536 and 32768 - stood in scaling.py five times
  and again in desk.py, the stand-in's analog front end twice and its
  values: `ADC_CODES` and `ADC_HALF_CODES` in scaling.py, imported.
  The kelvin offset stood in scaling, dial, thermal and the stand-in's
  angle sensor: `KELVIN_AT_ZERO_C`. Rpm was `TWO_PI / 60.0` in motion
  four times, in commission and in sensorless: `RAD_S_PER_RPM` beside
  `TWO_PI`, with `TORQUE_FACTOR` (three halves) and `HALF_SQRT3` (as
  drive_math.c rounds it) beside it, where the stand-in's drive and
  the motor model each wrote them. The winding's quarter into the iron
  was 0.25 and 0.75 in the stand-in twice over: `WINDING_INTO_IRON` in
  thermal.py, the mirror of board_thermal.c's, and the stator's share
  written as one minus it. THE ONE THAT WAS WRONG IN KIND: the
  commissioning's gate-supply reading multiplied volts at the pin by
  6.7 - the schematic's 57k/10k, a second copy of `VGATE_ONBOARD` and
  deaf to the record's own divider (invariant 7) - it reads the
  record's scale now. The stand-in's clock was 475 MHz in system.py
  and 475 ticks a microsecond in daq.py twice: `SYSCLK_HZ` and
  `TICKS_PER_US` in values.py, with `RING_BYTES`, `ACCUMULATE_MAX`
  and `MASK32` for the ring, the sum's bound and the counter's wrap;
  the angle stand-in's registers by angle.py's names. The commissioning
  grid's 41 and 40.0 and the refine pass's 0.7 and 1.3 are
  `GRID_STEPS` and `REFINE`; the ANSI codec's palette arithmetic and
  SGR parameters are named ranges. Proof: structure 621, simulated
  254, daq_api 75, sensorless 138, views 206, render 79, broker 33,
  the offline gate, CI; the scanner 927 -> 849, the library 500 ->
  422. LEFT, BY DESIGN: the rest are defaults and fixtures, and the
  bench tools' 427 are a bench's parameters.
* THE LIBRARY REACHED INTO TOOLS/ FOR THE BOARD FINDER; NOW THE FINDER
  IS THE LIBRARY'S (2026-09-13). `tools/find_board.py` held the one
  implementation of "which port is this board" - the port listing, the
  USB-VID kind, the probe through `connect`, the port state, discover -
  and three library sites needed it: the broker naming a held port's
  path, `open_session` looking for the board, the MCP tools' bus
  header. Each one inserted `tools/` onto sys.path inside a function
  and imported the script there, which is a library depending on a
  script three directories up, a layering inversion, and the last of
  the function-level imports that were not a cycle. The probe half is
  `coaxial/ports.py` now - the same code, the same docstrings, reading
  `board.connect` at the call so a test can still patch it - and the
  three sites import it like anything else; `find_board.py` is the
  command line over it and keeps the SWD power check, which only the
  programmer can answer and is a tool's business. The test seam moved
  with the code: the link suite patched `coaxial.connect`, the package's
  re-export, and patches `coaxial.board.connect` now, the definition;
  the class-name check that read `find_board`'s source reads
  `ports`'s. TWO THINGS THE MOVE FOUND: the tool re-exported seven
  names it never used and the structure suite said so, so the model
  runner and the link suite take the constants from the library and
  the tool re-exports only what its own command line calls; and the
  link_diagnose tool had a local list named `ports`, which shadowed the
  module the moment it was imported - `listed` now, and it was the link
  suite's first run that found it. `ports.py` is in the runner's path
  map (broker, mcp, link) and ARCHITECTURE's module list; the structure
  suite grew four checks with the module, 621 -> 625, and the counts
  are synced across CLAUDE.md, run_tests.ps1, ARCHITECTURE and
  .counts.json. Proof: structure 625, broker 33, mcp 50, link 109,
  ollama tools 219, ollama board 28, every moved module imported first
  in a bare process with the editable finder stripped, the offline
  gate, CI.
* THE SESSION IS THE LIBRARY'S (2026-09-14). `coaxial_mcp/session.py`
  held `Session` - one lazily opened board, the surface every tool
  reads - with `Origin`, the label, the broker check and
  `open_session`, and the rig imported `open_session` from it inside
  its constructor because the pair at module level was a cycle: the
  library's front door depending on the MCP server's module, the
  server's module depending on the library. It is `coaxial/session.py`
  now, moved whole with `git mv`; the rig imports it at the top like
  any other module, the server, the model runner's command line and
  chat, the chooser and the suites import it from the library, and the
  stand-in's docstrings name it where it lives. The one definition the
  move doubled - `BROADCAST = 0`, which `protocol.py` already had - the
  structure suite caught on its first run, and the session reads the
  protocol's. Nothing in `coaxial/` imports `coaxial_mcp` or reaches
  into `tools/` any more; the function-level imports left in the
  library are the broker's two of `Board` (board.py imports the
  broker), the optional pandas, numpy and Pillow by the tree's rule,
  and three plain hoists - the transport's ACK, the NTP server name,
  the scaling module - taken with the broker next. Proof: structure 625, mcp 50,
  broker 33, daq_api 75, simulated 254, link 109, every moved module
  imported first in a bare process, the offline gate, CI.
* THE HANDOVER IS THE TRANSPORT'S, AND THE BROKER NO LONGER IMPORTS
  BOARD (2026-09-14). The broker needed `Board` for one thing, twice:
  handing USART3 from the text console to the binary protocol - the
  console key, a settle, the input discarded - for a port it takes and
  for a request whose first try met silence. board.py imports the
  broker at the top (to spawn and attach), so those two were the
  library's last function-level imports born of a cycle. The handover
  touches nothing but the transport, so it is `transport.hand_to_binary`
  now; `Board.open_binary` delegates and keeps its name for the stand-in
  and every caller. The three plain hoists went with it: the ACK reply
  shape in subsystem.py (transport imports nothing of it), the NTP
  server name beside the clock the stand-in already imported, the
  scaling module beside the two constants the stand-in's front end
  already took from it. What is left inside functions in the library is
  the optional pandas, numpy and Pillow, by the tree's rule. Proof:
  structure 625, broker 33, mcp 50, simulated 254, daq_api 75, the
  modules imported first in a bare process, the offline gate, CI.
* THE HOST'S MIRRORS OF FIRMWARE CONSTANTS ARE HELD TO THE C, EVERY RUN
  (2026-09-14). The stand-in accumulates to the board's bound, the
  thermal mirror splits the winding as board_thermal.c does, the
  identifier runs thermal_ident.c's constants, protocol.py is cmd.h's
  device and op numbers - each a copy by hand, and a copy that drifts
  is a wire that lies quietly. `test_structure` reads the `#define`s
  off the C now (suffixes stripped, arithmetic evaluated) and compares:
  seven named pairs (the winding split, the SOA floor in ppm, the
  accumulate bound twice, the ring, sin 60 and two pi as drive_math.c
  rounds them), every `DEVICE_*`, every `*_OP_*` against the matching
  enum by name and number in both directions, and the identifier's
  thirteen constants by name. Four checks, 625 -> 629, the counts
  synced. THE FIRST RUN FOUND A DRIFT: two gate-driver ops carried the
  same numbers under different names - `GAPRST` and `DUTYQ` in cmd.h,
  `GAP_RESET` and `DUTY_FINE` in protocol.py - harmless on the wire and
  exactly the kind of thing that stops being harmless the day one side
  is renumbered; the C took the host's names, two sites each, build 0
  warnings. Proof: structure 629, modbus core 77, CI.
* THE FIRMWARE BUILDS IN CI (2026-09-14). Every push ran the host
  suites and nothing else; a C commit's proof was the bench's build
  and a flash. `.github/workflows/firmware.yml` now configures and
  builds the tree on every push with the presets the bench uses -
  `cmake --preset Debug`, then the build preset, no cube-cmake - on
  Arm's GCC 14.2.Rel1, the major the ST bundle's 14.3.1 shares; fails
  on any `warning:` in the log, which is the bench's rule made the
  runner's; prints the size line; and uploads the ELF as an artifact,
  so a commit that says 197 060 B can be checked against what a clean
  machine made. Proven first here: plain cmake with the bundle's gcc,
  ninja and cmake on PATH configured and built the tree with 0
  warnings, so the presets stand without ST's wrapper. What the runner
  cannot judge stays what it was: parity, conformance and the live
  views over a flashed board. A red run posts its log tail as a commit
  comment, like the host's.
* THE PACKAGES' BROAD EXCEPTS, NARROWED TO WHAT THEIR REASONS NAMED
  (2026-09-14). Eighteen `except Exception` in the library, the MCP
  server and the model runner, each carrying a reason - a quiet board,
  the port listing, a model call, older firmware - and the reasons were
  three things. `errors.LINK_FAULTS` is the library's own errors and
  the port or socket under them, `(RigError, OSError)`: the broker's
  streaming loop, the IMU's second look, the probe, the rig's
  who-else-is-here, the runner's has-a-board and is-it-alive, the chat's
  link diagnosis. `client.FAULTS` is what a call to the daemon raises
  from outside the client - its refusals, the socket, the HTTP layer
  under urllib, a reply that is not JSON: the narration, the model swap,
  the intent compile. The MCP tools' `OLDER_FIRMWARE` is the link's
  faults plus the attribute and the key a newer question finds missing
  on an older board - and the first run of the runner's suites found
  that shape twice, a board double with no thermal observer and a
  system with no channel map, which is what the two bare guards had
  been covering without saying so. The port listing narrowed to OSError
  twice; the intent parser to the JSON errors. Three stay broad with
  their reason sharpened: the broker server's edge, where whatever a
  request raised must reach the client as an error; the reader thread,
  which carries anything across to the caller that raises it; and the
  sandbox, which catches BaseException by its docstring. A bug in the
  library no longer reads as a quiet board. Proof: structure 629,
  broker 33, mcp 50, simulated 254, daq_api 75, the runner's tools 219,
  link 109, board 28, reply 23, prompt 113, runner 223, the offline
  gate, CI.
* THE BENCH TOOLS' EXCEPTS, THE SAME WAY (2026-09-14). Twenty-one in
  tools/, and fifteen named one of the same three things: the test
  runner's model handles (preload, a handle for unloading, the release)
  and the picker's model call take `client.FAULTS`; the runner's and
  the picker's four `git` calls take the process errors, the revision
  count the parse error too; the runner's port listing takes OSError;
  the picker's reading of the model's JSON takes the JSON errors; the
  chooser's broker count and its board watch take `LINK_FAULTS`,
  imported inside the two functions beside `coaxial` itself, because
  the chooser's page is instant and imports the library off the frame
  loop; the capability probe behind `--model auto` takes what the
  probe can meet. Six stay broad, each with its reason sharpened: the
  two terminal probes, where termios is absent on Windows and has its
  own error class where it is not; the view loop, which shows the
  exception and keeps drawing because a bench page that dies hides its
  own reason; the chat loop, shown and kept; and the session page's
  stop path twice, where the stop must finish and the session still
  end. Proof: structure 629, runner 223, views 206, the offline gate,
  CI.
* THE ACQUISITION ENGINE IS A PORTABLE CORE, AND HAS A SUITE (2026-09-14).
  board_daq.c was the largest module in the tree with no host test: 1363
  lines, 54 file-scope statics, and everything from the ring to the tone
  generator tested only by the bench after a flash. Cut along the line
  the thermal core drew: `daq/` is the ring of records, the summing
  window, the anti-alias chain and its ladder, the tone generator and
  the live accumulator - portable C11 over the filter core, one `daq_t`
  with every number arriving as a cycle count or a sample, the interrupt
  hold and the sensor snapshot as two callbacks the glue supplies - and
  board_daq.c is the hardware around it: the converter reads on either
  clock, the cycle counter, the reference's power gate, the sensor
  snapshot, the checks against the converter, and the public API, 460
  lines. The engine builds lazily on first use, the shape the rest of
  board/ uses. Static asserts hold the engine's sizes to board.h's and
  its saturation bound to board_limits.h's. `test_daq_core.py`, 59
  checks through the host gcc, decodes every record as a host does: the
  ring's drop and high-water mark, the window's sum and count, the clock
  close and its saturation at 32767, pin duties and sensor words in the
  record's bytes, a finite task, power lost, the live accumulator, the
  sweep and its gate, the chain's decimation and refusals, the ladder
  climbing at six eighths and coming down after three looks at the low
  mark, the tone's exact debt and bounded burst and a sine to the LSB.
  THREE THINGS THE FIRST RUN SETTLED: the engine's feed guards on the
  task running, which the glue's poll and interrupt paths had guarded
  for it; the ring holds every whole record it has room for and not one
  more, so a 4096-byte ring at stride 14 holds 292 - the test's first
  guess of 291 was the test's; and the count-closed gate counts from the
  counter's zero, so a first trigger is due once an interval has passed
  since it, immediately on a running board. Build: 0 warnings, flash
  197 060 -> 197 856 B and DTCM +60 B for the struct's indirection at
  -O0; structure 629; the board glue's proof is the bench's, parity,
  conformance and the live views over a flashed board. Twenty-seven
  suites now, 3048 checks, the counts synced.
* A STRUCT PER FIRMWARE MODULE, THE FOUR LARGEST (2026-09-14). The
  thermal glue had 26 file-scope statics, the IMU driver 18, the PWM
  stage 16, the drive glue 12 - a module's state as seventy loose
  variables a debugger shows one at a time and a reset clears one
  assignment at a time. Each is one `static struct { ... } s` now,
  placed where the first stood, every doc comment kept on its member,
  every initialiser designated - the link volts at -1, the thermometers
  at NaN, the margin and the trip cap at one - and every use `s.name`,
  474 of them rewritten by a tool that refused a file where `s` was
  already an identifier, a declaration whose initialiser ran past its
  line, or a name that would collide. TWO THINGS THE TOOL LEARNED: a
  `static const` table is not state and stays in flash - the IMU's zero
  buffer went into the struct on the first pass and DTCM grew by its
  size, 316 bytes, before the rule; and a member's array bound has to be
  defined above the struct, so the IMU's feature count moved up. THE
  COST, and it is -O0's: flash 197 856 -> 201 980 B, because every
  `s.x` at -O0 is the struct's base plus an offset where a static was
  one address, and the Debug preset is what the bench flashes; the two
  modules on the interrupt path that matter, the drive and the sync,
  are compiled at -O2 whatever the preset and pay nothing. DTCM 38 576
  -> 38 572 B. The scanner's file-scope statics 167 -> 50, the fifty
  the smaller modules' handful each - the log ring's 9, the sync's 8,
  the acquisition glue's 5 - left for the same pass when they earn it.
  Build 0 warnings; the proof is the bench's: parity, conformance and
  the live views over a flashed board.
* THE WIRE'S SHAPE, HELD TO ITSELF ON BOTH SIDES (2026-09-14). A
  fixed-shape reply is one sequence of widths: the C handler writes it
  field by field, the Python decoder reads it field by field, and
  PROTOCOL.md tells it in prose. The C handlers carry the warning -
  an offset moved breaks every decoder for one bit - and the only thing
  holding the two sides together was the bench's parity suite over a
  flashed board. `test_structure` parses both now: the handler's
  `wr_*` calls, a `for` loop's body counted its bound's times off the
  headers, and the decoder's `r.*` calls through the reader's own
  vocabulary - `milli`, `micro` and `nano` as the widths they take,
  `q16` a u32, `flags` a byte, a loop over a tuple of names its length,
  a loop over a read's own result once, `maybe` an appended field
  counted like any other since this build writes them all. Three
  pairs, three checks: the gate drivers' state, the drive's, the
  acquisition task's. THE FIRST RUN FOUND ONE: field 26 of the gate
  drivers' state, the dead-time skew, written as `wr_u8` with a cast
  where cmd.h says `i8` and the decoder reads `i8` - the same byte, so
  no decoder was wrong, and exactly the kind of drift that stops being
  harmless the day someone reads the writer instead of the header. The
  wire has `wr_i8` now, two's complement in one byte, and the skew is
  written with it at both sites. The generated layout the suggestion
  asked for is the step after this one; what this settles is that the
  two answers a wire crosses cannot disagree without a suite saying so.
  Proof: structure 629 -> 632, build 0 warnings, flash 202 012 B, CI.
* THE WIRE'S SHAPE, EVERY FIXED REPLY (2026-09-14). Three pairs became
  fourteen: the gate drivers' state, the drive's state, setpoints,
  window, moments, model and observers, the acquisition task's state,
  the log's, the rails', the thermal observer's state, budget and
  edges, the cycle counter's read. What the parsers had to learn to
  get there, each from a reply that refused the first cut: a loop over a
  count the reply itself carries - the node table, the edge table, the
  rails, the setpoints - is a body repeated an unknown number of times
  on both sides, and the two bodies are compared; a loop over a header's
  count is read off any header in the tree, `DRIVE_LAGS` from drive.h
  as much as the phase count from board.h, and a literal bound is a
  number; `<=` and a cast in a loop's condition; a helper handed the
  writer or the reader is followed - `wr_field` on the C side,
  `_thermometer`, `_edge` and `_rail` on the Python side; a block that
  leaves - an early return, a continue - is an alternative path with the
  fall-through's shape and is skipped; a loop that reads nothing emits
  nothing; and the matcher backtracks over how many times a body
  repeats, because the field after twenty node temperatures is another
  i32 and a greedy run swallowed it. Two replies stay out by nature:
  the layout, whose rows carry strings and whose pins are optional, and
  the live accumulator, whose rows are counted by another reply's
  field list. No drift this time - the skew was the one - and the
  fourteen are held from here on. Proof: structure 632 -> 643, the
  counts synced, CI.
* A STRUCT PER FIRMWARE MODULE, THE REST (2026-09-14). The same pass
  over the six modules the first left: the log ring's 9 statics, the
  injected group's 8, the acquisition glue's 5, the angle sensor's 5,
  the link's 3 and the STO chain's 3 are one object each now, 162 uses
  rewritten. Two things the tool learned here: a buffer placed by the
  linker script - the acquisition ring in AXI SRAM, with its section
  attribute - is not state and stays where the script put it, like the
  constant tables before it; and a table with a multi-line initialiser
  - the UART driver's three ports - is refused rather than guessed at,
  so dev_uart.c keeps its four. The scanner's file-scope statics
  50 -> 17, and the seventeen are those: the port table, the placed
  buffer, the IMU's zero table, and the single flags of modules with
  one thing to remember. Build 0 warnings, flash 202 076 B, DTCM
  38 580 B; the proof is the bench's after a flash.
* THE HOST'S LONG FUNCTIONS, THE ONES THAT WERE SEVERAL JOBS
  (2026-09-14). The scanner lists forty past eighty lines, and most are
  one thing by nature: the raster engines, whose proof is a picture the
  bench judges, and the bench pages' mains, one page each. Six were
  several jobs in one body, the shape the firmware's were, and are the
  jobs now. The thermal stand-in's constructor lays the base network in
  `_lay_base` and starts the board in its room in `_start_in_room`,
  each with the paragraph that explained it; what is left of it is
  attribute by attribute with its line of why, 92 lines of which two
  thirds are those lines, and stays. The drive stand-in's
  observers are `_observers_skip` (the periods not stepped, in closed
  form), `_observers_window` (the window integrated period by period,
  ending at the rotor) and `_observers_blend`. The clock's sync is
  `_marks` over `_best_bracket`, `_ntp_or_pc` at both ends with one
  guard where there were two, and `_against_utc` for the rate and the
  epoch - as functions of the clock, not methods, because the stand-in
  borrows `Clock.sync` with itself as the receiver and the first cut
  hung them on the class where the stand-in could not reach them; the
  simulated suite said so. The runner's argument parser is four groups
  in its old order - the model, the turn, the board, the permissions -
  and the prompt loop's turn is `_turn`, with `_greet` before it; the
  link diagnosis is its steps, `_power_step`, `_ports_step` and
  `_answers_step`, each returning whether the checklist goes on. The
  bodies are unchanged and so are the comments; only the seams are
  new. Proof: structure 643, simulated 254, sensorless 138, daq_api 75,
  runner 223, ollama tools 219, link 109, board 28, the offline gate,
  CI.
* THE WIRE'S OTHER HALF, THE REQUESTS (2026-09-14). The replies were
  held field for field; what the library packs into a request and what
  the handler reads off it were not. `test_structure` pairs them now
  by the op's name alone - the dispatch table says which handler reads
  which op, the enums are already held to cmd.h, and the decoder names
  the op it sends - and compares the handler's `rd_*` reads to the
  `pack(...)` widths: a field read under an `rd_left` guard, a ternary
  or the block the guard opens, is optional and taken when it is there;
  a loop over a count is its body that many times; `rd_bytes` is
  whatever is left. Sixty-odd ops on ten devices, one check a device; a
  payload built some other way than a literal pack - the gate stage's
  on and off constants, the duty triples' concatenations, the IMU
  write's bytes - is skipped and named. THE FIRST RUN FOUND THE MIRROR
  OF THE MORNING'S DRIFT: the dead-time op's skew, packed `i8` by the
  library as cmd.h says, was read with `rd_u8` and cast - the same
  byte, and the same silence about it that `wr_u8` kept on the reply
  side until the reply check spoke. The wire has `rd_i8` beside `wr_i8`
  now and the handler reads with it. A request cannot drift on either
  side without a suite saying so. Proof: structure 643 -> 653, the
  counts synced, build 0 warnings, CI.
* CI BUILDS RELEASE TOO (2026-09-14). `-Wall` at `-O0` cannot warn of
  a use before a store, a dead store or a path that returns nothing -
  those need the optimiser's dataflow, and the bench flashes Debug.
  The firmware job is a matrix over both presets now, each with its
  size line and its ELF as an artifact. Built first here with the
  bundle's own cmake, ninja and gcc on PATH: Release configures and
  compiles with 0 warnings, 120 968 B of text against Debug's 202 100
  B of flash - so today the optimiser has nothing to add, and the job
  is there for the day it does. THE MEASUREMENT THAT IS THE BENCH'S:
  the struct pass cost four kilobytes because every member access at
  -O0 is a base plus an offset, and `-Og` would be debuggable and
  optimised; `daq.c` runs once per converter sample, the reason
  `filter.c` is at -O2 whatever the preset, and could join it. Neither
  is adopted here - the LOOP panel's cycle counters and the keepalive's
  worst gap are the numbers that decide, and they need a board. TODO
  carries it.
* PROTOCOL.md IS HELD TO THE HANDLERS (2026-09-14). The document was
  the wire's third answer and the one no suite read: cmd.h's enums are
  held to the library's, the replies and requests to the decoders, and
  the tables a reader trusts first were prose to every check.
  `test_structure` reads each device section's op table now - the
  backticked widths of the request and reply cells, an `Op N:`
  paragraph where a cell says `below`, "per node" and "x count" as a
  body repeated, a bracket as optional from where it opens, `str` and
  `bytes` as whatever follows - and holds them to the dispatch table's
  handlers, one check a device; a cell whose prose carries widths
  outside backticks is skipped and named. Six sections described their
  ops in prose (angle, link, log, time, thermal, power) and are tables
  now, in the form the other five had; the power ops were the one set
  named locally in the .c and are `POWER_OP_*` in cmd.h with a handler
  for the release, so the enum check holds them too. THE FIRST RUNS
  FOUND THREE DRIFTS: the gate drivers' skew stated `u8` twice where
  the wire carries `i8`; the IMU feature op stated `u8 took` where the
  handler answers nothing; the IMU probe stated no request where the
  handler reads a length and a select byte. The rewrite found a
  fourth: the thermal state's `u8 10` for a node count that has been
  twenty since MINOR 13. Two parser rules were learned on the way: a
  block that writes and then returns without an error is the reply
  itself (the IMU id's answer inside its retry loop, `cmd_took`'s took
  byte), not an alternative path to skip; and "then" before a span is
  not a repeat. Proof: structure 653 -> 665, eleven devices one check
  each, the counts synced, build 0 warnings (202 124 B of flash, 24 B
  for the power handler), CI.
* THE STAGE DRAWS ON THE CONSOLE, NOT ON ITS FLAG (2026-09-14). The
  bench opened BOARD ATTITUDE and it fell over on the first frame, in
  `scroll_state`: `cannot create weak reference to 'bool' object`. The
  attitude and angle pages handed `frame_of` their `is_terminal` flag
  under the name `console`, where the desk, the thermal observer and
  the rotor observer hand the Console; the rotor observer had met the
  same slip the day the column learned to page and fixed it in its own
  main with a comment, and the other two kept the flag. Before the weak
  table the slip was silent: `setattr` on a bool failed inside a try,
  `paged` read `.size` off the bool, the guard swallowed it and
  answered no room - so on those two pages the column never paged, the
  wheel scrolled a state the drawing never read, and the piped test
  runs saw nothing because a piped page takes the plain path before
  any of it. The weak table made it a crash on a terminal. THE FIX IS
  AT THE SEAM: `_fills` refuses a bool with a sentence, on the piped
  path too, so every page's two-frame run in the views suite fails on
  the slip - and the first run found three more pages handing the flag
  to `panels_of`, the table template, where it had been harmless:
  session, capture, gate drivers. All five pass the Console now, and
  the angle page colours its dial on `console.is_terminal` rather than
  on the truth of whatever it was handed. PROVED ON A TERMINAL WITHOUT
  ONE: every page's main run with its `stage()` swapped for a
  forced-terminal Console, 120 by 36, three frames each - ten pages,
  the chooser and the chat included, all left cleanly; the five with
  an instrument column paged it (attitude 3 of 3 boxes, angle 2 of 2,
  desk 3 of 3, thermal 2 of 5, rotor 4 of 7), the table pages have no
  column; and the template alone on 100 by 30 with eight boxes showed
  five, two arrows down moved it, the flag was refused. That runner is
  the scratchpad's `prove_page.py`; the views suite keeps the guard's
  check. views 206 -> 207, structure 665.
* THE SAMPLE PATH'S MEMORY TRAFFIC, BY INSPECTION (2026-09-16). The
  bench asked for fewer cache misses and RAM accesses without obscuring
  the code. The map first: `.data`, `.bss` and the stack are in DTCM,
  zero-wait and uncached, so struct layout there is a size question and
  not a miss question - `-Wpadded` over the tree's own sources with the
  target compiler counts 141 padded sites, all left as they are, since
  the fields group by concern and a hole in DTCM costs bytes, not
  cycles. Nothing on the fifty-kilohertz path reads a constant out of
  flash: the chain's coefficients, the drive's parameters and the
  ladder are copied into the structs that own them. Three things were
  left. THE RING WAS WRITTEN A BYTE AT A TIME: `put` copied a record
  into AXI SRAM byte by byte with a modulo each - forty bus writes and
  forty divisions a record, up to fifty thousand times a second since
  the TIM1 clock feeds it from the ADC interrupt - and the linker
  script's comment still said the main loop filled it a byte at a time
  and the bus cost nothing. It is two memcpys a record now, on both
  sides and wrap included, the 59 core checks holding the ring's
  semantics. THE CODE FETCHED FROM FLASH THROUGH THE ONE INSTRUCTION
  CACHE the main loop shares with the interrupt, so an ADC interrupt
  could begin by refilling the lines Modbus or the thermal observer
  had evicted; the path's objects - the control law and its observers,
  the anti-alias chain, the acquisition engine and its glue, the sync
  and PWM interrupts, the cycle counter, the injected ADC read and the
  HAL's ADC interrupt handler in front of them - are placed in ITCM by
  the linker script, by object with no attribute in any source, and
  copied out of flash by the startup beside `.data`: 28 976 B in Debug
  and 26 480 B in Release of the 64 K, the linker's stubs carrying the
  calls between the two (288 B in ITCM, 656 B in flash). And `daq.c`
  and `board_daq.c` joined `filter.c` at `-O2`, since a member access at
  `-O0` is a load and a store each. Debug flash 202 124 -> 200 856 B
  with the ITCM image counted - the tally had counted it as neither
  flash nor RAM, fixed - and 0 warnings in both presets. THE DATA CACHE
  STAYS OFF, on inspection as well as for the record's read-back: a
  streaming write into cached memory reads each 32-byte line before
  writing it, so the ring would cost more, not less, and nothing else on
  the path lives where a cache reaches. NOT MEASURED - no board here.
  The first flash is the proof the section and the copy are right (a
  wrong one hard-faults on the first ADC interrupt), and `test_bench.py`
  against its baseline, the LOOP panel's cycle counters and the
  acquisition's `worst` are the numbers. The next candidate, unmeasured
  too: `HAL_ADC_IRQHandler` is every flag check in the peripheral before
  the callback, fifty thousand times a second; a handler of the board's
  own in its place needs the .ioc to stop generating it.
* THE ROUND RASTERISERS WORK THEIR GEOMETRY OUT ONCE (2026-09-16). The
  bench asked for the host's simulators and renderers optimised too, so
  every page was profiled on a forced-terminal Console, 120 by 36,
  thirty frames each (`prof_page.py`, `prof_cum.py` in the scratchpad).
  The two largest single costs were the two round drawings, each
  classifying every sample of every dot from scratch every frame - a
  hypot, an atan2 and a walk through the rings and bands, four times a
  dot: the machine's cross-section 173 ms a frame on the rotor observer
  page (1.65 million classifications for thirty frames), the dial 107
  ms on the shaft angle page. A sample's radius and angle, and whether
  it lies on a ring that never moves, in a band the rotor or the drive
  decides, or in the air, depend on the box and the cell aspect alone;
  so each seat and each face keeps a sample table now, built on first
  ask and kept per size through `raster.table` (eight sizes, a runaway
  set clears them), and a frame only turns the rotor, stubs the teeth
  and moves the needle. The votes are kept in `SUBDOT` order so the
  sums are the same floats: 269 frames rendered from the committed
  library and from this one, over readings, drives, sizes and aspects,
  compared cell for cell - identical. Measured, a frame of each: the
  dial at 64x23 52.3 -> 8.9 ms, at 46x19 in colour 38.6 -> 6.5; the
  motor at 70x30 111.5 -> 19.9 ms, at 40x22 49.1 -> 9.1; the whole
  rotor observer drawing at 80x36 158 -> 36 ms. On the pages: the rotor
  observer's loop 410 -> 118 ms a frame (its observers' window shrinks
  with the frame, since the stand-in integrates the time that passed);
  the shaft angle page 204 -> 52 ms, which is its own 20 Hz pacing.
  views 207, structure 665.
* TWO SECONDS ON EVERY PAGE'S WAY OUT, AND WHERE A CONNECT WAITS
  (2026-09-16). The page profiles showed `rig.close` at 2.0 s on every
  simulated page: `_release_stage` asks `broker.clients()` whether
  another session shares the board, `clients` attaches on a 2 s
  timeout, and on this bench a loopback connect to the broker's port
  with nobody listening is not refused - the SYN is dropped, netstat
  shows no listener, and `socket.create_connection` waits the whole
  timeout: measured 2.012 s at 2.0, and 4.0 s for `localhost`, which
  tries both families. So every page's exit, every views-suite
  subprocess and every `attach` with the 10 s default paid for a
  question with a known answer. Two fixes at the two seams. The
  transport connects on its own `CONNECT_S` of one second and puts the
  caller's timeout on the socket for the asks - a serving broker
  accepts in the kernel at once, busy or not, so a connect that has
  not completed in a second is one nobody is listening for, and the
  long timeout was only ever meant for a reply waiting on the serial
  port. And a stand-in is served by no broker by construction, so a
  simulated rig's close does not ask - `self.simulated`, the rig's own
  word; the stand-in session carries no origin, which the first cut
  read and the stand-in suite caught. Measured, a page's whole run at
  three frames, start to exit: the shaft angle 0.84 s, the gate
  drivers 0.70 s, against 2 s of close alone before. broker 33,
  simulated 254, views 207, structure 665.
* THE TRIANGLE RASTERISERS DO A THIRD LESS PER PIXEL, FOR THE SAME
  FLOATS (2026-09-16). The vector drawing's cost is `engine.raster`
  and the shadow map's twin loop in `wireframe._shadowmap`: per
  triangle, per pixel of its box, two edge functions of the form
  `(x2 - x1) * (py - y1) - (y2 - y1) * (px - x1)`, scaled by the
  area's inverse. The first product and both edge slopes are the same
  for every pixel of a row; they are worked out once per row now, and
  the cell index once per hit. NOT the incremental stepping a GPU
  does: stepping a weight along a row accumulates rounding, and a
  depth test at a shared edge could then go the other way in the last
  bit. This keeps the same products in the same order, so every
  weight and every depth is the same float - forty frames of the
  chooser's turntable at 52x18 and the attitude page's board at 76x30,
  from the committed library and this one, identical to the character.
  Measured a frame: the turntable 36.9 -> 32.1 ms, the attitude board
  75.0 -> 61.7 ms. The rest of a vector frame is the shading pass and
  the outline, and on the attitude page the crew's band raster runs
  this same loop in eight processes. render 79, structure 665.
* WHAT THE HOST PROFILES DID NOT PAY FOR (2026-09-16). Measured and
  left, so the next pass does not measure them again. A MEMO OF
  DECODED ART LINES: rich decodes each line's escape codes into a Text
  every frame, and a page's lines repeat between frames - the rotor
  observer 87 % of its lines, the thermal observer 35 %, the attitude
  page 31 % over thirty frames. Built and timed on the captured lines:
  the attitude page 15.4 -> 15.4 ms a frame, the rotor 2.3 -> 0.3, the
  thermal 3.4 -> 2.9. The lines that repeat are the cheap ones, and
  what rich costs a frame - 12 to 46 ms by page - is its layout and
  segments, not the decode. Taken out. THE MESH WARM-UP: the chooser
  spends 3.2 s before its turntable turns - the STL parsed and centred
  in 0.5 s, three vertex clusterings at 0.65 s each - and the attitude
  page's pool does the same in six processes. A cache file beside the
  STL was tried before and is not wanted in the tree (`mesh.facets`),
  and one in another folder is the same decision moved, so it is the
  bench's to reopen. Under cProfile that pool's workers ran out of
  memory twice on this laptop while the page itself runs; one worker's
  parse and centring peaks at 152 MB by tracemalloc, 116 880 faces, so
  the profiler's slower parent lets six of them peak together - noted,
  not chased. Within budget and left: the thermal stand-in's step (54
  ms a frame at the page's 2.5 Hz), the drive stand-in's observer
  window (28 ms a frame, and it shrinks with the frame), the shadow map
  rebuilt each frame while the stand-in's board turns (a real IMU at
  rest hits its bucket), and rich's layout.
* THE SAMPLE PATH READ END TO END, AND A LINK THAT DID NOT RELINK
  (2026-09-16). The injected callback and what it calls were read for
  anything left to take out of the fifty-kilohertz path: it reads the
  three JDRs and the two rank-2 registers straight off the peripherals
  (the HAL's getter is a switch behind two asserts, and the tree took
  it out before), squares and sums, pushes the log ring, feeds the
  acquisition - which reads the pin mask straight off IDR, again by an
  earlier measurement - and steps the drive. Nothing is left to
  inspect out; what remains is the bench's to measure. Two more
  residents went into ITCM, since the path runs through them per
  sample: the log ring's push (946 B, per sample while a source is
  armed) and the pin mask with its count (160 B), 30 064 B in Debug
  and 26 944 B in Release. THE FIRST BUILD SAID OK IN A SECOND AND
  NOTHING HAD CHANGED: the toolchain file passes the linker script as
  a flag, which CMake does not track, so an edited script leaves the
  old image in place - the two objects were not in the map. The
  target now names the script as a `LINK_DEPENDS`; proved by touching
  the script and seeing the image's time move. Every ITCM build
  before this one had relinked for another reason (the CMake file or
  an object changed), so the images shipped were right; a script edit
  alone would not have been.
* THE 3D RENDER DID NOT LEAK; NUMPY'S IDLE THREAD POOL WAS CHARGED ONCE
  A PROCESS (2026-09-16). The bench reported a memory leak in the 3D
  render crashing VS Code. The machine was measured before the drawing
  was read for it: this laptop has NO PAGE FILE (AutomaticManagedPagefile
  false, no Win32_PageFileSetting), so the commit limit is the 23.7 GB of
  RAM; when asked, 21.5 GB was charged and 2.2 free, the editor, a
  browser, a VM and this session holding 12 GB between them, and
  Windows' own log (System 2004, low virtual memory) had fired seven
  times that day - python.exe processes of 690-750 MB each at 14:12-14:13,
  Code.exe on top at 17:54 and 17:59. Without a page file Windows fails
  the next allocation instead of slowing down, and the editor is what
  asks next. THEN THE PAGES, each run 400-900 frames on a forced-terminal
  Console into a counting sink with the process tree sampled every half
  second (the scratchpad's memwatch.py): the attitude page with its
  eight-worker crew 236 -> 251 MB resident over the first 45 s - the
  backdrop's 24 rung steps, the shadow buckets and the fits filling - and
  flat within a megabyte for the next 60 s, its workers flat at 47 MB
  each; the rotor observer 54 -> 58 MB and flat; the chooser's turntable
  223 -> 226 and flat. Every module cache was read for its bound as
  well - MESHES_KEPT 8, OUTLINES_KEPT 4, 64 shadow buckets, 24 backdrop
  steps a window size, FITS_KEPT 64, the seats' and faces' TABLES_KEPT,
  persist's three frames - none grows with time. Nothing to fix in the
  drawing. WHAT LEAKED WAS COMMIT, NOT MEMORY, AND AT IMPORT: a fresh
  interpreter after `import numpy` commits 499 MB at 27 MB resident -
  fifteen regions of exactly 32 MB, OpenBLAS's per-thread scratch for
  the sixteen cores, never touched but charged (VirtualQueryEx over the
  process, the scratchpad's regions.py; a bare interpreter with ten
  sleeping threads commits 8 MB, so it is not the stacks). `import
  coaxial` costs the same 504 MB because rig -> motion -> loop imports
  numpy at the top, and every process the views spawn imports coaxial:
  the attitude page is one page, six decimating workers and eight
  drawing ones - 7.5 GB of commit before a frame - and the MCP server
  this session ran sat at 545 MB commit against 49 MB resident all day.
  The first harness run died of it: MemoryError reading the 5.8 MB STL
  in a decimate worker, with 11 GB of RAM free. `tools/montecarlo.py`
  had met the same thing on the threadripper and set
  OPENBLAS_NUM_THREADS=1 in its own worker; the views paid it anyway.
  THE FIX IS WHERE NUMPY ENTERS: `loop.py` sets OPENBLAS_NUM_THREADS to
  1 (setdefault, so a shell that wants the pool keeps it) before its
  import - `import coaxial` 504 -> 22 MB, the attitude page's whole tree
  peaks at 1 269 MB during decimation and rests at 530 (page 242, eight
  workers 285); nothing in the tree multiplies a matrix worth a thread
  pool. test_structure holds it: numpy is imported at module level in
  loop.py alone and the cap precedes it (2 checks; 665 -> 667, 3085 ->
  3087). NOT MEASURED: the render inside the bench's own VS Code
  terminal - the sink counts 50 kB a frame, 1 MB/s at 20 Hz into
  xterm.js, which the alternate screen discards but the parser still
  reads. THE TARGET DOES NOT LEAK, BY INSPECTION: no malloc, calloc or
  free in the tree's own C (only the host test harnesses); the map
  links newlib's malloc through `-Wl,-u,_printf_float` (CubeMX's
  default) -> dtoa, and findfp's one-time buffer for printf - reachable
  only from the console's printf, which formats no float; `_sbrk`
  refuses past `_sstack` (the heap starts at `_end` 0x200090b8, 92 KB
  below the stack top, `_Min_Heap_Size` 0x200); every ring and table is
  static - the DAQ buffer, the rail users and their expiry, the UART
  ports. No board was attached today (COM4 absent, `--discover` none),
  so the heap end (`__sbrk_heap_end`, 0x200033c0) was not read over
  SWD; it is the number to read after a long session, TODO has it.
  THE HEADROOM IS STILL THE BENCH'S: with the fix in, the offline gate
  run beside a 600 MB tracemalloc pass of the same page (1.0 GB of
  commit free at the time) failed three checks with MemoryError, all of
  them the attitude page decimating - and passed 207/207 and 254/254
  run alone once that pass was stopped. The page's own peak is now the
  six decimate workers each parsing the 5.8 MB STL into Python lists,
  ~150 MB apiece by tracemalloc, 1 269 MB for the tree; that is the
  next thing to shrink if the machine keeps no page file. The
  tracemalloc diff of the running page was not taken: 900 frames under
  a 25-deep trace had not finished in twenty minutes, and the resident
  tables above are the measurement.
* "JAGGED" WAS THE DECIMATION, NOT THE ANTI-ALIASING (2026-09-16). The
  bench asked for the anti-aliasing code: the board's edge in BOARD
  ATTITUDE sometimes looked too jagged. Judged in the raster first -
  seven poses at the page's framing (108x40, zoom 1.267), rendered and
  rasterised, then the shallow arcs cropped and magnified - and the
  rim was two things a dot or four apart: the exact outline's dotted
  chain, smooth, and inside it the face's fill ending on a staircase of
  whole cells with a black gap between that varied cell by cell. THE
  GAP WAS THE DECIMATED SOLID'S OWN RIM: `mesh._clustered` snapped
  every vertex to its grid cell's MIDDLE, so a rim vertex moved up to
  half a cell in or out by where the grid happened to fall. Measured
  as the outer radius per five degrees against the exact mesh, in
  braille dots at the page's framing (88.6 a model unit): grid 32
  -2.8 to +2.8 dots, 48 -1.9..+1.7, 64 -1.4..+1.1, 24 -1.2..+15 (a
  dropped triangle's notch). The page earns grid 48 at that size, the
  menu's turntable 32. THE VERTEX IS THE CELL'S MEAN NOW - the corners
  that landed in it, summed, and the collapse test still on the cell
  middles - and the rim's shortfall is 0.00 dots in every bin at 48
  and 64, 0.3 mean and one bin of 3.5 at 32, 9.6 in one bin at 24;
  never outward, since a mean of points inside the disc is inside.
  Face counts unchanged (1809, 5570, 5645, 12430). In the raster the
  fill runs to the chain at every pose, and the parts' outlines draw
  whole where the lumpy depth buffer had hidden their far edges -
  the picture reads busier, which is the bench's to judge. THE
  ANTI-ALIASING WAS READ TOO, AND WAS HALF A CELL COARSE: the fine
  raster was 2x2 a cell - a quadrant one dot wide, two tall - so the
  face's clipping and the rim line could sit on two of a cell's four
  dot rows, and a shallow edge stepped by half cells. `engine.fine`
  makes the camera at the braille dots, 2x4 a cell and square
  (`project` takes an `aspect`, 0.5 for a cell, 1.0 for the dots),
  `fold` answers `reached` as the glyph's own bit order, `_dots` clips
  by the dot's bit, and `EDGE_GLYPH` is 256 masks, the reached dots
  beside a missed one, so the line sits on the row the edge crosses.
  And `_rim` lays that line OVER the face's reached dots instead of
  replacing the cell: replaced, the fill stopped a whole cell short of
  the silhouette wherever the rim crossed one - a second stair. COST:
  twice the fine pixels. The raster's rows are one span each, so the
  first miss after a hit ends the row - same tests, same floats - and
  at 108x40 on the grid-32 solid the dot raster went 79 ms to 69, the
  old 2x2 49 to 45; the setup per triangle is most of what is left.
  A full frame single-process at the page's size, the grid-48 solid,
  78 ms before to 100-113 after; with the eight-worker crew, the
  page's own way, 46.9 ms against the 52 recorded on 2026-09-06. The
  chooser's turntable and a run of four frames draw single-process
  and pay it. test_render: the fold checked at 2x4 with the mask as
  bits and `fine`'s shape (79 -> 80); the cube oracle passes with the
  mean vertices (a lone corner's mean is the corner, so the cube is
  exact now). views 207, simulated 254, structure 667; 3088 checks.
* THE GATE WAS FOUR HUNDRED SECONDS OF WAITING IN A QUEUE (2026-09-21).
  The offline gate ran its twenty-five suites one after another: 400 s,
  of which five suites are 350 - sensorless 151.5, the acquisition front
  door 72.0, simulated 59.8, views 36.6, the broker 30.7. Each of those
  was then measured by its own process' CPU time (GetProcessTimes on the
  child, the scratchpad's `par_probe.py`): sensorless 23.8 s of CPU,
  the front door 0.9 s, the broker 2.2 s, views 3.0 s with its pages
  apart, simulated 47.6 s - so all but the last are SLEEP, the stand-ins
  pacing themselves on the wall clock the way a board does, and a
  machine with sixteen cores was waiting on one sleeper at a time. The
  five run side by side all passed, each in the time it takes alone.
  `run_tests.py` now runs the suites that open the stand-in or nothing
  four at a time (`--jobs`, half the cores, four at most: this laptop
  has no page file and one views page is fourteen processes), started
  longest first by what each took last time - `.counts.json` grew a
  `seconds` section, measured rather than a list somebody keeps - and
  reported in the plan's order, so the tally reads as it always has.
  The five that may reach a board or hold the model (mcp, parity,
  bench, conformance, live) run alone after the rest, since the bench
  suite measures the link's own rates and conformance its frame gaps.
  A stopped run cancels what has not started. Measured: the gate 400 s
  -> 141.7 s, 2732 passed, 0 failed, every suite within a few seconds
  of its time alone; it now ends with sensorless, which is where the
  next seconds are.
* EVERY HANDLER IS NAMED FOR ITS DEVICE AND ITS OP (2026-09-22). The
  bench asked what was left to make uniform. Counted: thirteen command
  files name a handler `h_<device>_<op>`; `cmd_thermal.c` and
  `cmd_power.c` named theirs `op_<op>`, fifteen handlers, and both
  defined an `op_state` the wire checks could tell apart only by file.
  Renamed to the form the rest use, the wire-shape table with them.
  Proof of a pure rename: the committed tree built in a second worktree
  and both images compared byte for byte - 200 912 B each, twelve
  bytes differ, all inside the two copies of the build stamp (day,
  hour, minute, second; the earlier four-byte figure was two builds
  within a minute). 0 warnings, structure 667. The proof of the
  behaviour is the bench's parity, conformance and live suites after a
  flash, as for every C change without a host test.
* TWENTY-TWO NOTEBOOKS BECAME NINE PAPERS (2026-09-22). The bench asked
  for the examples filed to real uniformity - short academic papers
  with a pragmatic undertone - and merged to one per functional area.
  Counted first: twenty-two notebooks in as many shapes, nine figure
  sizes and no shared style, a helper defined in two of them, two
  patching `sys.path`, every one opening the device with the same line
  and closing it in the cell before last, so what varied was the form
  and not the flow. THE FORM IS THE BUILDER'S NOW: `host/tools/
  notebooks/parts.paper` lays every paper out - title, italic subtitle,
  an abstract, `## 1 Setup` with the knob and the open cell identical
  everywhere, numbered sections each a paragraph then the code that
  measures then the number read back, the close, Conclusions as one
  cell printing numbered findings with their numbers and one paragraph
  reading them, an `At the bench` paragraph saying what to run and what
  the stand-in could not show, and References to the tree's own files -
  so uniformity is a property of the builder and not a discipline each
  author keeps, and `test_structure` holds the generated files to it
  (nine checks, plus one that there is one per area). The figures go
  through `coaxial.figures`, one shape, matplotlib imported there the
  way the front door imports pandas; the propeller law fed to the
  stand-in's rotor is `Propeller.on_model` in `coaxial.motor`, defined
  once. THE MERGE: acquisition (the DAQ session, pandas, the live plot),
  link (the shared session, with the broker measured on a scripted wire
  in-process since it cannot serve the stand-in), sensors (IMU and
  angle), the power stage (gate drivers and the loss arithmetic),
  thermal (budget, model, identification), the drive (the Monte Carlo's
  three parts and the rotor observer), motion (stepper, servo, sensorless
  against the shaft, the speed loop, the propeller sweep), applications
  (the four missions), commissioning (auto_tune). Every old conclusion
  number has a home in a new one; the two Monte Carlo cells keep the
  one `sys.path` line the tree allows, since the tool sits beside the
  library. Written by eight authors in parallel, each verified by a
  skeptic against the old notebooks - the session died under them once
  and the link module came back cut mid-sentence, finished by hand -
  then executed one at a time on the stand-in: 33 to 56 cells, 6 to 66
  s each, 27 figures, no cell raised. Two things the run found: a paper
  that rebinds `top` from a spread to a plot panel prints nothing for
  its conclusions (the panels are named for what they show now), and
  the structure suite's copied-definition check counts an identical
  `CELLS = paper(...)` or `SETUP = []` across modules as a copy - so the
  package lays the papers out in one place and a paper that arms the
  stage says so in a section of its own. structure 667 -> 674.
* THE BOOTLOADER'S CORE, PROVEN ON A RAM FLASH FIRST (2026-09-22). The
  bench asked for an extremely slim bootloader: blank nodes taking
  their image and their calibration record from the master over 10 Mbit
  Modbus broadcast, switching on board type and position, everything on
  the master, and the bench keeping its debugger. docs/BOOT.md is the
  design, and this is its second commit: `boot/src/boot_core.c`, the
  state machine over four calls - erase, program a word, read, say -
  hardware-free like modbus/ and daq/, and `boot/test/harness.c` wiring
  those to a two-megabyte byte array with 128 K sectors that refuses to
  program a word twice, the way the controller does. Three designers
  were put on it in parallel and hit the session's limit before any
  reported, so the design is one engineer's; the core is tested by 39
  checks through gcc and ctypes: the whole exchange (hold, who, assign,
  erase, 301 chunks with a ragged last one, verify, a 768-byte record,
  seal, go, dump), three chunks dropped and named by the bitmap then
  re-sent with the repeats not programmed again, seal refused in words
  before verify, go refused on the console before seal, the master dying
  mid-stream leaving an invalid image that the next erase forgets, a
  chunk before any erase counted and ignored, a wrong crc reported with
  what was summed, an erase for another type ignored, an image that does
  not fit refused, an image the debugger wrote valid by the four tests
  and three kinds of wrong one not, a first word that will not program
  leaving the image invalid, and CRC-32 equal to zlib's. TWO THINGS THE
  TREE SETTLED ON THE WAY. The user-defined function codes the RTU core
  routes end at 0x6E and every one is taken, so the bootloader is DEVICE
  11 under 0x6E, its table in PROTOCOL.md held to `boot_core.c` by the
  same structure check as every device's, its ops mirrored in
  `protocol.BootOp`, its defines read from `boot/inc/boot.h`; and the
  took byte is the wire's now - `wr_took` in wire.c, 49 sites renamed
  from `cmd_took`, so a bootloader that links no command file refuses
  in the board's words. The wire also gained `rd_bytes`, the raw payload
  after an op's fields, which the request check already knew to read as
  the rest. What is NOT built yet: the registers (`boot_main.c`, commit
  4), the application relocated (commit 3), the master in Python
  (commit 5); nothing has run on a board. structure 674 -> 676, the
  boot core 39, tree 3136, firmware 0 warnings.
* THE APPLICATION RELOCATED BEHIND THE BOOTLOADER (2026-09-22). The
  bootloader's third commit (docs/BOOT.md): the linker script's FLASH
  origin moves to 0x08020000 and its length to 1792 K, a `.app_header`
  section at +0x400 holds the magic, the size, the version and the type,
  and the size is `_app_size`, a linker symbol taken by address - on the
  first build the header read 202 096 and `objcopy -O binary` wrote
  202 096, so the header is right by construction and no stamping step
  exists to go stale. TWO DECISIONS TAKEN AGAINST THE DESIGN AS WRITTEN.
  The identity - unit, position, flags - does not go into the record:
  that would have been CAL_VERSION 16, and the bench board's version-13
  record rides the two-versions-back upgrade rule, which a new version
  would have pushed it off; and an identity in flash outlives the boot
  the master assigned it in. It goes through THE HANDOVER SLOT instead:
  the top 32 bytes of DTCM, `boot_hand_t` in boot.h, placed by
  `.boot_hand (NOLOAD)` at `_estack` with the stack's top lowered by
  those 32 bytes, outside `.bss` so no startup zeroes it - the
  bootloader writes the assignment under a magic before it jumps, and
  `Board_BootInit` applies it after the record loads
  (`modbus_map_set_unit_id`, which nothing had called, and the UART5
  termination on PE14); a bench board flashed over SWD finds no magic
  and is unit 1 as before. And the way back is device 11's own `stay`,
  not a new link op: `cmd_boot.c` serves `state` and `stay` from the
  application and refuses the other eleven in words, so the structure
  check now reads BOTH servers of an op - `_c_ops` keeps a list per op
  where it kept one handler, and cmd_boot.c's had been silently
  shadowed by boot_core.c's until it did. The reset waits 50 ms in
  `Board_BootPoll` for the took byte to leave the wire, then writes
  STAY into the slot and resets; the bootloader will find it there. THE
  VECTOR TABLE IS SET BY THE IMAGE ITSELF: the generated `SystemInit`
  leaves VTOR alone unless `USER_VECT_TAB_ADDRESS` is defined, and that
  define lives in a CubeMX file compiled inside the drivers' object
  library, so the startup writes `SCB->VTOR = g_pfnVectors` as its first
  act after the stack pointer - three instructions, and an image that
  is whole wherever the core arrived from, the bootloader's jump or the
  debugger's `--start`. Verified off the ELF: vectors at 0x08020000,
  the header at 0x08020400 reading `CXAP`, 1.6.0, type 1; SP
  0x2001FFE0 under the slot at 0x2001FFE0..0x2001FFFF; the reset
  vector thumb inside the image. Both presets 0 warnings, Debug 202 K
  of the 1792 K, Release 135 K. `build_and_flash.py` sizes against the
  application's region now and says after a flash that a reset reaches
  the image through the bootloader's sector: a board with the OLD
  layout at 0x08000000 and this image at 0x08020000 has half an old
  image at its reset vector, so the bench's first act is commit 4's
  bootloader over SWD. MINOR 18: device 11 in the application's
  dispatch. Not run on a board; the registers are the next commit.
* A `UL` LITERAL IS 64 BITS ON THE RUNNER (2026-09-22). The boot core's
  commit went red on CI while it was green here: `boot/ builds
  warning-free with the firmware flags` failed on both Pythons with
  `conversion from 'long unsigned int' to 'uint32_t' may change value`
  at harness.c's sector arithmetic. `0x08000000UL` is `unsigned long`,
  32 bits under MinGW and on the target, 64 on Ubuntu's LP64 gcc - so
  the same `-Wconversion` that is silent on both of this bench's
  compilers speaks on the runner's. Thirteen literals across boot.h,
  boot_core.c and harness.c are `U` now: every one fits an unsigned int
  on every host the tree builds on. The runner had told the whole
  story in the commit comment its failure step posts, read off the
  public API with no token - the way a red run is meant to be read
  here.
* NOTHING ON A NODE IS VERSIONED (2026-09-22). The bench put the
  bootloader's point in one breath: the exercise exists to be rid of
  versioning - of the Modbus protocol, the code, the data structures
  and the binaries on the nodes. A node gets all but a sliver of its
  firmware over Modbus; the sliver is a Modbus shell, the boot code and
  every pin in a known state so nothing floats; what the master offers
  is a binary payload straight into memory, and what non-volatile
  memory already holds is overwritten where the checksums do not match.
  The core as written erased on every `erase` and programmed on every
  `seal`, so a boot where nothing changed cost two sector erases and
  every word of the image and the record - the opposite of the
  principle. NOW: `erase` compares first - a valid image in flash whose
  header size and CRC-32 are the ones offered is kept, the bitmap is
  set whole, the state goes to VERIFIED and the console says `kept`;
  the stream that follows for the other nodes lands as repeats and
  programs nothing; `seal` compares the record word for word with its
  sector and programs nothing where they agree, and programs the first
  word only where it was held back. `test_the_same_image_offered_again`
  measures it on the RAM flash: the same image and record offered to a
  node holding them - zero erases, zero words; a different record with
  the same image - one erase, seven words, the record's sector alone; a
  different image of the same size - erased and streamed as ever. The
  harness gained a power cycle (`boot_h_reboot`: the state gone, the
  flash kept) to say it, since a node that just sealed still knows what
  it holds and the question is about one that does not. The header's
  version field stays because a build has one; BOOT.md now says in so
  many words that nothing decides on it, and that the bootloader's
  first act after the clocks is every pin the type's table names driven
  to its safe level - the six gate inputs low as outputs (TIM1's idle
  state is RESET on all six, board_pwm.c), AFE_ON low, PA10 low, the
  termination open, both driver-enables low - which is commit 4's
  `boot_main.c` to write. Boot core 39 -> 45, tree 3142.
* THE BOOTLOADER TARGET, BUILT AND SIZED, NOT RUN (2026-09-23). Commit 4
  of docs/BOOT.md: `boot/src/boot_main.c` is the whole hardware layer at
  register level with CMSIS for the names and no HAL - VOS1 then PLL1
  (HSE 25 MHz / 5 x 64 / 2) for 160 MHz, HCLK and APB1 at 80 so the
  RS485 pair divides to exactly 10 Mbit with 8x oversampling (BRR 16, no
  fraction; APB at 80 MHz is why VOS1 - VOS3 tops APB at 50); the pin
  table driven before anything else is clocked, six gate inputs, PA10,
  PB2 and PE14 low as outputs; three USARTs polled with their FIFOs on
  and no interrupt anywhere; the flash controller's erase and 256-bit
  program with every error flag checked and cleared through CCR and a
  word refused unless it reads erased, as the harness refuses it; the
  console a 256-byte ring drained a byte a pass so a line never blocks
  the wire; the jump filling the handover slot, resetting the USARTs,
  handing the clock tree back to HSI with PLL1 and HSE off and the PLL
  registers, ACR and VOS3 at their reset values - the application's
  HAL refuses to configure a PLL that is the system clock - then `msr
  msp` and `bx` in one asm statement so no frame is read after the
  stack moved. Its own linker script loads .text and .rodata to ITCM
  and .data to DTCM, the startup copies them, the vector table and the
  copy loops alone run from flash; `.ARM.exidx` is discarded, and the
  slot is the same 32 bytes at the top of DTCM the application's
  script gives it. `boot/CMakeLists.txt` is its own directory because
  the toolchain file puts the application's linker script, map and
  `-u _printf_float` into the tree's link flags and a directory scope
  is where they are swapped - the first link pulled newlib's syscalls
  through printf's float support and failed on `_kill`. The slave
  core gained `MB_NO_REPLY` so a user function can answer silence: a
  bus of blank nodes shares unit 247 and only the one the unique id
  names answers (modbus core 77 -> 78). Measured: 14 268 bytes of
  flash in Debug, 7 572 in Release, 6 732 of DTCM; both presets 0
  warnings, with -Wconversion on every file but boot_main.c, whose
  CMSIS cache helpers trip -Wsign-conversion in Release. build_and_
  flash.py sizes both images on their own regions and `--boot`
  flashes the bootloader first; CI sizes and keeps both. An
  adversarial review by six agents was attempted and every one died on
  the session's limit after 0.8 M tokens, so the code stands on one
  engineer's reading: the first flash over SWD is the review. Pulled
  25 render commits first; structure 676 -> 683 upstream, tree 3210.

## The local model

* Asked for raw codes with the AFE deliberately off, a model wrote
  "Mid-scale ... 25.00 C" out of the warning text itself.
* A local model reported a coaxial cable or connector, twice.
* `ch=['phA']` was guessed; BUS_VOLT and A0 were invented; the left
  knee, asked for in Swedish, was sent as `right knee`.
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

* **The daemon had no runner, and the page said "(500)"** (2026-09-12,
  the bench seeing an error loading the models). Every chat and
  generate on
  this laptop had answered 500 since 2026-09-03, and the body says why:
  `error starting llama-server: llama-server binary not found (checked:
  ...\lib\ollama\llama-server.exe, ...)`. `lib\ollama` holds one CUDA
  DLL (`cuda_v13\cublasLt64_13.dll`) and nothing else; `upgrade.log`
  ends 2026-09-03 20:30:54 with the OLD install's uninstaller deleting
  every `ggml-*.dll` and the runner - "Uninstallation process
  succeeded", "Log closed" - and no install after it, while `ollama.exe`
  and `ollama app.exe` are the 0.33.3 binaries dated 16:50 the same
  day. The tags are there (gemma4:12b 7.6 GB, llama3.1:8b 4.9 GB) and
  `/api/tags` answers, so the page's model choice, warm and
  `Clear-Resident` all passed and the preload's catch printed
  `$_.Exception.Message`: the remote server returned an error, (500)
  internal server error - the status line in the console's own
  language, the body never read; the Python client one layer down had
  the words all
  along (`/api/chat 500: {"error":"error starting llama-server: ...`).
  `Get-DaemonWords` reads the body now - `$_.ErrorDetails.Message`
  under PowerShell 5.1, else the response stream seeked back to 0
  (Position 625 of 625 after Invoke-RestMethod has been through it,
  measured; without the seek the body read empty and the first version
  fell through to the status line) - and a body that says
  `llama-server binary not found` is a stop with the fix (`.\setup.ps1`,
  or the ollama.com installer) rather than a warn and a prompt where
  every question fails; any other body is shown in the daemon's words.
  No pull fixes a missing runner, and the reinstall is the operator's.
  ollama.com answers from here (200), so nothing stops it. Reinstalled
  the same evening (0.34.0, `lib\ollama` whole again): `/api/generate`
  loads llama3.1:8b, and the page's one-shot against the stand-in -
  what electronics is this, asked in Swedish - loaded the model in
  2.9 s and
  answered off the parts list in 22 s end to end, 2 tool calls, 844
  tokens, exit 0.
* **One pull, drawn from the daemon's numbers** (2026-09-12, the bench
  asking that the LLM page pull a missing model itself and show a
  progress bar). The page shelled out to `ollama
  pull`, whose bar is a TTY repaint, and dbg.py refused an absent tag
  with the command to type while MODELS.md said both pulled.
  `coaxial_ollama/pull.py` streams `/api/pull` - `status`, and
  `digest`, `total`, `completed` while a layer comes - and draws it in
  Say's columns: 24 braille cells at half-cell resolution (⣿, ⡇ for
  the half step, ⣀ for what is to come), the percent, bytes of bytes,
  the layer's rate and what is left at it; rewritten in place on a TTY,
  a row at every status and every five percent off one, ASCII when the
  stream's codec cannot hold braille. An `error` line from the daemon
  is raised as OllamaError in its words ("pull model manifest: file
  does not exist" in the suite), a cloud tag refused before any
  request. `board_chat.ps1` runs `python -m coaxial_ollama.pull $Model`
  when the tag is not in the list; dbg.py's start goes through
  `cli.ensure_pulled`; `/model TAG` mid-session still refuses, so a
  typo costs a command and not gigabytes. Measured off a TTY,
  all-minilm:22m through this laptop's daemon: 28 rows, the 46 MB
  layer at 9 MB/s with "11 s left" at 9 % and "0 s left" from 91 %,
  three small layers after it, `pulled all-minilm:22m, 46 MB in 10 s`,
  exit 0, the tag removed again; the scripted 4.9 GB stream in the
  suite is 26 rows off a TTY and one row rewritten 106 times on one,
  closed once. link 109, structure 621, 2978 in all.
* **The chooser's chat page pulls too, on its boot strip** (2026-09-22,
  the bench: an error when the model is not there - make the terminal
  download the suitable model). `coaxial_tty.ps1`'s CCC page ran the
  picker, which named llama3.1:8b on an 11 GB card with 1.4 GB already
  taken and 3.4 GB held back - 7.6 GB to spend, so the pulled
  gemma4:12b at 7.8 GB did not fit - and died in a traceback from
  `require_model` with `ollama pull llama3.1:8b` as its last line,
  while both prompts had pulled through `ensure_pulled` since
  2026-09-12. The page goes through `ensure_pulled` now, and the pull's
  rows go to the page's amber boot strip rather than to stderr under
  it: `pull()` takes a `rows` drawer, and the page's `Strip` sets the
  strip's bar to the layer's share and its bracketed text to the tag
  and the figures (`Progress.figures()`, the row minus its bar), cut at
  a gap between figures for the console's width - a strip that wraps
  is two strips. Two bars on one row was not tried, by reasoning: the
  strip's rich Progress repaints its row on its own clock and the
  pull's carriage-return rewrite would land inside it. Off a terminal
  there is no strip and the pull draws its own rows on stderr; a daemon
  that refuses ends the page with `ollama: <its words>` and exit 2, as
  dbg.py's start does, so the chooser's "exited 2 - its last lines
  above say why" points at one line and not forty of traceback.
  Measured on a recording console with the suite's scripted 4.9 GB
  stream paced at 15 ms an event: 17 frames at 100 columns, the longest
  95 cells, `PULLING llama3.1:8b   53 %  2.6 GB of 4.9 GB  49 MB/s
  47 s left` halfway; at 80 columns the same 17 frames, the longest 75,
  the rate and the estimate cut off at the gap; no frame on two rows at
  either width; the two-frame smoke exit 0. `python -m coaxial_ollama`
  - the plan runner - still refuses an absent tag with the command to
  type: a plan names its model, and a typo there is a command, not
  gigabytes. link 114, 3147 in all.

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
  247/247. The bench's report was that app_robot_arm seemed to have an
  error in its code.

* **`app_precision_servo`'s trace looked hard-filtered and subsampled**
  (2026-09-07): the shaft was read every 20 ms, and a held rotor rings at
  some 30 Hz - the plot was the ring aliased into a slow sawtooth drawn
  with markers. Read every 2 ms on the stand-in (427 Hz through each
  phase) and drawn as a thin line, the trace is the physics: 0.18 deg
  peak to peak held, 3.34 deg the moment the 0.03 N.m load steps on -
  the pendulum released, the sag 1.58 deg - and 0.62 deg after the
  correction, which re-centres the spring and leaves the ring to the
  damping. On a link the A1335 answers every 15 ms or so; a trace this
  fine is the model's, and the cell says so.

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

* **BOARD ATTITUDE's board sits one row lower** (2026-09-07, the bench:
  it sits a little high): `orientation.LIFT` 0.39 to 0.42 - a row is
  0.028 of
  the view's 36, and 0.36 was the fit, 0.39 the row asked for on
  2026-08-30. Rasterised before and after at 110 by 36: the board's top
  moved from the Z label's row to one below it, and its feet still clear
  the frame's bottom. `test_render`'s shipped-board check reads the lift
  off the constant instead of carrying its own 0.39.

* **The notebooks show the pages' pictures** (2026-09-07, the bench
  asking for the coloured braille frames as PNG, through a handy helper
  in the Python library): `coaxial.ansi.image` parses a frame's escapes
  and
  draws it cell by cell in the bench's faces - Consolas, Segoe UI Symbol
  for the braille - as a Pillow image, which Jupyter shows inline as the
  value of a cell; `ansi.png` writes one. `tools/ansi2png.py`, the
  routine's judging tool, is now the command line over the same code -
  it carried its own parser and drawing, and the view suite's frame
  check imported the tool for the parser. Pillow arrives with matplotlib,
  so a notebook already has it. Four notebooks carry pictures:
  `thermal_model` its steady state in colour, `thermal_budget` the board
  as the observer sees it, `thermal_identification` the board in each of
  the tour's three rooms at the last loaded minute of the leg, and
  `imu_session` the board at the quaternion the IMU reported, in the
  attitude page's wireframe, `rotor_observer_session` the machine's
  cross-section at the model's last angle with the teeth driven to the
  phase currents (2026-09-08), `angle_session` the face between its
  scales. Rasterised and read before landing: the fixed scale beside
  each map, the wireframe on its ground, the can and its magnets.

* **SHAFT ANGLE wears two scales** (2026-09-07, the bench finding it a
  little plain and asking for scales beside it: the die temperature and
  the field strength in gauss): the die's temperature to the left of
  the face on the A1335's
  operating range, -40 to 150 C; the field to the right on 0 to 1200
  gauss. **Each tube is three bands** - blue under normal, green through
  it, red past it, the bench's three bands for normal temperature and
  field - blue, green, red: the die's normal 15 to 65 C, where this
  board
  works, and the field's the datasheet's recommended 300 to 1000 gauss.
  The thermal map's ramp was tried on the die first and a
  room-temperature die came out in the ramp's blue, which read as cold;
  the first field rule had red for no magnet and amber past the band,
  and the bench asked for the one scheme on both - each a tube of dots
  filled to the reading, its empty glass the same four dots in ash - it
  was one dotted column, which the bench asked why it always got as a
  single greyed line, and read as a stray line beside the bar - the
  graduations
  numbered outboard and the reading under it level with the face's
  caption, which leaves the gauss to the scale. `dial.scale`,
  `dial.beside` and `dial.instrument` - the face between its scales,
  which `angle_session` shows as a picture - are pure; the page reads
  TSEN and FIELD again every 5 s
  through `configuring()`, which costs the angle a reading or two each
  time. **The face gives way to the scales**: they were gated on a
  terminal 124 columns or wider - the full face or nothing - and the
  bench's is narrower, and it saw SHAFT ANGLE still not updated;
  now the face is drawn as wide as the viewport leaves after the two
  scales, 58 down to 36 (`show_angle.fit`), and the rim it loses is an
  eighth - at 21 rows the face is bounded by its height, 33 dots of rim
  at any width from 42 up, 29 at 36. At 100 columns the face is 38 wide
  with both scales; under 98 it stands alone; `--scales` / `--no-scales`
  force it. The stand-in's die is as warm as
  the thermal stand-in's board node. Rasterised before landing: plain,
  colour at 61 C and 380 G, and the weak case at 12 G with the tube red
  and the face's needle gone.

* **Every markdown file reads clean under the editor's own linter**
  (2026-09-07, "en massa markdown-lintfel"): markdownlint 0.41 over the
  ten files, with the tree's conventions in `.markdownlint.json` - 80
  columns of prose, tables and code exempt, one marker per list level.
  279 findings, 232 of them the new MD060: every table's delimiter row
  was `|---|---|` under rows spaced `| a | b |`, and the rule reads that
  as two styles in one table. Fixed by the linter's own fixer, with the
  blank lines around headings and lists; by hand, the PR-description
  skill's prose rewrapped and the host README's indented code blocks
  fenced like the rest of the tree's. The Python linter tried first
  lacks the table rules, which is why the first pass missed them.

* **The notebooks read clean under Pylance** (2026-09-07,
  the bench seeing a Pylance complaint in foc_montecarlo.ipynb): pyright
  over
  every notebook's code cells, 27 complaints in four of them, none a
  runtime fault and all worth hearing. `foc_montecarlo`: a speed band
  answered None where every `%f` and `rpm()` after it wanted a number -
  nan now, which says "nowhere inside" and prints; a second `band` over
  the first; the injection chooser's None indexed; and pandas typed as
  any scalar or a Series wherever a column list indexed a frame, so the
  grouped mean goes through `agg` and auto_tune's table through
  `to_string`. `shared_session`: the rig's `board` and `origin` were
  attributes born None and set by `open()`, so every `.origin.x` read as
  a possible None - they are properties now, and a rig that is not open
  raises instead of answering None (invariant 8); the attribute fallback
  looks where the board moved to, which a recursion found first.
  `thermal_identification`: the thermal stand-in's `_gate` and
  `_derate_to` hooks born None, typed as the callables the board wires.
  Twenty-two notebooks, zero errors; the four re-executed clean.

* **The coaxial package reads clean under Pylance** (2026-09-07): pyright
  in basic mode over the host tree found 464 complaints, 150 of them in
  the package. Two were dead code - `Daq.drain` and `Daq.once`, on the
  board and the stand-in alike, called a `read()` no acquisition has had
  since it became `acquire`, and nothing called them. The rest were the
  editor being right about what the code could not show: mixins calling
  what the concrete class brings (`CalibrationOps`, `Subsystem._op`,
  `_Mode._start` - declared now), attributes the board wires onto other
  objects after construction (declared where they live), values born
  None and set later (accessors that raise instead - the clock's best
  bracket, the thermal shadow, the stand-in's task and situation, the
  crews' pools), a `tuple[()]` that typed every loop over it as Never
  (`or []` now), and the socket server's attributes. One guard did bite
  while it was being written: the gate stand-in's drive hook was read
  through `getattr` under a name a grep missed, and removing its wiring
  flattened the sample-point scan to one variance at every trigger -
  the sensorless suite said so, and the hook is declared now.

* **The tools, the MCP server and the model runner read clean under
  Pylance too** (2026-09-07): 107 complaints, a quarter of them the
  module docstring being Optional to a type checker at every
  `description=__doc__.splitlines()[0]`. One real fault among them: the
  test runner's timeout path returned five values where its callers
  unpack six, so a suite that timed out would have crashed the runner
  instead of being reported - it returns six now. The rest: subprocess
  output that may be bytes or text made text in one helper (`find_board`,
  which `build_and_flash` imports); POSIX-only `os.sysconf`, `getloadavg`,
  `termios` and `tty` reached through `getattr` and `importlib` so the
  Windows editor stops reading them as missing; `winreg` through a
  checked local; the socket pipes, the sweep's ends, the pose parsers
  and the memory probe guarded where they were Optional; rich's
  renderable protocol met under its own parameter names; the
  attribute-typed dicts and lists declared where their first element
  set the wrong type. `.vscode/settings.json` adds `host/tests` to the
  analysis paths, since three tools import test modules.

* **The whole host tree reads clean under Pylance** (2026-09-08): the
  193 complaints left in the suites were scaffolding, and three of them
  were something else - `test_views` defined
  `test_every_gauge_shows_its_own_scale`,
  `test_the_bead_is_round_at_every_angle` and
  `test_the_terminal_is_asked_how_tall_a_cell_is` twice each, a paste
  that landed twice; the copies were identical and the runner called
  each name once, so no check was lost, and the first copies are gone.
  The scaffolding: the runner's `client`, `session`, `out` and `io_log`
  are declared `Any` where a scripted fake stands in for them, as is
  the transport's `serial`; `keep_alive` and `tag_ok` take what the
  tests and the bench pass them; a tool's `Reported` result is read as
  text where a test reads it as text; the Modbus suite names an
  exception code through one helper instead of `EX.get(code, code)` with
  a code that may be None; a slot's deliberate typo goes through
  `setattr`; the injection chooser's None, the harness's struct, Popen's
  pipes and the conformance port are narrowed once. pyright: 0 errors in
  the tree; the offline gate 2582 passed, 0 failed.

* **SWITCH SOA read the board's worst node, not the switches** (2026-09-08,
  the bench seeing MOTOR SOA and SWITCH SOA at exactly the same value):
  the ROTOR OBSERVER's first gutter tube took
  `headroom`, the worst of all ten nodes, under the switches' name. On
  the stand-in's demo at frame 200 it read 67.7 % from the copper patch
  under leg U at 78 % of its ceiling while the six switch nodes sat at
  37; on a long run, once the winding is the worst node, it read the
  winding - the same number MOTOR SOA reads, which is what the bench saw.
  `switch_headroom` takes the worst of SOA_NODES, the six a duty cycle
  drives, each against its own ceiling; the board's worst stays the SOA
  HEADROOM gauge's, so the page now shows three figures that are each
  what their name says. A board that reports no per-node spend falls
  back to its worst.

* **STBL over a number the trip was holding down** (2026-09-08, the
  bench seeing STBL at 70 % of SOA, where the hysteresis and the limits
  should have shown UNCR): the foot's word was the
  identification's state and its percent the margin in force, which op
  10 reports as the least of the identification's own and the trip cap -
  and after a trip the cap is the one in hand, 0.70 back a percent a
  minute, while the model can be STABLE at 1.00 underneath. On the
  stand-in through a trip: minute 10 `STABLE 0.80`, minute 20 `STABLE
  0.90`, the identification alone at 1.00 by then. The wire carried
  nothing to tell the two apart, so MINOR 17 appends the trip cap to op
  10 (firmware, the parser, the stand-in); the foot says `TRIP 72%` in
  the trip's red while the cap holds the margin and the state's word
  once the cap has recovered past the model, the THERMAL OBSERVER's
  margin row says `the trip cap`, and the MCP tool's text says
  `recovering`. A board before MINOR 17 answers no cap and reads as
  before. Built with zero warnings; not run on the board.

* **A re-trim tripped the stage, and the foot read TRIP for good**
  (2026-09-08, the bench seeing it stuck at TRIP with a percent, then
  asking for a look at how TRIP is handled against UNCR, CONV and
  STABLE with the observer's innovation - something odd there). Traced
  on the rotor
  page's own demo loop, headless, four reads a second: the page lays
  the room tour on the stand-in, and the tour moves the room from 20 C
  to -25 C a hundred model seconds after STABLE - at model second 555,
  with the fourth send at the clamp under way. The innovation rose from
  0.05 to 0.28 on the one sample that straddled the step (the two die
  thermometers 1.6 and 2.1 K under the shadow, the NTC 0.2), the margin
  fell from 1.00 to 0.82 on it while the state was still STABLE, the
  ceilings were re-trimmed by it, and driver U at 92 % of the old span
  stood at 112 % of the new: `tripped`, MOE dropped, trip 1 at model
  second 573 - for a policy step, not for heat. The trip cap then held
  the margin at 0.70, the demo's operator re-armed as soon as no node
  was over the trimmed ceiling, and trips 2, 3 and 4 came inside six
  wall seconds at wall 285-291 with the cap back at 0.70 each time; the
  next sample threw the state to UNCERTAIN (the room reset, right for a
  room step) and the observer's NTC ran 6 K under the truth while the
  room was re-identified from 19 C toward -3. The foot read `TRIP 70%`
  rising a percent every six wall seconds and never anything else.
  Three things, one commit. THE TRIP IS JUDGED ON THE RECORD'S CEILING:
  `thermal_soa_t` carries `trip_c`, the untrimmed ceilings, `soa_from_cal`
  fills it and `thermal_budget` trips on it; `limit_c` stays the
  throttle's, so a ceiling pulled in under a node reads 255 with the
  clamp closed and cools, and MOE is dropped only at the limit the
  record gave (invariant 10's words made exact). The stand-in's
  `_tripped` mirrors it. THE FOOT'S WORD: TRIP only while the cap holds
  the margin UNDER THE FLOOR - the bench's point that it must let go of
  TRIP once over 80 %, its line being `WINDING 97.7 C TH OBS TRIP 89%`;
  above
  it the number is one a state could give and the word is the state's,
  the cap's percent still the one in force. Measured after: the same
  loop, five wall minutes, no trip on the room step, the clamp at 0 %
  through the re-trim and back, the foot `STBL` and `CONV` with the
  margin, TRIP only under a real ceiling. Firmware 0 warnings.

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

* **The frames' corners had a foot** (2026-09-12, the bench asking for
  the regions' corners on the thermal observer fixed, showing ⠼). A
  frame on the map is drawn
  on the cell grid: the top and bottom lines across a dot row of the
  corner cells, the sides down the lane the edge's millimetres fell in
  - and the lines spanned BOTH lanes of the corner cell whatever lane
  the side was in, so a left side in the cell's right lane had its
  lines run one dot past it: `⠼` at a bottom-left corner where `⠸` was
  meant, `⢲` at a top-left where `⢰`, and the mirror on the right.
  Measured against a literal table of the eight right-angle glyphs, one
  per corner and lane, every mark at 40, 48, 60, 72 and 88 cells with
  the corners the rim runs through left out: 63 of 142 wrong - MCU, REG
  and HS on the left at 48 and 60, REG and U on both sides at 72 and
  88, between four and seven frames at every size. The lines now start
  at the side's lane (`_draw_frame`): 0 of 142, rastered at 48 and 60
  and read. The same table is the views suite's check, with the eight
  glyphs also drawn off a blank field for both lane combinations.
  views 206, 2981 in all.

* **BOARD ATTITUDE's frame rate on the threadripper, and the Marquee
  decoding the art itself** (2026-09-22/23, the bench: why is the
  attitude page's frame rate so low on this machine, and what would raise
  it). MEASURED FIRST, `--simulated` at 108x40 with the stand-in turning
  so every frame is a moving frame: compose 74 ms with the crew of 8, 129
  without, 76 with sixteen workers (the parent's serial work is the limit,
  not the cores), 106 at 150x44; 29 ms outside compose; in a real conhost
  window (152x48, VT on, truecolor) 73 + 32, so the console adds ~3 ms and
  the terminal is not it - Windows Terminal is not installed and conhost
  runs the chooser. A resting board under the deadband is 3-4 ms. So a
  moving frame was ~105 ms, ~10 Hz, against the laptop's 50 (2026-09-05):
  Zen 1's single thread. cProfile of the parent over 60 frames: rich's
  Live.update 94 of 164 profiled ms - Text.from_ansi over the forty art
  lines 48, Layout and render 35 - the renderer 49 (the crew wait 34,
  _paint 21.5 of which _outline 15). A workflow of three lenses then
  measured the levers, one at a time, each prototype compared byte for
  byte with the tree's output: (1) rich: Text.from_ansi decoding ~700
  24-bit runs a frame and Text.render re-sorting the spans in the Panel,
  Align, Layout and LiveRender passes, 16 of the 30 ms after the renderer;
  a Marquee splitting the SGR codes itself into Segments cut
  frame_of+Live.update 29.9 -> 13.6 ms, identical; (2) the renderer's
  parent passes: 9 of 19.4 ms removable identically - the outline's
  vertices projected once and its trace overlapped with the crew through
  map_async and a GIL yield, the shadow map built in the workers, the
  reach cached per solid, the encode cached, _glow's helpers inlined;
  (3) the crew: the slowest of eight equal bands is 34-37 ms and its
  floor is the setup over all 5570 triangles repeated in every band (12.8
  of an empty band's 15.3 ms; the vertex pass is 2.0, so crew.py's "what
  every band repeats is the vertex pass" was wrong), not the IPC (0.9 ms
  round trip, 15.6 kB results); the lever is ONE POSE AHEAD - the newest
  pose's bands submitted before the previous pose is painted, with the
  sends off the Pool's GIL-starved handler thread (per-worker pipes, or
  the switch interval at 0.5 ms) - the view's loop 76 -> 46 ms a frame
  (13 -> 22 Hz), the picture identical one frame later; the y-reject moved
  to the top of the triangle loop 36 -> 32 ms wall, identical; bands
  balanced on the previous frame's coverage 36 -> 22, NOT identical
  because engine.shade seeds a bare cell's grain on the strip row - so
  today's 8-band crew already draws 528 bare cells a frame with a
  different grain than the no-crew path or a four-worker laptop, a latent
  difference to settle before the bands move. Two refuters and the
  synthesis died on the session's midnight limit; the first refuter held
  the Marquee and caught the first landing of it. LANDED FIRST, this
  bullet: `stage.Marquee` decodes the art's SGR itself - one re.split a
  line, the effect of each parameter string decoded once (KEEP, the
  default or a Color per channel; 275 distinct strings in a frame), a
  Style per (fg, bg) - and yields ready Segments; a line carrying any
  other escape takes Text.from_ansi as before. The refuter's catch:
  parsing ints and a Color per run cost 7.66 ms over the forty lines
  against the prototype's 1.71; cached by the parameter string it is
  1.05. Measured in the view's own loop, 120 frames at --hz 30: 108x40
  92.3 -> 69.8 ms a frame (10.8 -> 14.3 Hz); 150x44, the chooser's own
  framing in a 152x48 window, 134.0 -> 93.6 (7.5 -> 10.7 Hz). Console
  output byte-identical on 47 captured frames, and the suite holds every
  colour form the tree's art carries to the Text path's bytes, the
  fallback and a crop (test_views 211, 3151 in all). NEXT, in order: the
  one-pose-ahead crew with the settle memo (a held pose is re-rastered
  FACE_SETTLE times today, ~347 ms every time the board rests) and the
  y-reject; then the parent's passes.
* **The crew one pose ahead, on its own pipes** (2026-09-23, the second
  step of the above). `Crew` is eight worker processes on their own
  `multiprocessing.Pipe`s with the sends on the calling thread, in place
  of a `Pool`: the Pool's handler thread needs the GIL to pickle and send
  each job and a parent busy painting hands it over every 5 ms, so the
  eight jobs left the parent over ~40 ms and nothing could overlap the
  wait (measured by the crew lens, and again with
  `sys.setswitchinterval(0.0005)` as the other cure - the pipes need no
  process-wide setting). `submit` and `collect` are a frame's two halves;
  `frame` and `raster` are the two together, so every other caller is as
  it was. `wireframe._face_ahead`, chosen by `render(ahead=True)` with a
  crew and `persist` (the attitude page passes it): the pose asked for
  goes to the crew and the pose whose bands are ready - the previous one
  - is painted meanwhile, with its own rotation for the triad; the pose
  the board rests on is collected and painted at once; a flight made
  stale by a framing change is dropped; with nothing in flight the held
  picture stands one more frame, which is where the one-frame lag begins,
  and with no picture to stand (the first frame, a resize) the crew is
  waited for as before. The settle memo rode along: `_face_layer` keeps
  the cells beside the layer and repaints from them while the pose
  settles, one raster per pose instead of FACE_SETTLE + 1 (held in
  test_render: "from ONE raster"). Identity held in test_render on the
  same crew, a rest then four poses then a rest: every ahead picture is
  the sync path's a frame later, the lag's first frame is the held
  picture, and a rest drains the flight; the steady vote makes this
  exact because at rest its two held frames are already equal. Measured
  in the view's own loop, 120 frames at --hz 30, crew of 8: 108x40
  compose 56.2 -> 28.4 ms (the crew wait gone from it), 69.8 -> 42.7 ms
  a frame, 14.3 -> 23.4 Hz - past the 20 Hz default for the first time
  on this machine, 92.3 ms and 10.8 Hz two steps ago; 150x44 (the
  chooser's own framing) 93.6 -> 59.4 ms, 10.7 -> 16.8 Hz, from 134 and
  7.5. The picture is one frame late: 43 ms at that rate, under the
  IMU's report interval and the steady vote's own frame. crew.py's
  "what every band repeats is the vertex pass" was wrong and says so
  now: the crew lens measured an empty band at 15.3 ms of which 12.8 is
  the setup of every triangle before its row reject and 2.0 the vertex
  pass - that setup is the floor, which is why sixteen workers measured
  no faster than eight. NEXT: the y-reject hoisted to the top of the
  triangle loop (36 -> 32 ms wall, identical, measured by the lens), the
  parent's passes (9 of 19 ms identically), and the grain seed on the
  frame row before any band balancing. test_render 86, 3157 in all.
* **The front page's readout: what the board says it is, printed in a
  late-seventies console's register** (2026-09-23, the bench, in
  Swedish: split the COAXIAL 63100 box in two, the model turning above
  and, in the lower box, "a prompt that prints data about the board in
  a cool retro-futurist way, like the USS Nostromo or Blade Runner",
  going on with more about the board and not forgetting the development
  with an LLM / Claude, the register retro-futurist throughout; old
  text erased or overwritten, "you decide, just make it maximally
  cool" - and then, seeing it: strike the SHIP references and the like,
  so it does not get silly. So the register stays - terse, upper case,
  a prompt and its answer, dotted leaders - and the film's own lines
  went: the box is `READOUT`, it opens READOUT ONLINE. READY FOR
  INQUIRY and ends END OF INQUIRY. STANDING BY, and the bench's local
  model is AT THE BENCH). The right column is two boxes now: the
  turntable above, the readout below - `tools/readout.py`. THE FACTS
  ARE THE BUS'S: the identity
  page is 0x41 (unit, type, firmware and protocol, MCU, commands, the
  link, and the board's own description line under `> DESCRIBE UNIT`)
  and the fitment page is the parts list, 0x6D kind 4, name and role
  per row; the stand-in answers both when no board does, and its
  description says SIMULATED in the line itself. Read once through a
  short session in the page's link thread when the link is known (the
  stand-in's at once under --simulated, off the frame loop) and closed
  again so a view can have the port; a refusal leaves AWAITING LINK
  with the refusal's words under it. Nothing in the file names a part
  (held: an identity with an empty parts list prints NO FITMENT
  REPORTED and no name). The third page is the host's own account, in
  the same register: ENGINEERED in dialogue with Claude / Anthropic,
  AT THE BENCH a local LLM with the board tools over MCP, the CONSOLE
  pages, the CORES, VERIFICATION as "N of 28 suites run here, M
  checks" read off tests/.counts.json - measured on this terminal, not
  quoted - or "unmeasured on this terminal", RECORD findings: every
  measurement kept. THE MOTION: page by page - a page types in at 90
  characters a second, holds four seconds, decays from the top a row
  every 70 ms with the going row dimmed to the frame's teal, and the
  next inquiry ticks in; identity, fitment, provenance, round again; a
  block cursor blinking at 2 Hz through typing and hold; a status row
  `INQUIRY 2/3  FITMENT` on top; a page longer than the box is as many
  inquiries as it takes, so nothing scrolls mid-line. The box takes a
  third of the body's rows, a status row and five lines at least,
  twelve lines at most, and the model keeps the rest; the leader column
  is each page's own - its longest label, a space and at least one
  dot, never past half the box. A fixed column of eighteen was tried
  first: UART5 TERMINATION filled it and a forced dot put its value a
  column past the others (the bench saw the space), and with no dot it
  lined up but had no leader (the bench asked for the dot) - both
  gone with the column following the page. A label past the column
  keeps its row with the value under it, so the 26-column floor still
  fits every line. The turntable rides
  the Segment Marquee too. Judged in the raster at 100x30 and 152x48
  (tools/ansi2png.py): the model above, the readout below in teal
  voice, dotted leaders and sodium values, the cursor on the typed
  end. Held in test_views: three pages within the box, the identity
  page's 0x41 fields, the fitment page's parts and the empty case, the
  provenance page's words, the 22-column fit, the motion on a scripted
  clock (type, hold, decay, the next inquiry, round again), a
  mid-typing frame ending on the cursor; the two-frame smoke draws it
  with the stand-in's identity. test_views 218, 3164 in all.
* **BOARD ATTITUDE's board sits at the frame's middle** (2026-09-23,
  the bench: move the object down so it lies centred in the box).
  `orientation.LIFT` 0.42 -> 0.5, the bounding sphere's centre on the
  frame's middle row. MEASURED before choosing, the board's drawn rows
  at 108x80 (tall enough that nothing clips) and zoom 1.27, three
  poses: flat, its centre sat 1.5 rows above the middle at 0.42 and
  4.5 below at 0.5; tilted 25/15 degrees, 4.5 above and 2 below; tilted
  40/30, 8 above and 2 above - the camera's 34-degree tip projects a
  flat disc's centre under the sphere's and a tilted one's over it, so
  no constant centres every pose, and a fit per frame would read as
  translating (2026-08-30). 0.5 puts the tilted poses, which are what a
  real IMU shows, within two rows of the middle; 0.46 would split the
  difference with the flat pose (1.7 low, 1.3 high, 4.8 high) if the
  bench comes to prefer it. At the chooser's own 108x44 the flat board
  overflows the frame at this zoom whatever the lift - rows 0..43 at
  0.42, 4..43 at 0.5, so the feet lose four rows and the top gains
  them - and the tilted poses fill it either way; the change shows on
  a taller frame and on a tilted board. test_render's shipped-board
  check reads LIFT off the constant, so nothing there moved.
* **The outline draws the slab's other face too** (2026-09-23, the
  bench: something odd with the edge enhancer, clearest at the hole in
  the middle - mainly as the board turns from its back to the component
  side, the enhancer and the hole not in step in position or in time,
  as if one lagged the other a frame). Not a lag: `_outline_loops` took
  the slab's TOP loops only - the rim and the bore at the measured top
  level, the parts standing over it - and the depth test hid them
  whenever the board showed its back. Measured on the mesh: the top at
  z -0.0686, the bottom at -0.1006 with 2 500 vertices, the bore's ring
  a 16-edge loop at the top only. So from behind the hole had no ring
  at all (rendered and looked at: back flat, back oblique 150/10, both
  bare), and just before edge-on the ring drawn was the far face's,
  the slab's 0.032 units - two cells at the view's zoom - away from
  the hole it framed, the offset changing sign through the turn: the
  "lag". `_slab_bottom` measures the bottom face (the most populated
  level a millimetre or more under the top with SLAB_BOTTOM_SHARE of
  its population; None for a one-faced slab, which the suite's
  synthetic one is), and the bottom's rim and bore and the parts
  hanging under it join the loops; `_outline`'s depth test, whose
  OUTLINE_GRACE 0.012 is under the thickness, shows whichever face the
  camera sees, both at edge-on. 875 -> 925 loops; `_outline` 5.34 ->
  6.90 ms a moving frame at 66x40 on the threadripper. Rendered after:
  the ring hugs the hole from behind and obliquely from behind, stands
  on edge at 100 degrees, and the front is as it was. Held in
  test_render: the one-faced slab has no bottom, and the same slab
  given a bottom face draws both rims with the top still the top.
  test_render 88, 3169 in all.
* **The outline through the face's own vertices, from the face toward
  the camera** (2026-09-23, the bench: still an offset between the edge
  enhancer and the rest of the renderer - in general, not only the
  centre hole, but clearest there). TWO CAUSES, both measured. (1) Two
  geometries: the outline traced the exact mesh (OUTLINE_EXACT) while
  the face rastered the decimate, whose vertices are the means of their
  grid cells - so a bore's circle framed a polygon inside it by the
  chord's sagitta, and every part's edge sat where its corners' cells
  happened to fall. Measured at zoom 3 on grid 64, the ring's nearest
  dot against the hole's raster edge: +3.1 fine px outside, flat. Now
  `_snapped` moves the exact mesh's points to the decimate's vertices
  (a cell's mean lies in its cell, so the decimate's own vertices name
  their cells and nothing has to come out of the clustering;
  `mesh.cell_key` is the one rule for both) and the line runs through
  the face's polygon corners: +0.15 fine px at that zoom. At the
  view's own zoom the misfit was already under a dot either way (|mean|
  1.0 -> 1.1 fine px flat, 0.63 -> 0.66 tilted), so the hole was not
  where the bench saw it. (2) Both faces drawn: with the bottom's loops
  added the day before, the FAR face's rim showed THROUGH the hole -
  the bottom's from the front, the top's from behind - the slab's
  thickness away from the near one, a ghost arc inside the ring
  (rendered front 25/15 and seen). Physically visible, read as the
  outline out of place. So each slab loop carries its face, and
  `_outline` draws the slab's edge and bore from the face toward the
  camera (the body's z in view depth, m8), both within SLAB_EDGE_ON 0.1
  of edge-on; parts draw from either side under the depth test as
  before. Rendered after at the view's framing: front flat, front
  25/15 and 50/20, back oblique 150/10 - one ring each, on the hole;
  edge-on the rims on edge. `_outline` 5.9 ms a moving frame at 66x40
  steady (6.9 with both faces drawn), the first frame 1.4 s for the
  exact index and the snap, which the attitude page pays behind its
  boot strip. Held in test_render: the slab's loops know their face and
  a part's knows none; from above the bottom's rim adds no cell, from
  below the top's adds none, edge-on the second adds some.
  test_render 90, 3171 in all.
* **The ghost hole was the decimation's, and the crew's floor is grid
  48** (2026-09-23, the bench, with a screenshot of the stand-in seen
  steeply from the side: the edge enhancer shifted down-left, the hole
  as a ghost up-right - "some position or time sync error between the
  edge enhancer and the model"). The view's own chain was rebuilt
  frame by frame off the screen - the stand-in's IMU, latest(), the
  deadband, attitude() with the tare, orientation.render with the crew
  one pose ahead - and the picture reproduced; then run again without
  the pose ahead, without the crew and without the steady vote: the
  same picture every time, so none of them. At that pose (q -0.600,
  0.264, -0.257, 0.710; the camera sees the solder side 73 degrees
  from face-on, m8 -0.285) the bottom bore ring's cells sit ON the
  bore's centre - centroid column 31.2 against the centre's 31.3 - and
  the "hole" is two uncovered cells beside it at grid 32, none at 48
  or 64. The decimate at 32 clusters on cells of 0.0625 against a
  bore 0.2 across: the bore wall's triangles collapse and go, the
  collar's with them, and the face around the hole is left with gaps
  that show the ground - the ghost - while the ring, right where the
  hole is, stands beside them. Rendered at the bore, magnified, at 32,
  48 and 64: the 32 face full of gaps, the two finer ones closed with
  the ring on the hole. CREW_LEAST 32 -> 48: the 48 decimate keeps
  5 645 triangles against 32's 5 570 (64 keeps 12 430) and moves the
  bore's vertices a mean 0.008 units against 0.030, and in the view's
  own loop at 108x40 with the crew of 8 it costs 31.4 -> 32.3 ms a
  frame of compose (46.7 -> 44.4 Hz in the harness, both past the 30
  Hz cap) - a millisecond for the gaps closed. The chooser's turntable
  rides the same floor. Not changed: the LOD table, so past zoom 3.6
  the 64 decimate takes over as before.
* **The slab's edge and holes are the raster's own silhouette** (2026-
  09-23, the bench, after the floor at 48: "still the same error - don't
  know what you're going on about"). What settled it was the view's
  own frames: the attitude page run exactly as the chooser runs it, in
  a 152x48 console on the stand-in, with every fifth frame's art
  written out and cropped at the bore. Seen from below nearly face-on
  (frame 0110, m8 -0.85) the ring sat on the left edge of a black hole
  that went on to the right past it; the ring was where the mesh's
  bore is, the black was the hole as the decimate draws it - the thin
  ring of triangles round the bore collapses at 32 AND at 48, the
  face's hole is wider and elsewhere than the mesh's circle, and no
  projection of the mesh can sit on it. Three moves of the line had
  missed that: the other face's loops, the snap to the decimate's
  vertices (right in principle, but the decimate has no vertices where
  its hole's edge falls), the face toward the camera. So the slab's
  rim and holes are not loops any more: `_edge` draws the coverage's
  own boundary, dot by dot, off the fold's `reached` bits - eight
  256-entry tables (a mask's dots whose in-cell neighbour a way is
  unset; a neighbour mask's unset dots facing our border) and one
  flood fill of the empty cells from the frame's border, so the
  exterior and any hole of EDGE_HOLE_CELLS or more count and a pinhole
  inside the face does not; the frame's own edge is not an edge.
  Parts keep their crease loops from the exact mesh, snapped
  (`_outline_loops` is parts standing over the top and hanging under
  the bottom, nothing at either level; `_slab_bottom` stays for the
  latter). A line taken from the coverage cannot disagree with it.
  Rendered at the view's six dumped poses with the tree's pass: the
  ring on the hole from below at 0110 and 0130, the ground showing
  through it; the rim a clean line in the steep ones. Cost at 66x40,
  twelve moving frames: `_edge` 4.2 ms a frame (the flood 1.85),
  `_outline` 4.7 with the slab's loops gone (5.9 with them) - three
  milliseconds net; the prototype over dots cost 14.7 before the
  tables. Held in test_render on a synthetic coverage: a rectangle's
  perimeter and a 2x2 hole's four-neighbours draw, the interior and a
  pinhole's neighbours do not, the frame's edge draws nothing, and a
  cell with its lower dot row alone draws that row; the outline's
  loops are the box on top and a box hanging under, neither slab.
  test_render 92, 3173 in all.
* **The edge keeps the cell's own dots** (2026-09-23, the bench, on
  the silhouette's sheet: "still looks a little shifted in the pictures
  you made"). The edge cell was REPLACED by the boundary's dots: a cell
  holding four of the face's dots and one boundary dot showed the one,
  so a dark moat a cell wide ran between the bright line and the face,
  and the line read as standing off the face - shifted. Now the edge's
  mask is OR'd onto the glyph the cell already wears: the boundary cell
  stays as full as the face left it and the line's tone is the only
  change. Rendered fresh at the six dumped poses: the ring on the
  hole's own dots in 0110 and 0130 with no dark ring between, the
  steep ones the same rim as before. Run again after the push as the
  chooser runs it, on the stand-in in this console (66x44, 150
  frames, every fifth written out) and cropped at the bore with the
  camera the view used: the ring on the hole's own boundary cells in
  0110 and 0130 with the ground showing through the hole, the rim a
  clean line hugging the face in the steep poses; the fan continuous
  and the rungs a fraction of a row apart between dumps. A first crop
  put the bore seven rows above the hole - the crop had projected it
  for 108 columns while the view had drawn 66; the header the dump
  carries is the width to crop by. Then, asked for the back seen
  obliquely: the solder side toward the camera at the screenshot's
  own pose (73 degrees off face-on, normal +0.68, +0.67, -0.28) and
  tilted about a diagonal by 60 and by 44 degrees (normals +0.64,
  -0.58, -0.50 and +0.50, -0.48, -0.72), rendered at 108x44 and
  cropped at the bore at 2x. At 73 the hole is a sliver and the edge
  dots sit in it; at 60 and 44 the ring is on the hole's own boundary
  cells with the ground through the hole, nothing beside it.
* **The edge is the LIT coverage's silhouette** (2026-09-23, the
  bench, on those three: "the middle picture shows it most clearly",
  and a hypothesis - "some timing or frame bug: if the edge lags the
  render by a frame it would show most at small angles, where it moves
  fastest"). Tested first: those pictures are one fresh frame each,
  persist empty, no crew, no frame before them to lag behind, and the
  offset is in them; the steady vote holds a whole cell's glyph, edge
  and face together, so it cannot part them; and the crew's pose ahead
  paints face and edge in one pass. Not timing. The bore at that pose
  taken apart in layers at 4x - drawn, face without the edge, every
  reached dot, the edge alone - with the cells round it as numbers
  (reached dots / drawn dots): the coverage's hole is a narrow slit,
  and beside it down-right lie cells reached 8 of 8 that draw NOTHING,
  a band three cells wide. They are the bore's far wall seen through
  the hole: steep to the light, `lit` clamps to zero, blank. Every
  covered-but-blank cell in the frame had heat exactly zero - 79 at
  60 degrees, 52 at 44, and 645 of 1 312 at the screenshot's 73,
  where half the slab is its own rim wall. So the visible dark opening
  was wall plus slit, the ring stood round the slit, a wall's width
  inside the dark: "offset". The same at the slab's rim: the line a
  wall's width outside the lit face, the dark band between them the
  "moat" the bench kept seeing at steep poses. Fix: with the light
  given, `_edge` takes a cell the light left at zero as empty, and the
  line goes round what is drawn - the whole opening, the lit face's
  own edge; mounting holes and slots on the solder side, whose
  see-through was under three cells, now get their rings by the same
  rule. Without the light (the mono path draws walls by class) the
  coverage stands as before. Rendered at the three poses: the ring
  round the whole dark oval at 60 and 44 with the ground through it,
  the rim hugging the lit face at 73, the six dumped poses' bores the
  same. Cost at 108x44 on the 60-degree frame: `_edge` 7.53 to
  7.89 ms, 211 to 301 edge cells. Held in test_render: with heat
  given, a dark band three cells wide moves the edge to the last lit
  column and draws nothing in the band, a dark pinhole no edge.
  The bench, on the three sheets after the change: "looks better like
  that". test_render 100, 3181 in all.
* **Ink never leans below the floor** (2026-09-23, the bench, on the
  next sheets: "the board edge is torn, with a gap to the edge", and
  of the 73-degree back pose, "looks nice, but the board gets a bit
  artificially thick"). Taken apart in `engine.shade` at that pose
  (lean 0.28), every covered cell by class, flatness and art ink: of
  653 blank cells, 599 were FLAT cells with ink 2 - the art's ':' -
  taken to class 0 by the lean dimming, which scales one full class at
  45 degrees (LEAN 3.4) and at 73 takes 2.4; 20 more were ink 1, and
  34 ink 0, the art's own blanks at the bore. What still drew, 475
  cells, had no art hit at all - the parts' walls and lids, on the
  bare-geometry path, at class 2 - so the picture was the parts'
  bodies standing on a plate that had vanished: the "thick block", and
  a rim line (the lit coverage) wandering wherever the blanked face
  happened to end - "torn". At 60 degrees the same rule took the 37
  ink-1 cells beside the bore, and the 50 ink-0 cells there are the
  decimate's flat triangles over the art's hole, not a far wall: the
  raster's `top` flag is the triangle's flatness, and it was set on
  every one of them. The sun could not sort walls from face either -
  the light is view-fixed and the whole solder side has sun 0. Fix:
  an inked art cell's level never falls below the bare floor (0.55,
  class 1), whatever the lean; the art's blanks - its holes, its
  outside - stay blank, so the lit-coverage edge still rings the
  art's hole, which is the mesh's circle and the true one. Rendered
  at 73 and 65: the face drawn whole as '.', the rim one clean line
  on it, two small rings for the parts; the bore's ring on the art's
  hole at 60 and 44. Ambient floors on the key light (0.1, 0.2) were
  tried first and changed nothing - the blank cells never reach the
  glow pass, which skips class 0. The bench: "now the first picture
  looks crisp". Held in test_render at the screenshot pose: under a
  tenth of the covered cells blank, where half were. test_render 101,
  3182 in all. Cost, measured after the fact in the view's own loop at
  108x40 over 200 frames, HEAD against the commit before: compose
  32.1 ms median to 40.2 (p90 45.8 to 53.1), 23.0 to 23.7 frame to
  frame - the plate drawn whole at the steep poses, where half its
  cells were blank and skipped by the glow, the dots and the edge.
* **THE ART SLID OVER THE GEOMETRY: the ray's hit on z = 0, and a
  half-cell in the lookup** (2026-09-23, the bench, on the crisp
  build: "still two holes at i -0.7063 j 0.2652 k -0.4117 real
  0.5110"). At that pose (the solder side 66 degrees off face-on) the
  bore's cells as numbers, reached dots over drawn: the art's blank -
  covered, flat, ink 0 - four cells wide UP AND RIGHT of the
  decimate's see-through, each ringed on its own. First hypothesis,
  the decimate: at grid 48 the bore's wall (0.032 thick, a 0.042
  cell) clusters its two rings into one and the hole comes out
  smaller and elsewhere; `mesh._clustered` took a `keep` and the 86
  corners at radius 0.100 stay exact (5 645 to 5 737 triangles) -
  right, held in test_render, and it moved nothing: the same two
  holes. The art's own blank measured next: 9 columns by 5 rows of
  106x54, 0.17 by 0.19 units, centred - the size of the bore. So the
  LOOKUP: `engine._art_hit` intersected the cell's view ray with the
  plane z = 0, but the slab's faces sit at z -0.069 (top) and -0.101
  (bottom) - the model is centred on its whole height, parts included
  - so at a tilt the ray met z = 0 a parallax away from the surface
  it was shading, 0.07 to 0.10 units' worth, and the art slid over the
  geometry as the board turned: nothing at face-on, most at the steep
  poses, the direction with the tilt - the bench's first description
  of the fault, "clearest when it turns from the back toward the
  component side", was of this, and every line drawn from the
  coverage since was chasing a texture that had moved. Fixed: the
  cell's own view-space point goes back into model space through the
  rotation's transpose, its x and y are the top view's lookup, its z
  against the face the art is read on - `planes` (top, bottom) from
  the solid's own vertices, the bottom from behind - and the rise
  keeps the ray's measure, height over the plane's lean. That moved
  the blank to the other side: three cells UP AND LEFT. Measured
  through the lookup along the four axes, the art's blank reached
  0.105 along +x, 0.070 along -x, 0.060 along +y, 0.135 along -y: off
  by 0.02 in x and 0.04 in y, exactly half an art cell each - the
  index scaled by (w - 1), so the origin fell on cell 52 of 106 and
  row 26 of 54, not 53 and 27. Scaled by w and clamped, the blank
  and the see-through are one hole: at the bench's pose a two-cell
  slit (too small for EDGE_HOLE_CELLS, no ring, the wall's dots round
  it); at 60 and 44 degrees the ring tight on the hole with the
  ground through it; face-on from below (0110, 0130) the ring on the
  hole and smaller than before, art and geometry agreeing. The crew's
  shading tuple carries `planes` too - the workers unpacked seven and
  died with EOFError in the parent, the pose-ahead path had its own
  copy of the tuple. A synthetic plane at z = 0 reads the same both
  ways, so nothing else in the suite moved. Cost: none - the view's
  loop at 108x40 measured 41.0 ms median against HEAD's 40.2, within
  the run-to-run spread. test_render 103 (+2: the ring kept, no
  corner inside it), 3184 in all.
* **The art stops at its disc, and the edge is one dot thick again**
  (2026-09-23, the bench, on that build: "the band inside the edges
  gets really torn at i -0.1350 j -0.7956 k -0.5388 real -0.2413").
  The edge drawn alone at that pose (the solder side 46 degrees off
  face-on): a clean line round the lower left, and along the upper
  right two staircases a cell apart, taken by turns. The rim cells as
  numbers: 27 of them flat, covered, class 0 - art cells with ink 0.
  The art's ink reaches 0.94 to 1.00 of the span by direction (24
  directions measured; the plate has flats), so a covered cell at the
  mesh's rim can land on the art's blank OUTSIDE its disc, draw
  nothing, and the lit edge steps a cell inward there and back out
  where a wall cell (steep, bare, class 2) takes over. Four ways on
  one sheet: as drawn, torn; the art's blank past radius 0.96 counted
  as no hit (`engine.ART_DISC`), so the rim is bare geometry and
  draws by depth like a wall - one clean line; the edge's dots alone
  in their cell instead of OR'd onto the face's - still torn, so not
  the cause; both - clean and thinner. Both taken. The merge goes
  because the moat it was built for (the edge bullet above) was the
  art's parallax, the face itself missing beside the line; with the
  art on the geometry it only lit the whole boundary cell in the
  edge's tone, the bright band the bench had read as thickness.
  Rendered after: the rim one thin line at the torn pose, the ring
  thin on the bore face-on from below (0110, 0130) with the dither
  right up to it, no moat; the back's three poses the same. Held in
  test_render: face on, a point at 0.90 of the span hits the art and
  one at 0.98 does not, and the origin lands on cell 53 of 106, row
  27 of 54. test_render 105, 3186 in all.
* **The parts as blocks and drums, and a grace that follows the cell's
  depth** (2026-09-23, the bench: "the edge enhancer seems to make
  edges between the objects instead of enhancing the objects' edges,
  visible at the fuse and the CM choke"; then "some larger components
  with rounded corners, like the choke, get an edge OF the corner - I
  meant simplifying the object to simple geometry, a block, and
  enhancing its edges and corners; otherwise edges flicker in and
  out"). The layers at the choke (the board's tallest part, 0.186
  high at (0.00, -0.43)): the face draws its lid with the plate's own
  dither and nothing marks it, the edge pass draws nothing there, and
  the outline pass draws FRAGMENTS of its crease loops - so what shows
  is strokes floating between the parts. Two causes. (1) The hidden
  line's grace was fixed at 0.012, while a face tilted 45 degrees
  spans 0.02 to 0.04 units of depth inside one cell at the bench's
  framing, so a lid's edge lost to its own lid's near corner in the
  same cell. The grace now adds the cell's own depth span, read off
  its four neighbours (OUTLINE_SLOPE): the outline drawn alone at 45
  degrees went from 250 cells in 30 pieces, the largest 20, to 484 in
  31, the largest 72; at 65 from 198 in 22 (largest 45) to 459 in 17
  (largest 163) - the loops close into boxes and rings. Cost: the
  pass 8.4 to 9.4 ms at 108x44. (2) A part's crease loops are
  wherever its tessellation folds past 60 degrees: on a rounded part
  the rounding's own facets, an edge of the corner, coming and going
  with the view; a rounded extrusion folds only at its two end
  profiles. Measured over the 917 loops: 759 are two-corner ridges,
  and the loops of one part - the choke's rounding, base and lid -
  fall in four. So the pre-scan (`_stereotypes`, once per outline
  source, 151 ms): loops of one side whose footprints nest by 0.6 of
  the smaller (STEREO_NEST - any overlap chained neighbours: five
  capacitors each swallowed 85 to 91 loops of the pin fields round
  them) are one part; a part is a DRUM when its widest loop's own top
  corners sit on one radius and not on a box's sides (a chamfered
  square's corners share a radius too - the CPU came out round in the
  prototype; and the capacitor's rim ring sits 0.04 under the eighty
  facets of its domed top, so the lid is the widest loop's, not the
  highest points') - else its BLOCK, the least oriented box round all
  its points, lid and four legs to the slab. A loop in one vertical
  plane is an end profile when it reaches the base within a
  millimetre - two of one width and height facing across are one
  block, an odd one a sharp ARCH - and a HOLE in a wall when it floats
  over the base: the screw terminals' 40 openings, 0.016 up, which as
  arches were rectangles ("the holes in the screw terminals have
  become rectangles"), drawn as the oval inscribed in their bounds.
  A wall's arch or hole draws only where the wall faces the camera by
  0.25 (STEREO_FACING): face-on they are seen edge-on, and lay as
  short bright dashes along the rim and in pairs on a terminal's two
  walls - the four rings on the bench's screenshot at i -0.0489 j
  0.0245 k -0.0036 real 0.9984. The board's 917 loops come out as 262
  blocks, 4 drums, 15 arches and 40 holes; the pairwise footprint test
  cost 2.6 s until swept along x. The snap of the outline to the
  decimate's vertices (`_snapped`) goes: the primitives are the
  mesh's own geometry, and the disagreement it was built for was the
  art's parallax. Rendered at the component side 45 and 65 degrees:
  the parts as boxes with legs, the capacitors as circles, the
  terminals' openings as ovals; the view's loop at 108x40 measured
  43.6 ms median (41.0 before), p90 56.1. Held in test_render: a box's
  loop is a block of eight segments; a lone twelve-corner lid on one
  radius a drum; a chamfered square a block; two facing profiles one
  block; a base ring under a lid one part; neighbours overlapping by a
  tenth two blocks; a loop floating in a wall a twelve-segment oval
  that draws nothing edge-on and draws at 60 degrees; and at 65
  degrees the outline's largest connected piece is over a hundred
  cells (333 of 675). The bench, midway: "looks crisp now".
  test_render 114, 3195 in all.
* **The block takes its widest loop's orientation, its lid the
  crest's own box, and a circle inside a part is a ring** (2026-09-23,
  the bench, on that build: "the smallest PE terminal with its little
  screw hole still gets a square screw hole; the CM choke gets no
  crisp edge; the phase terminals get a strange edge halo; the USB
  connector gets a parallelogram distortion"). Each measured off the
  pre-scan. The USB shell's widest loop is a rounded square of 14
  corners, whose least-area box lies at any angle at all - the
  2-degree search happened on 60 - and the choke's group of 25 loops
  had its box turned 14 degrees by a side feature 0.036 wide: a box
  fitted over every point of a part is turned by whatever pokes out.
  Now the block takes the angle of the part's WIDEST loop, and the
  board's own axes unless turning saves over 2 % of the area
  (STEREO_TURN); the box round all the points is drawn in that frame.
  The phase terminals are pairs of rounded profiles 0.23 wide whose
  flat top runs 0.16: the full box's lid stood a cell outside the
  rounded shoulder, the halo, so the lid is now the box round the
  CREST's points in the block's frame and the legs lean from the base
  corners to the lid's - the frustum of the profile; a plain box's
  crest is its base and the legs stand straight. Every other loop of
  a part whose top corners sit on one radius at even spacing draws
  as a ring at its own height; even spacing (STEREO_EVEN, gaps within
  2.5 to 1) replaced the box test for a circle, which took every
  octagon for a chamfered square - eight corners sit two to a side of
  some box - while the CPU's chamfered corners crowd in pairs, 4
  degrees at a chamfer against 86 along a side. The PE terminal's
  "square hole" is its clamp: an 8-corner rounded square 0.039 wide
  (corners a degree apart in pairs), 1.2 cells at the bench's
  framing, drawn as a block; the hole itself has no crease loop in
  the mesh - a smooth rim, a five-corner arc 0.004 across - and its
  1.5-cell diameter is under the outline's 2-cell minimum, so nothing
  round can be drawn there at that zoom; zoomed in, the clamp grows
  and the hole would need a loop the mesh does not have. Also: a loop
  in a vertical plane with no height is a ridge along a part's top
  (the choke's, three corners 0.14 long) and joins its block, not a
  hole; and `_collinear` allows 1e-4 of spread against the line, the
  mesh's tenth of a millimetre. 322 primitives: 262 blocks, 4 drums,
  15 arches, 41 holes, no block over 0.06 turned off the axes. The
  view's loop at 108x40: 40.9 ms median, p90 51.5. Held in
  test_render: a lid with an eight-corner circle on it is a block and
  a ring; a nested loop poking out a corner does not turn the block;
  a crest 0.2 wide on a base 0.4 leans all four legs in. test_render
  117, 3198 in all.
* **The preload: the front page fetches the model behind itself, and
  the readout's first inquiry says so** (2026-09-23, the bench: "as
  the first page in READOUT, present what is being loaded in the
  background - prefetch or preload for rendering and modelling -
  depending on whether the machine has enough free space"). The
  chooser starts every view as its own python process, so nothing the
  front page holds in memory reaches a view; what can is a file.
  Measured cold, one process: the import 0.27 s, the STL's parse
  0.66 s (116 880 faces, 5.8 MB), the six decimates 2.87 s - each
  parsing the file again - the outline's exact index and its loops
  1.47 s, the stereotypes 0.14 s, the shadow casters 0.46 s; the
  decimates, the index with its loops and the primitives pickled are
  6.9 MB and load in 0.09 s. So `coaxial/preload.py`: one pickle
  under the user's local application data (LOCALAPPDATA, or ~/.cache)
  - a cache file BESIDE the model was tried before and is not wanted
  in the tree, and this is not beside it - stamped with the model's
  path, size and mtime and twelve hex digits of a hash of the mesh and
  wireframe sources, so a changed decimate or fit makes it stale, and
  read only when the stamp matches; a torn or foreign file is no
  preload. Gated on the room: a gigabyte of free memory to build, a
  hundred megabytes of free disk to keep - measured here 38.7 GB and
  383 GB - else a refusal in words. The front page starts a child at
  BELOW_NORMAL priority (`python -m coaxial.preload`) and reads its
  steps line by line into the readout's first inquiry, PRELOAD: the
  model and its size, memory and disk free, the last four steps, the
  status - "ready: 6.9 mb on disk" in 0.11 s when the pickle stands,
  "written: 6.9 mb" after 5.8 s when the child built it, or the
  refusal. A view's `_lods` adopts the pickle into its caches under
  the builders' own keys before decimating anything: the attitude
  view's three-frame smoke run, crew spawn included, 6.4 s without
  the pickle and 2.9 s on it. Held in test_views: the first inquiry
  is PRELOAD carrying the status, a saved bundle loads back on the
  model's stamp and not on another, the room answers; in
  test_render: adopted, the LODs, the outline source and the
  stereotypes are the bundle's own objects. test_views 221,
  test_render 118, 3206 in all. **The bench's first start threw**:
  "partially initialized module 'coaxial.orientation' has no
  attribute 'MODEL'" out of the warm-up's render. `orientation` and
  `wireframe` import each other, harmless in one thread; the new
  preload thread imported `orientation` FIRST while the warm-up
  thread imported `wireframe`, so wireframe finished on a partial
  orientation and the render read MODEL before orientation's body had
  reached it. The preload thread now imports `wireframe` - the same
  module the other thread does, so it waits on that import's lock -
  and takes MODEL through it. Five fresh interpreters running both
  threads together after: no exception, the warm-up ready and the
  preload "ready: 6.9 mb on disk" every time.
* **The line lies on the face's dots, lifted 3.0** (2026-09-23, the
  bench, on the blocks: "there is still a lot of halo in the edges").
  Cell by cell at 30 degrees, the face alone against the face with the
  outline: along every line the cells fell from three or four dots to
  one or two - the outline REPLACED the cell's glyph with the line's
  own dots - so a line ran as a dark groove with bright specks, and at
  lift 4.5 the specks glowed: the halo. The lid cells themselves,
  measured, were drawn (four dots, heat 3 to 4) and the walls too
  (heat 1 to 3.7) - it was never the face missing, only the line's
  cells. Four ways on one sheet, a phase terminal and a capacitor at
  30 and 45 degrees: alone at 4.5 (as drawn, the groove and the glow),
  merged onto the face's dots at 4.5 (no groove, a bright band),
  alone at a small lift (the groove, unlit), merged at 1.5, 2.5 and
  3.5 (a denser, brighter run of the same dither - 1.5 vanishing,
  3.5 readable). Taken: the outline's and the edge's dots OR'd onto
  the face's, and OUTLINE_LIFT 3.0 - the 4.5 was measured for a lone
  line of one or two dots that needed tone to be seen at all, and a
  merged cell carries five or six. The rim takes the same rule; at
  4.5 merged it had read as thickness.
* **A block has twelve edges: the base's four too** (2026-09-23, the
  bench: "still no edge round the CM choke - and don't start going in
  circles adjusting thresholds"). No threshold touched; the choke's
  block traced dot by dot at the bench's face-on pose instead: 90 dots
  emitted, 0 hidden by the depth test, 90 landed in 38 cells - the
  block WAS drawn, whole. What it drew was the lid, the crest's box
  (y -0.46 to -0.40: the toroid's top arc and a ridge), and four legs
  leaning out to the base's corners at y -0.50 and -0.38 - and no
  base. Seen from above the base is the silhouette of a rounded part,
  so the outline ran through the body's middle with four short ticks
  toward corners nothing joined: no edge round the choke. The crease
  loops never had base edges because a part's footprint shared
  corners with the copper and went out with it; a primitive's base is
  its own rectangle. `_block` now draws lid, base and legs - twelve
  segments; on a straight box the far base edges lie behind the body
  and the depth test hides them, the near ones mark where the part
  meets the board. Rendered face-on and at 30 degrees: the choke's
  base rectangle round it with the crest's box inside, the phase
  terminal's outer frame, the USB shell's; the composite at 30
  degrees shows the choke as a frustum. Held in test_render: a box's
  loop is one block of twelve segments, a crest 0.2 on a base 0.4
  still leans four legs.
* **The preload's inquiry shows once** (2026-09-23, the bench: "make
  READOUT show that it preloads only once, not in a loop"). The
  PRELOAD page was the first of the cycle and came round with it. Now
  `readout.draw` drops it for good the moment the page index has
  passed it - the state remembers, the index shifts back so the same
  page goes on - and the cycle runs IDENTITY, FITMENT, PROVENANCE from
  then on. Held in test_views on a scripted clock, 240 s of frames:
  the inquiries run PRELOAD, IDENTITY, FITMENT, PROVENANCE, IDENTITY,
  FITMENT, PROVENANCE, IDENTITY - the preload once. test_views 222,
  3207 in all.
* **A block is a block, and what nests in it is it** (2026-09-23, the
  bench: "one of the connectors still has a square hole"). Two
  candidates measured. The PE terminal at 152x48: its clamp is a
  rounded rectangle 0.026 by 0.039, 2.1 cells, its countersunk hole
  0.034 across - 1.8 cells - with facets tilted 20 to 50 degrees, so
  no fold reaches the crease rule's 60 and the hole has no loop; a
  ring 1.8 cells across is a square either way, and no threshold was
  touched for it. The phase terminals: every primitive inside one
  listed - a block 0.182 wide (x 0.719 to 0.901) inside an arch 0.23
  wide (0.695 to 0.925), because a screw terminal has TWO profiles per
  wall, outer and inner; the inner pair made a block INSIDE the
  outer's frame, two rectangles 0.024 apart, 1.3 cells at the bench's
  size: the square hole. And the crest's lid inside the base's box did
  the same to every rounded-shoulder part - the lid was my fix for a
  halo that turned out to be the groove. Two things, no thresholds.
  A block is the box round all its points, lid over base, legs
  straight; a single-edge loop (a wire's silhouette, a pin - 125 of
  them) is its own stroke rather than a box round a diagonal. And
  what nests inside a block's footprint on the same side - a block,
  an arch, a stroke, by the `_nested` rule the loops are already
  grouped by - is absorbed into it: the box round both, to the
  taller's height; drums, the rings on lids and the holes in walls
  stay their own. 322 primitives become 268: 87 blocks, 125 strokes,
  4 drums, 11 arches, 41 holes. Rendered face-on and at 30 degrees:
  the phase terminal one frame, the pair one block, the choke one
  rectangle. Held in test_render: a crest narrower than the base is
  still one block over the base with straight legs; one edge is a
  stroke; a 0.4 box inside a pair of 0.5 profiles is one block of
  extent 0.5 to the profiles' height. test_render 120, 3209 in all.
* **wireframe.py split into nine modules, one concern each** (2026-
  09-23, the bench: "split out wireframe and all the other filters so
  it is not a mess of if-statements - hard to overview for humans and
  LLMs without the history, and it burns tokens compared with a slick
  solution"). 3 098 lines cut by line range - every line moved once,
  none copied (the structure suite's duplicate-definition check), none
  lost (the script asserted it) - into: `solids` (159 lines: the STL
  as solids, decimates, the bore kept, the slab's faces, casters),
  `creases` (302: the crease loops off the exact mesh), `stereotype`
  (492: the pre-scan), `shading` (884: the depth ramp, the lamp, the
  key light, the halftone, the rim glyph, the face art, the shadow
  map), `ground` (335), `lines` (356: `_trace`, `_outline`, `_edge`),
  `triad` (138), `steady` (57), and `wireframe` (581: the pipeline -
  `render`, the face held and painted one pose ahead, `_lods` and the
  preload's adoption - with THE MAP in its docstring). The import
  graph is a DAG: `orientation` imported `wireframe` at the top and
  `wireframe` imports `orientation`, harmless in one file, but any
  new module importing `orientation` first would have met a partial
  `wireframe`; the import is now inside the one function that renders.
  The preload's stamp hashes the modules that build the bundle - mesh,
  solids, creases, stereotype - so the pickle rebuilt once. What the
  move caught: three moved functions reached their caches by bare
  name (`_MESHES`, `_OUTLINES`, `_STEREO`) and the structure suite's
  "uses only names it has" said so before any test ran; two locals
  named `solids` shadowed the module in the attitude view's boot and
  in the adopt test; and three monkeypatches had to move to the
  module that LOOKS UP the name - `_paint` reads `wireframe._outline`,
  `_stereotypes` reads `stereotype._outline_source` - not the one
  that defines it. Every external reference was repointed by script
  (longest names first, so `_outline` never ate `_outline_source`).
  Measured after: the view's loop at 108x40 42.5 ms median (42.6
  before), the attitude smoke and the front page's smoke exit 0, the
  render suite 120, the views 222; the structure suite grew to 715 -
  one import, one docstring and one names check per new module.
  3241 in all.
* **The floor's lines are their supercover and the rungs slide**
  (2026-09-23, the bench: "the perspective lines toward the horizon
  look jagged and 'static'"). Two faults, both on the raster at
  108x44. The fan's lines were DASHED: `_segment` sampled each piece
  one dot along its steeper axis with the count truncated, so a piece
  1.9 dot rows tall lit two dots and skipped one, and a diagonal
  stepping a dot column left a gap at the step - 264 pieces a line,
  a gap in most. Now every piece between the dot-column and dot-row
  crossings lights the dot its midpoint falls in: the line's
  supercover, a chain of touching dots - 4 010 dots to 4 512 (twelve
  percent) in 1 728 to 1 786 cells, the static and its first step cast
  in 29 to 46 ms once per window size. And the rungs JUMPED: at
  RUNG_STEPS 24 a rung stood 0.21 s and moved, the near one 0.42 rows
  (1.7 dot rows) at a time. At 96 the largest move on screen is 0.105
  rows at 108x44 and at 150x44 (0.048 at 60x20), under half a dot row,
  every 52 ms. A cold step cost 6.7 ms at 108x44 (9.2 at 150x44), and
  at 96 steps one falls every 52 ms for a size's first five seconds;
  `_backdrop` now lays the rungs' cells over the static's settled
  greys, copying and greying only the cells a rung touches, the rung
  dots placed inline (3 240 `_ground_dot` calls a step were most of
  it): 5.0 and 6.9 ms, the output bit-identical to the old over 288
  steps with the old sampling. The cache holds two sizes' worth of
  steps (BACKDROPS_KEPT 192) and starts over. In the view's own loop
  at 108x40 with the crew, 200 frames with the cold steps inside them:
  compose 34.9 ms mean, 32.1 median, 45.8 at the ninetieth percentile,
  23.0 ms frame to frame. Held in test_render: a segment from dot
  (0, 0) to (6, 7) lights 14 touching dots and none off the line, a
  step moves an on-screen rung by under half a dot row, the cache
  fills to the cap and the step past it starts over. test_render 99,
  3180 in all.

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
