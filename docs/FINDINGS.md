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

* The link goes quiet now and then. Open. 600 requests ruled out four
  causes; `power_check.py`, `thermal_calibrate.py` and the conformance
  suite tolerate the silence and retry.
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
  115 200 everything reported; the wire ran at 80x the number in the
  link report. Found the day the THVD1450's rating went into
  HARDWARE.md; the baud joined the record at CAL_VERSION 9.
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
* With the link up and the session holding COM4, `find_board.probe`
  opened it a second time, Windows refused, and the checklist printed
  the board as not answering. Two `dbg.py` sessions with COM4 open read
  every probe silent and the board was diagnosed as halted, started
  over SWD and reflashed; none of that was the matter with it.
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
* Two gate driver stages ran 15 C hotter than the third. The full suite
  was started three times and none of the 1970 checks could say
  anything; a 600-sample pin count and a register dump found it: the
  gate pins at CubeMX's LOW speed. VERY_HIGH since.
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
| a sensor enabled at 60 ms never reported | the interval went out little-endian on a big-endian wire - 27 minutes |
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
* The supply's idle current was measured at 0.050 A but through a
  shunt the owner does not trust.
* The bench suite's regression: the thermal observer reading two ADC
  channels and two SPI transactions on every poll, and before that a
  poll blocking long enough to lose a Modbus character.

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

* 2026-08-27, six minutes after W32Time had synced, this PC sat 947 ms
  behind UTC and was losing a further 25 ppm; Windows had declined to
  step it, the offset being inside the 1 s `MaxAllowedPhaseOffset`.
  `set_time_from_pc(reference='utc')` measures the PC against NTP and
  takes both out; with no route it falls back to `'pc'` and says so.
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
* Loading 7.6 GB again was most of a run's wall time; the model is
  loaded once per run. Killed from outside, 8.4 GB stayed on the card;
  a session left running held 9.69 GB for 27 minutes at 1 %.
* The desktop alone held 2.6 GB of VRAM at 0 % utilisation.
* `prompt_save ... total state size = 342.623 MiB`; restoring a
  311.575 MiB checkpoint threw `std::bad_alloc`; two copies of the
  weights on a 16 GB card was a 500, `cudaMalloc failed`. Tuned: ten
  questions, twenty-seven calls, zero `std::bad_alloc`, one load.

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
* The slab's top was assumed at z 0 for an evening; measured from the
  mesh, the gate a millimetre over it shows 44 loops wider than 0.12
  units. 914 of the 1 458 edges in the 95 loops drawn were under half a
  cell face-on.
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
* Four causes of the occasional quiet link, over 600 requests. The
  fault itself stays open.
