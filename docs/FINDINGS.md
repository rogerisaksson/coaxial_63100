# Hardware & Firmware Findings

## Resolved Defects

* **`ADC_PCSEL` Accumulation:** The HAL blindly ORs `PCSEL` without clearing it, leaving every configured channel permanently connected. DC bus noise dropped 7x after explicitly clearing it pre-configuration.
* **Silent Zero on a Failed Conversion:** `ADC_ReadOneChannel` returned void and left `*outRaw` at 0 when `ConfigChannel`, `Start` or a 10 ms `PollForConversion` failed. On a differential channel code 0 is 0 V, so a failure was indistinguishable from a measurement. Now `bool` through all six `Board_*` readers; the noise and burst loops abort rather than fold a zero into the mean. `h_adc_burst` took on the value checks `h_adc_noise` already had, so its refusal is SERVER DEVICE FAILURE rather than ILLEGAL DATA VALUE.
* **Blind Differential Reads:** `ADC_ReadDifferentialVolts` hijacked whatever state the ADC was left in rather than configuring its own channel. The function was purged.
* **Modbus Qty 0 Ignored:** A valid request for zero items failed silently due to a lazy 7-byte minimum PDU check. Lowered to 6.
* **HSE Boot Warning:** Blindly rejected PLL1 even when sourced directly from HSE. Fixed.
* **String Formatting Exception:** Missing tuple parentheses in the MCP renderer swallowed outputs. Caught solely because the exception handler was cynically narrow. Keep it that way.
* **`port_state` Unstubbed in `test_link_diagnose`:** The suite stubbed `comports`, `connect` and `check_power` but not `port_state`, which opens a real port. The result depended on the bench: it passed with the probe connected and failed with it out, where COM4 read `busy` and the checklist stopped one step short of what the check asserted. Stubbed; the BUSY branch it had been reaching by accident now has its own check.
* **Calibration Rollback That Rolled Back Nothing:** `Board_CalSetParam`
  assigned first and, on a value that would divide by zero, reverted by
  reloading flash. On a board whose record has never been saved that reload
  fails and changes nothing, so the refused value stayed - `vref_uv` left at
  zero, every reading garbage, and the command still answering ILLEGAL DATA
  VALUE. Found by `test_conformance.py` re-reading the record after every
  refusal it provokes. Now validated before the assignment.

## LLM & Host Infrastructure

* **LLM OOM Crashes:** `llama-server` throws `std::bad_alloc` due to its own bloated prompt cache and checkpoints, not prompt length. Fixed by disabling `LLAMA_ARG_CACHE_RAM` and `LLAMA_ARG_CTX_CHECKPOINTS` in the daemon.
* **`.port` Read as a COM Port:** `SimulatedSession.port` is a bus label (`AX`), never `None`. `link_diagnose` guarded on `configured is None`, so a session that fell back to the stand-in spent 15 s on an SWD probe and then reported "Configured port AX: not among the ports above - the cable may be unplugged". The `simulated` marker that fixes this in `_interface` already existed and was not used here. Step 4's closing advice also opened with "Powered" whatever step 1 concluded.
* **`check_power` Discarded Its Own Reading:** With no target the programmer takes 30.3 s - a second connect attempt at 8 MHz - against a 15 s budget, and `TimeoutExpired.stdout` already holds `Voltage: 0.00V`. The handler returned `None`. Now parsed from the killed run.
* **`-Ask` Pinned the Card:** A one-shot was exempt from the unload on prompt exit while the same script passes `--keep-alive 30m`, so four smoke tests left 8.4 GB resident with nobody at the prompt. `-Ask` now takes a list: one load, N questions, one release.
* **`--sections` Read Only Under `--match`:** `run_tests.py --live --sections tools` ran all three - `tools` and `all` both returning 176 checks in 255 s. Coverage tiers were unaffected; they set the sections directly rather than through the flag.
* **Fabricated telemetry when a tool fails:** `llama3.1:8b` reports readings it never took and substitutes physical constants (NTC B=3950 for the real 3380). **Mitigation:** `gemma4:12b` is the default - slower, and it abstains instead. The structural guard is that a value only reaches a verdict as a `report` argument, never as prose.

## Confirmed Behaviors (Not Defects)

* **JTAG Connect-Under-Reset:** Fails because ST-Link probes neglect TAP re-initialization. Hardware is innocent. Workaround: SWD or `SWrst`.
* **Halted Core Silences USART3:** An aborted JTAG/SWD reset leaves the core halted, killing serial comms. Use `mode=HOTPLUG`.
* **Phase V 0.85V Offset:** Isolated op-amp failure on a single board. Do not calibrate around broken hardware. Since the phase channels report amperes it reads as -52 A with nothing connected, which is what makes it hard to ignore - and `0x6E` device 3 op 3 would now make it vanish. Do not zero Phase V on this board.
* **NTC Bit-Exact "Anomalies":** Johnson noise is 120x below LSB. Bit-exact readings are physics, not a frozen register.
* **UART Overrun (ORE):** Latches and kills RX permanently. Now explicitly cleared via `ICR`.
* **ADC Offset Calibration:** Drifts ~100mV across boots because it runs against an unpowered reference (`AFE_ON` low).
* **Probe Readings Under Concurrent Use:** `--power` drives the ST-Link and `port_state` opens the VCP; neither says anything about the hardware while another process holds them. A concurrent `run_tests.ps1 -All` makes `--power` report `ST-LINK error (DEV_CONNECT_ERR)` in 2.3 s where the same call reads `0.00V` in 15.2 s once the bench is idle. A leftover `dbg.py --repl` makes `port_state` report `busy`, which is what the checklist then correctly says.

## Confirmed Behaviors (Not Defects) - continued

**`AFE_ON` (`PB2`) powers the IMU.** Measured 2026-08-26: with it off the
BNO08X answers reads and never acts on a write. With it on, then reset, then
`Set Feature 0x05` at 20 ms: 135 rotation vectors in 4 s, 51 distinct.

What the fault looked like on the way there, all of it wrong:

| Suspected | Ruled out by |
|---|---|
| SPI mode or bitrate | the advertisement reads back clean at 1.48 MHz, mode 3 |
| chip select not reaching the part | clocking with CS high gives `ff ff ff ff`, with CS low `14 01 00 00` |
| a shorted or held SPI2 pin | PB12..PB15 each drive high, drive low and follow both internal pulls |
| the buffer truncating the advertisement | 276 bytes into 320 |
| `PS0`/`WAKE` not wired | H_INTN answers a wake in 0-3 ms |
| the part in the bootloader | it reports SH-2 3.2.0, part 10004148 |

`product_id` looked like proof the link was two-way and was not: the part
sends an unsolicited product id response after every reset, so the answer was
in the queue whether or not the request arrived.

**A1335 on SPI4, brought up 2026-08-26.** Three things cost time, all of
them in code:

| Symptom | Actual cause |
|---|---|
| `HAL_SPI_Init` returned `HAL_ERROR` | `IS_SPI_HIGHEND_INSTANCE` names SPI1..3 only, and SPI4 refuses a data size above 16 bits. The 20-bit packet goes out as four 5-bit words - exactly twenty clock edges under one chip select |
| every register returned the previous one's value | the answer lags a frame. The address arrives on MOSI bits 17..12 while MISO has already shifted out bits 19..16, so a read is two packets. Asking TSEN, FIELD, TSEN in turn returned the previous register every time |
| the angle wandered with the board still | `FIELD` reads 3 gauss - there is no magnet in front of the part, and the angle is then noise. Not a fault |

Confirmed once the framing was right: `TSEN` 2471 counts = 308.9 K = 35.7 C,
`FIELD` 2 gauss, the poll loop at ~22,000 reads a second with no errors.

The register map is not in `datasheets/angle_sensor` - that datasheet defers
it to the Programming Manual. Addresses come from
`github.com/ScranchNew/Allegro-A1335-Sensor-library`, and `0x6E` device 1
op 5 sets which one the loop reads so a better address needs no rebuild. The
CRC field's width is documented and its polynomial is not, so it is reported
and never checked.

## Ruled Out

* **`Chat` Decomposes into Turn, Steering and Budget:** All three touch `client`, `history`, `io_log`, `language` and `last_channels`; steering owns one attribute alone and budget none, so three objects need a shared state struct all three hold. `debug.py` is 1544 lines of which `Chat` is 1088; moving the wording (112) and module helpers (105) out leaves 1327. No class ceiling in the structure suite either: a mixin split defeats one, and a module ceiling would only demand this refactor.
* **`-Wconversion` Finds Silent Truncations Here:** 113 warnings across all seventeen of our sources, 112 inside HAL/CMSIS headers they merely include, one ours - a `0U` ternary narrowing to `uint8_t`. The integer-heavy files are heavy in explicit casts. The flag now sits on the ten HAL-free sources, where it also guards invariant 1.
* **NTC Sample Time Too Short:** The 15nF capacitor provides the necessary charge.
* **DIFSEL Ignored:** It is written safely while the ADC is disabled.
* **VREF Sag at 475 MHz:** A coincidental 1% shift on the DC bus; other channels shifted completely asynchronously (up to 9.7%).
* **Prompt Length Causing OOM:** See cache mechanics above.

## Open Anomalies

* **Unpowered Calibration:** The proposed fix (waiting for `AFE_ON`) remains untested.
* **IN11 (`Cinj`) 9.7% Shift:** Explained by the schematic, not by the ADC. `Cinj` is the output of the RS485 pilot detector - a 1 kHz to 10 kHz band pass into a TLV3492 comparator pair (HARDWARE.md, Safe Torque Off). With no master injecting a pilot there is no signal, so the channel reads whatever the band pass makes of the board's own noise, and a frequency-dependent reading is what that looks like. Re-measure with a pilot present before treating any of it as drift.
* **DC Bus Read Discrepancy:** Two different read paths yield a persistent ~30 mV delta.
* **PE15 (`nFAULT`) Polarity:** Reads 0 (asserted) when the AFE is powered. Three explanations were open; the intended logic is now stated - high is normal, low is a fault - which **rules out inverted logic** and leaves a real fault or a supply pull. The firmware inverts nothing: `Board_Pe15` returns raw IDR and both the Modbus discrete input and `0x6D` pass it through, so the measurement is the pin's electrical level.

  Explained, and the earlier explanation is superseded: it attributed the
  level to an unpowered open-drain output on the gate drivers.
  **A 2EDL8034 has no fault pin** -
  PG-DSO-8, and the eight are VDD, HB, HO, HS, HI, LI, VSS, LO. `PE15`
  carries `FAULTIN` from `STO.SchDoc`, where it sits on U11 (NL7SZ97) pin 1
  and on U4 (TPS3840PL30) MR, with R99 220 ohm to `FAULTOUT`.

  Measured 2026-08-27 over SWD hotplug, AFE off: `GPIOE->IDR` bit 15 = 1,
  `GPIOB->IDR` bit 2 = 0. So the pin is high when the AFE is off and low
  when it is on, unchanged as a measurement - only the mechanism was wrong.

  **The conformance suite's "independent witness" is still worthless as
  written.** It reads a pin the MCU does not drive, whose source is a logic
  gate fed through R97 100 kohm, and it will change meaning the moment the
  STO chain releases. That check needs replacing before the supply is
  switched on, not after it starts failing.

  **No MCU pin powers the drivers, and none should.** The supply is released
  by the Safe Torque Off chain, unlocked by a common-mode pilot tone the
  master injects on the RS485 pair - see HARDWARE.md. This is also why
  `s_parts` gives the drivers and the FETs `power = "STO chain"`, which
  names no GPIO on purpose.

## Closed 2026-08-27 - the synced current path

Three firmware defects, all found by comparing two paths that must agree
rather than by reasoning about hardware - which is the method that worked.

| Symptom | Cause |
|---|---|
| injected triple read (-31344, 24587, -32355) where the meter read (1423, -8285, 392) | `Board_SyncOnInjected` cast JDR to `int16_t`. JDR is **offset binary**, 32768 = 0 V. Now goes through `Board_AdcDifferential`, which is the one definition (invariant 7). `board_adc.c` already carried a comment about this exact bug from an earlier session. |
| `pilot_ok` and `level_ok` false for every call ever made | `STO_ReadOne` passed `NULL` for `Board_AdcRead`'s `scaled` argument, which refuses a NULL before reading anything. |
| `Board_SyncArm` could never succeed | `Board_SyncReady` required `JSQR != 0`, but JSQR is written by `SYNC_ConfigPhase` **inside** Arm. Ready now means "a timer to trigger from" and nothing else. |

Measured after the fixes, AFE on, gate drivers off: injected (1433, -8136, 390)
against a meter reading (1456, -8118, 442) - inside the ~55 LSB the meter
reports as its own noise. **49976 triples/s over 4.05 s against a 50 kHz
PWM, zero overruns**: one per period.

**The sample point is tunable and the latency is constant.** Sweeping CCR4
and reading `TIM1->CNT` in the interrupt: the trigger fires on the
**down-slope** as CNT passes CCR4, and the handler reads CNT a constant
**~965 ticks (4.06 us)** later, whatever CCR4 is. Scatter is +/-20..45
ticks, which is interrupt latency, not sampling jitter - the sample itself
is hardware-triggered. `at` is therefore a witness with a fixed offset and
**not** the instant of the sample. CCR4 = 0 stops the trigger outright
(OC4REF in PWM1 never activates); CCR4 > ARR is refused and op 4's reply
reports the unchanged register.

## The STO interlock works, and the gate drivers cannot be enabled

Measured 2026-08-27 with KEEPALIVE running: `fault` set, `Board_PwmClearFault`
returned 1, `Board_PwmEnable` returned **0**, `fault` set again - all inside
one round trip. PE15 is still low, so the break re-latches the moment the
latch is cleared. No pilot tone on RS485, so the chain has not released.

Consequence: Clevel is identical before, during and after an attempted PWM
run (mean 1304 / 1302 / 1304, sd 2299 / 2297 / 2301) because nothing
switched. That is the interlock, not a null result.

## The main-loop keepalive is too slow for an IMU cargo read

Simulated 2026-08-27 on `electronic_simulations/sto/sto.asc`, 30 ms
transient, waveforms read out of the `.raw` rather than by eye.

The chain does release: `VGATEDRV` holds **14.904 V** steady from 7.26 ms to
15.89 ms. `VLATCH` sits at **2.895-3.002 V** while solid - 107 mV of ripple
on a 3 V level, pumped at 100 kHz.

The collapse is a two-stage thing and only the second stage is slow:

| Stage | Behaviour |
|---|---|
| `VLATCH` crosses 0.75 V | **abrupt** - it gates a switch (`.model DCDC_CONV SW(Vt=0.75 Vh=0.35)`), so it snaps rather than decays |
| `VGATEDRV` after that | exponential, **tau = 115.0 us** |

    below 10 V ................................  46 us
    below 2EDL8034 VDD UVLO falling, 6.7 V ....  92 us
    below bootstrap UVLO falling, 5.7 V ....... 111 us

**How long may the pump stop?** 107 mV of droop per 10 us at 3 V is
10.7 mV/us, so reaching 0.75 V takes on the order of **200-400 us**
depending on whether the discharge reads as linear or exponential. Not
measured directly - the model's pump never gaps, it runs to 18 ms and
`VLATCH` had already collapsed at 15.75 ms for a reason not isolated here.

**That is the problem.** `Board_ImuPoll` can stall the main loop for
**1.5 ms** on a 276-byte SHTP cargo at 1.48 MHz - the comment in `main.c`
says so. That is 4-7x the latch's hold time, and the gate drivers are below
UVLO 92 us after it lets go. A keepalive toggled from the top of the main
loop is therefore **not safe once the IMU is streaming**, even though its
mean rate (214 kHz idle, 124 kHz under Modbus) looks like ten times the
margin it needs.

Two ways out, and the second keeps the dead-man property:

* break the SHTP cargo read into chunks so the loop never stalls that long;
* toggle from a periodic interrupt that is gated on a counter the main loop
  refreshes - the ISR stops when the loop stops, but a 1.5 ms SPI transfer
  no longer starves it. A free-running timer is still not an option: it
  keeps toggling after a hang, which is the one thing the chain exists to
  catch.

Caveats on all of the above: the model's pilot injector `PAM8406` has an
empty value, so no pilot tone was injected and the run reproduces the real
board's held-off state rather than a released one. `VGATEDRV` also cycles
(up 1.66-4.18 ms, down 3.08 ms, up 7.26-15.89 ms) for a reason not chased.
These are the model's numbers, not the board's.

## 0x42 reads the channels, so it refuses while the triple is armed

Found by the sweep above, not by a test. `h_adc_table` takes a **reading**
of every channel on the way past, and `Board_AdcRead` is gated by the meter
interlock - the injected group owns PCSEL while it is armed. So the whole
command answers SERVER DEVICE FAILURE, and a host cannot even look up a
channel by name: `index_of`, `channels()` and therefore `configure` all
went through it.

`0x6D` kind 0 is the map rather than the table. It answers what exists
without measuring anything, and works armed or not. `Analog.index_of` and
the new `Analog.names()` use it now. No wire change: the command that
should have been asked was already there.

## Open, seen once: NTC frozen at 0x9C00 on the first read after a reset

2026-08-27. The first acquisition after `build_and_flash` returned NTC as
**39936.0 exactly - 0x9C00 - with lowest == highest over 1557 samples**,
while Phase U on the same task varied normally (1419..1498). The settled
value on that channel is about 39820.

Does not reproduce. Three configurations tried straight afterwards -
`accumulate` 8 and 1, `digital` on and off - all gave a normal 20..40 LSB
spread matching the meter to within 15 codes. What was different about the
one that froze: it was the first task after a reset, with AFE_ON switched
on 0.3 s earlier.

Recorded rather than chased, because a channel reporting one exact value
with no spread is the shape of a plausible fake and this codebase has an
invariant about those. If it comes back, the thing to check is whether the
converter is being read before its reference has settled - the AFE powers
that reference, not just the signal path.

## Closed: the gate drivers trips were the bench supply's limit

Provoked deliberately 2026-08-27 with the limit raised to 200 mA at 24 V,
and it would not reproduce. Duty swept 0 to 100 % in six combinations -
AFE on and off, with and without the DAQ, with and without the synced
triple - and every one went clean.

So the two trips below were the supply's current limit being too tight for
the board plus its switching, not a fault. About 200 mA at 24 V is roughly
what it draws running.

Ruled out on the way: the guess that toggling six gate driver inputs into
an unpowered stage was forward-biasing their ESD diodes. If that were it,
AFE on - which leaves the drivers unpowered on this board - would still
trip. It does not.

What follows is kept because it is what was seen at the time.

### The two trips, as recorded

Happened twice, 2026-08-27, and was not understood at the time.

| run | duty | AFE_ON | outcome |
|---|---|---|---|
| first | 50 % | on | board power-cycled mid-sweep |
| second | 25 % | on | board went silent, then would not stay up |

Both with the STO break bypassed and equal duty on all three phases. Equal
duty puts **no voltage between the legs**, so no phase current can flow
whatever is connected - which is why it was chosen. Something else drew the
current.

What is ruled out:

* **Not a crash.** `RCC_RSR` showed no software and no watchdog reset, and
  in the earlier link-starvation case `s_keepalive` read over SWD proved
  the main loop was still turning. This is a supply event.
* **Not the boot state.** `MX_TIM1_Init` leaves MOE clear and
  `Board_PwmInit` sets CCxE with OSSI, so all six outputs are driven to
  their idle level - both FETs of every leg off - and the bypass is off at
  boot, verified on silicon.
* **Not obviously the drivers being powered.** Both runs had `AFE_ON` high,
  which on this bench board is the state that leaves the gate drivers
  *unpowered* (the inversion recorded below). If that holds, nothing in the
  power stage could have switched at all.

So either the inversion is not clean, or the current came from somewhere
that is not the gate drivers. Recovering needed the bench supply's limit raised -
about 200 mA at 24 V, which is roughly what the board draws running, so the
limit was tight rather than the board being damaged.

**Until this is understood, `bypass_break` is arming a power stage and not
a configuration flag.** `python_examples/daq_session.py` has the gate drivers off
by default for that reason.

## Broadcast beats a round trip for clock sync, by 7x

Measured 2026-08-27 on the debug probe's VCP:

| method | uncertainty |
|---|---|
| broadcast latch, host brackets the write | **5 243 us** |
| round trip, min of 20 | 35 883 us, ~17 941 us one way |

A 16-byte reply is 1.7 ms of line time. The rest is the VCP driver's
latency timer, and a broadcast never waits for it because there is no reply
to wait for. The round-trip method looks better on paper - it measures the
delay it is correcting for - and loses anyway. `clock.probe()` is kept so
the two can be compared on a segment with a different driver rather than
assumed.

Two other things the sync had to get right:

* **The settle was inside the bracket.** `transport.broadcast` slept 50 ms
  after transmitting so the slaves had acted before the next request, and
  the first version of the bracket measured all of it: +/- 55 521 us. It is
  a parameter now, and the caller sleeps outside its own measurement.
* **CYCCNT wraps every 9.04 s at 475 MHz.** Any capture longer than that
  comes back folded - twelve seconds looks like nine and three, out of
  order - so `clock.unwrap()` is not optional.

The rate is measured rather than taken from `sysclk_hz`. The **+6.3 ppm**
first recorded here was against this PC's wall clock over 3 s, and both
halves of that were wrong - see *This PC is not a clock reference* below.
Corrected against UTC over a window long enough to resolve it, the board
runs **-11.62 ppm**, floor 1.11 ppm.

## AFE_ON off stops the task and empties the buffers

That pin powers the ADC's reference, so with it off every channel reads
exact mid-scale (invariant 9). An accumulator holding half a window of real
samples and half a window of mid-scale divides out to something entirely
plausible, and no field in the reply would say so. So the task stops, the
ring and the accumulator are cleared, and a flag says why. Measured: 863
additions before, `running=False lost_power=True` and a `latest()` of
`None` after.

A stopped task stays stopped. Turning the supply back on does not restart
it, because nothing downstream would have noticed the gap.

**Sleep and the keepalive cannot both happen.** `__WFI()` wakes on the next
interrupt and the only periodic one here is SysTick at 1 kHz; the keepalive
needs an edge every 5 us. The shortest sleep available is 2-5x the STO
latch's hold. The loop is not idle-waiting either - 8.6 us a turn with the
AFE off, spent on the keepalive's own rate limiter. Downclocking is not a
knob but a re-derivation: SysTick, the UART divisors, the ADC clock and
TIM1's ARR all hang off it. The way that would open is a keepalive
conditional on somebody intending to arm the gate drivers, which changes what the
dead-man's switch proves and is not a decision to take quietly.

## The accumulator needs a count per channel

Measured 2026-08-27, seven channels over half a second with the sample loop
unthrottled: additions came back **1044 / 1043 / 1043 / 1043 / 1044 / 1044 /
1044**. `Board_DaqPoll` reads one channel per turn of the main loop, so a
take lands mid-sweep and six of the seven would have been divided by the
wrong number had there been a single count.

Means then track the meter where the channel is quiet - NTC 40466.9 against
40448.5, DC bus 20767.5 against 20760.8 - and do not where it is not:
Clevel read 32085 against the meter's 1259 and Cinj 20630 against 15350,
which is the aliasing already recorded below, seen from a second sampling
rate.

Unthrottled the loop manages **14 610 conversions per second** across seven
channels. `interval_us` now gates record production rather than sampling:
the ring is a capture and its rate is the link's business, the
accumulator's is not.

## A free-running acquisition task takes the link down

Measured 2026-08-27, and the mechanism is not the one it looks like. A
software-clocked task reading seven channels put about **190 us** of
converter work into every turn of the main loop. RTU discards a frame whose
characters arrive more than **t1.5 = 143 us** apart at 115200, so every
request was thrown away and the board went silent.

It was not a crash. `s_keepalive` read over SWD while the board was mute:
0x1C2EF5 to 0x1D4978 in two seconds, 72 835 edges - the main loop was
turning normally throughout.

| channels | loop turn | link |
|---|---|---|
| 1 | 59 us | answers |
| 3 | ~120 us | dies after four requests |
| 7 | ~190 us | dies |

**Rate limiting alone did not fix it.** At 200 Hz it survived and at 1000 Hz
it did not, because one poll still exceeded t1.5 whenever it landed inside a
frame. The fix is that `Board_DaqPoll` now reads **one channel per turn**,
assembling a record across several - which costs nothing in simultaneity,
since a software clock reads the channels one after another regardless - and
bounds the per-turn cost to a single conversion. With that, every rate from
200 Hz to unlimited answers 12 requests out of 12.

## What the link can actually carry

| task | stride | rec/s | payload |
|---|---|---|---|
| 7 ch + digital | 36 | 96 | 3460 B/s |
| 3 phases | 16 | 239 | 3828 B/s |
| NTC alone | 8 | 480 | 3838 B/s |

**About 3.8 kB/s whatever the record size**, against 11.52 kB/s of raw line
rate at 115200. Records per second scale inversely with stride and nothing
else. Reading in bigger blocks does not help - one read already fills a PDU.
The only thing that does is producing fewer records: seven channels and the
digital word drop 3851 at `accumulate` 1 and **none at 16**.

The board now works its own ceiling out from the stride and the baud, and
substitutes it when a free-running task asks for no rate. Running free it
delivered 88 rec/s against the 105 predicted, with zero drops - conservative
in the right direction.

**The ceiling is on records, and a record is `decimate` x `accumulate`
triggers.** Gating the triggers at the record rate would have sampled
sixteen times slower at `accumulate` 16 instead of averaging sixteen
samples: the same output rate with every sample but one thrown away. Fixed,
and then measured with zero drops throughout:

| task | accumulate | rec/s | samples/s |
|---|---|---|---|
| 1 channel | 1 / 16 / 64 | 376 / 294 / 182 | 376 / **4701** / **11614** |
| 7 channels | 1 / 16 / 64 | 96 / 89 / 29.5 | 96 / **1422** / 1886 |
| 7 + digital | 1 / 16 / 64 | 93.5 / 79.4 / 28.9 | 93.5 / 1271 / 1850 |

Sixteen-fold averaging costs **7 %** of the output rate on seven channels.
Where samples/s stops climbing is the board's own limit rather than the
link's: about **11.6 kHz on one channel, 13.2 k conversions/s in total**.
That is the converter and the main loop, and it leaves the H753 room to
spend on precision instead of bandwidth.

## The keepalive, after pumping every busy-wait

`Board_StoKeepalive` is now rate limited to one edge per 5 us - 200 kHz of
edges, the 100 kHz square wave the model drives `MCU_PWM` with - so the call
can be dropped into any spin without it running at the loop's own megahertz.
Three places now call it beyond the top of main(): both sensors' chip-select
`settle()`, and a register-level transmit in `dev_uart.c` that replaced
`HAL_UART_Transmit`.

| | edges | square | **worst gap** |
|---|---|---|---|
| both sensors held | 111 646 Hz | 55.8 kHz | **163.1 us** |
| both polling | 73 166 Hz | 36.6 kHz | **162.7 us** |
| IMU streaming + angle | 72 462 Hz | 36.2 kHz | **162.3 us** |

**5116 us to 163 us**, and for the first time under the latch's simulated
200-400 us hold. The rate is short of 100 kHz because the loop still blocks
in SPI: measured by holding each sensor in turn, the **A1335 costs 42 us per
iteration and the IMU costs 0.5** - the mean was never the IMU's, and the
chunking that fixed its worst case did nothing for the rate.

## The measurement ring

`board_log.c`, 1024 records of 16 bytes in DTCM. Producers are the injected
ADC interrupt at 50 kHz and the two main-loop sensor poll loops; the
consumer is the command layer, also in main(). Only the ISR can preempt, so
the critical section is PRIMASK - there is no RTOS and a mutex would be a
scheduler this board does not have.

Measured, phases captured from the ISR: **19.81 / 20.00 / 20.14 us
min/mean/max** against 50 kHz's 20.00, sequence numbers unbroken. Angle
lands at 55.8 us, dead regular.

It is a snapshot buffer, not a stream. At 50 kHz the ring is 20 ms of
history and a host draining fifteen per round trip manages about 3000/s
against 50000/s produced, so `dropped` counts the rest - 3026 in a 30 ms
window, which is the arithmetic working rather than a fault.

## The keepalive's worst gap is the Modbus reply, not the IMU

Measured 2026-08-27 with a worst-gap counter in `Board_StoKeepalive` (raw
CYCCNT, `0x6E` device 4, op 7 resets it). The mean rate hides all of this:
it barely moves between any two rows below.

| Condition | worst gap |
|---|---|
| 5 s with no Modbus traffic | **476.1 us** |
| 86 smallest-possible replies (1 byte) | 476.1 us |
| 86 state replies (53 bytes) | **5117.2 us** |

53 bytes at 115200 8N1 is 4.60 ms; the measured excess over the floor is
5117 - 476 = **4641 us**. **The main loop blocks while it transmits.** The
stall scales with reply length and has nothing to do with the sensors - it
is identical with `AFE_ON` low, where `Board_ImuPoll` returns immediately.

This overturns the earlier reading. The 1.5 ms SHTP cargo was real
arithmetic but never the dominant term; asking the board a question costs
three times more than the IMU does.

**The IMU cargo is fixed and no longer visible.** `Board_ImuRead` now
transfers in 8-byte chunks with an edge between each - 43 us at 1.48 MHz,
against the 52 us the loop already takes per iteration. Chip select is
`NSS_SOFT` and is not touched between chunks, so the part still sees one
transaction; releasing it is the `FF FF FF FF` bug. Verified streaming at
10 ms: **1524 updates, 0 errors, quaternion norm 1.000009**, and the worst
gap in a quiet window stays at 476.1 us whether the IMU streams or not,
where an unchunked 320-byte cargo would be 1730 us on its own.

**It is still not enough.** The latch holds 200-400 us (simulated), and the
476 us floor alone exceeds that before anything talks to the board. What
produces that floor is not yet identified - it is bit-exact across every
condition tried, which suggests one periodic thing rather than jitter.

## The bypass is off at boot, and cannot be otherwise

`MX_TIM1_Init` sets `BreakState = TIM_BREAK_ENABLE`. The only writes to
`BDTR.BKE` anywhere are the two inside `Board_PwmSetBreakBypass`, and
nothing calls it at startup. Verified on silicon straight off a reset:
`break_bypassed = False`, `fault = False`.

## The bench board's AFE gate is inverted, and it costs the measurement

Told by the user 2026-08-27 and consistent with everything measured: on this
board variant the gate drivers have supply **while `AFE_ON` is off**, not
while it is on. A hardware bug, to be patched by hand.

The consequence is the awkward one. `AFE_ON` also powers the ADC reference
(invariant 9), so:

| AFE_ON | gate drivers | current measurement |
|---|---|---|
| off | **powered** - the gate drivers can switch | meaningless: every channel reads mid-scale |
| on | unpowered - nothing switches | valid |

**Switching and measuring are mutually exclusive on this board.** Nothing
below was taken with a live power stage, and nothing can be until the patch.

## The current path is consistent, with the gate drivers inert

AFE on, break bypassed, gate drivers enabled, sync armed. Duty swept 0-100 % equal
on all three phases, and the sample point swept across the period:

| Swept | Phase U | Phase V | Phase W |
|---|---|---|---|
| duty, 0-100 % | means within 31 LSB | within 43 | within 34 |
| CCR4, 60..2360 | within 41 LSB | within 18 | within 34 |

All inside one standard deviation, zero overruns throughout, DC bus flat to
+/-0.01 %. Scaled, 8 blocks of 64 samples:

| Channel | value | sd | peak-peak | drift over 8 s |
|---|---|---|---|---|
| Phase U | +2.449 A | 0.412 A | 1.766 A | 0.117 A |
| Phase V | -57.178 A | 0.354 A | 1.576 A | 0.273 A |
| Phase W | -3.867 A | 0.412 A | 2.013 A | 0.210 A |
| DC bus | 24.7704 V | 3.1 mV | 13.1 mV | 3.1 mV |
| NTC | 38.798 C | 6.2 mK | 29.9 mK | 34.4 mK |

The absolute phase figures are offsets, not currents - nothing is connected,
Phase V carries the known bad op-amp, and the boot calibration runs against
an unpowered input with about 100 mV of boot-to-boot spread, which is +/-6 A
here. **The useful number is the noise floor: 0.35-0.41 A RMS, 1.6-2.0 A
peak-to-peak, 0.2 % of the 207.4 A full scale**, and it does not move with
duty or sample point. DC bus noise is 125 ppm.

Unexplained and worth reproducing before it is believed: an earlier run
tripped something at 50 % duty and power-cycled the board - `RCC_RSR` showed
no software or watchdog reset, so it was a power event. But that run also
had `AFE_ON` high, which by the table above means the drivers were
unpowered and nothing could have drawn current. Either the inversion is not
clean or the trip had another cause. A later run swept 0-100 % under the
same conditions without incident.

## CCR4 = ARR stops the trigger, exactly like CCR4 = 0

Measured: `updates` +52149 per second at CCR4 2360, **+0** at 2375 and **+0**
at 0. The latched triple then freezes, and 60 reads of a frozen value return
sd = 0.0 - which reads like a perfectly quiet channel and is not one. Both
ends of the range are degenerate; only one of them was documented.

## Open: Cinj and Clevel cannot be sampled asynchronously

Apparent duty falls monotonically with sample rate, which a fixed-duty
waveform cannot do:

| sample rate | Clevel apparent duty | Cinj |
|---|---|---|
| 68 kHz | 2.1 % | 31.3 % |
| 20 kHz | 0.8 % | 14.2 % |
| 5 kHz | 0.5 % | 3.9 % |
| 1 kHz | 0.4 % | 1.0 % |
| 200 Hz | 0.3 % | 0.4 % |

Either the signal is far faster than 68 kHz, or the sample-and-hold is
disturbing a high-impedance node - `ADC_SAMPLETIME_1CYCLE_5` is 20 ns and
Clevel sits on PB1, the channel already noted as unfiltered. Not settled.
Until it is, **no mean of either channel is a measurement**; take them
through the injected group or with a longer sampling time.

* **TIM1 latches a break during its own init.** Measured 2026-08-27: after
  `MX_TIM1_Init`, `TIM1->SR` bit 7 (BIF) was set while `PE15` read 1. The
  pin is AF open-drain with `GPIO_NOPULL` and floats while
  `HAL_TIMEx_ConfigBreakInput` enables BKE, so the break trips on start-up
  rather than on a fault. `Board_PwmInit()` now clears BIF after the pin has
  settled; a pin genuinely low latches it straight back. Confirmed after the
  fix: `TIM1->SR` = 0x1, UIF only.

## This PC is not a clock reference: 947 ms out and 25 ppm slow

Measured 2026-08-27, six minutes after W32Time had reported a good sync
(`LastKnownGoodTime` 13:35:06 UTC):

| | |
|---|---|
| offset from UTC | **+947 ms**, and growing |
| rate against UTC | **+25.3 ppm slow**, fitted over 121 s, floor 8.3 ppm |
| W32Time | `Stopped`, `Manual`, trigger-start |
| slewing back? | no - the offset grew across all nine samples |

Two servers agree - `time.windows.com` +937.325 ms, `time.google.com`
+937.788 ms. `pool.ntp.org` times out on this bench and is not the default.

`Stopped` proves nothing on Windows 11: the service is trigger-started, so
it syncs and stops again, and the first reading of this called a good sync
a failed one. What proves it is the offset. Windows declined to step 947 ms
because that is inside the 1 s `MaxAllowedPhaseOffset`, and the slew it
took instead is not catching up.

So `clock.sync()` defaults to `reference='utc'`: an SNTP offset at each end
of its own window, the PC's offset out of the epoch and the PC's rate out
of the frequency. With no network it falls back to the PC clock and says so
in `Sync.note` - a capture that believes it is on UTC when it is not is
worse than one that knows it is not.

The board, over 900 s with the correction in:

| | measured | floor |
|---|---|---|
| SYSCLK | **474.994 MHz** against 475.000 nominal | |
| board vs UTC | **-11.62 ppm** | 1.11 ppm |
| this PC vs UTC | +21.82 ppm slow | 1.11 ppm |

Ten times the floor, so measured rather than bounded. The three methods now
agree: the heartbeat's -13 ppm, a 60 s window's -13.57, and this.

### The correction was signed backwards, and an old number caught it

The first cut multiplied by `(1 + pc_ppm)` and put the board at **+35.3
ppm**. The PC under-counts, so cycles divided by its short elapsed makes
the board look fast and the correction has to divide, not multiply. What
flagged it was the heartbeat's independent **-13 ppm** from an earlier
method: +35.3 is not near it, -13.57 is.

Proved afterwards against the stand-in, whose oscillator is -12.0 ppm off
this machine by construction. With the PC measured at +17.61 ppm slow it
came out at **-29.61 ppm** against UTC, expected -29.61, residual 0.00.

### A sync window has to outlive the wrap to mean anything

`sync()` took two brackets `seconds` apart and refused a gap longer than
CYCCNT's 9.04 s. With a 5 ms bracket over 3 s that bounds the rate at parts
per **thousand** - the +6.3 ppm it used to print was three orders finer
than the method could resolve. It now samples through the window and
unwraps, so the window can be as long as the floor needs:

| window | floor |
|---|---|
| 3 s | 1 700 ppm |
| 60 s | 16.5 ppm |
| 900 s | 1.1 ppm |

`floor_ppm` is on every `Sync`, and `clock_drift.py` prints `bounded, not
measured` when the answer is under it.

## Two thirds of a round trip was the host waiting on itself

Measured 2026-08-27 on the debug probe's VCP at 115200. Every command cost
the same 46 ms - `clock.read_latch` 45.5, `imu.state` 46.0 - which is what
said it was not the board.

Where a 20-byte reply's 46.6 ms went, and none of it is the 1.7 ms of line
time it contains:

| | ms | what it was |
|---|---|---|
| `INTERFRAME_GAP` | 5.0 | a flat sleep; the specification says 1.75 above 19200 baud |
| `serial.timeout = x` | **9.8** | 3.25 ms each, three times a transaction |
| first byte + stream | ~5 | the link, doing its job |
| `QUIET_TIME` tail | **20.0** | waiting to be sure the frame had ended |

**Assigning `serial.timeout` costs 3.25 ms whatever it is assigned to** -
pyserial reconfigures the port, which on a USB VCP is a control transfer -
and it cost that even when set to the value it already held. The receiver
now never moves it: the port sits at `QUIET_TIME` for its whole life and the
budget for a late reply is counted by the reader, in slices.

The tail was six times what the link needs. Reading greedily with
`in_waiting`, the largest gap **inside** a frame is 3.40 ms, over both a
20-byte reply that arrives whole in one chunk and a 215-byte one that
arrives in 175. It is 8 ms now.

**46.6 ms -> 15.5 ms.** A polled view was round-trip bound at 21 frames a
second and is not any more.

Two things that looked like findings and were not:

* **A mid-frame stall of 17.8 ms.** It was `serial.read(1)` per byte - one
  driver round trip each. Reading what `in_waiting` reports makes it 3.3 ms.
* **A 2 % rate of unanswered requests.** It was the ring, armed and
  flooding, left behind by a script that had crashed. With the ring idle:
  750 transactions, no silence, and the board's own worst main-loop gap is
  58.9 us against RTU's t1.5 of 143.

Rejected: stopping the read on a valid CRC instead of on a gap. A prefix of
a 20-byte frame passes a 16-bit check about once in 4096, which is a wrong
reading every few minutes rather than an error, and nothing in the frame
says where it ends.

## One ring, two producers, and the fast one locked the slow one out

Measured 2026-08-27, `capture.arm(['angle', 'imu'])` at 115200:

| | before | after |
|---|---|---|
| angle reaching the host | 198 /s | 129 /s (its share) |
| **IMU reaching the host** | **1 /s** | its full rate |
| dropped, in 1.5 s | **77 733** | **0** |

The angle loop pushes on every successful SPI read - about 24 000 a second -
and the ring is 1024 deep and drops the newest when full. It filled in 43 ms
and every IMU report after that was refused. Nothing was broken: a shared
FIFO with drop-newest gives the whole ring to whichever producer is fastest.

Each armed source now gets an equal share of `cmd_link_records_per_second`,
enforced as a minimum gap in raw CYCCNT (invariant 2). A source under its
share never reaches the check, which is why the IMU's 50 Hz is untouched and
the angle loop's 24 kHz is not. `thinned` counts what the limit refused and
is reported apart from `dropped`, because they mean opposite things: dropped
is a sample the ring had no room for, thinned is one the link could not have
carried anyway.

It also took 24 000 PRIMASK sections a second out of the main loop.

## A DAQ record is a code; the unit in its layout is not

The acquisition task buffers converter codes and does not scale them. The
layout reports the channel's own unit, which says what the channel *means*.
The capture view printed the two together:

| shown | actually |
|---|---|
| `NTC +40470 centi-degC` | 40470 is the code; `ntc_temperature()` reads **38.1 C** from it |
| `DC bus +20811 mV` | **24.81 V** |

Both wrong by orders, and both looked like readings. It shows the code and
what it converts to now, through `scaling.converter` - which is where the
unit-to-conversion mapping moved, because the meter bridge had the only copy
and a second one in the capture view is the one that goes stale (invariant
7). The two views agree channel for channel: NTC 37.45 against 37.5, DC bus
24.82 against 24.8, Phase U +9.31 A against +9.4.

The same view also showed sums as readings - a record's value is the SUM of
`samples` - so everything doubled the moment its own backpressure raised
accumulation from 1 to 2.

And it was the only one of the four views with no `except RigError` in its
frame loop, which is why it was the one that died on a missed reply.

## What the link actually carries, against what its bitrate allows

`tools/link_bench.py`, 2026-08-27 at 115200 over the debug probe's VCP,
after the transport fix above. The floor is `bytes * 10 / baud` for 8N1.

| case | wire | floor | median | of max | payload |
|---|---|---|---|---|---|
| ping (echo, no payload) | 8 B | 0.69 ms | 15.5 ms | **4.5 %** | - |
| echo 16 B | 40 B | 3.47 ms | 15.6 ms | 22.3 % | 2.1 kB/s |
| echo 64 B | 136 B | 11.81 ms | 31.0 ms | 38.0 % | 4.1 kB/s |
| echo 250 B | 508 B | 44.10 ms | 49.1 ms | **89.8 %** | **10.2 kB/s** |
| ring burst, 15 records | 222 B | 19.27 ms | 46.5 ms | 41.5 % | 4.6 kB/s |

**The cost of a transaction is flat at about 5 ms**, so the whole curve is
that one number amortised. A ping is nearly all overhead by construction and
says nothing about a link; a full block is where the bitrate starts being
the limit.

This retires the 3.8 kB/s that `DAQ_LINK_SHARE_PCT` was set from - that was
measured through the 46.6 ms transport, and the same wire now carries
10.2 kB/s. The share is still 33 %, deliberately: it is the fraction a
stream may claim of a segment that also carries everything else.

## Fixed: the IMU produced reports and the poll loop never collected them

2026-08-27. The part had never stopped working. One real defect, and three
measurements that pointed the wrong way; both are recorded below because the
measurement errors cost more time than the defect did.

**The defect.** `Board_ImuPoll` returned every turn unless H_INTN was
asserted at the instant it looked - one GPIO read, no wait. That is correct
and cheap when the pulse is caught and useless when it is not, and it left
the part streaming rotation vectors at 50 Hz into a loop that never read
one. It now polls a header when H_INTN has not asserted, rate limited to
1 kHz in raw CYCCNT: a four-byte transfer at 1.48 MHz is 27 us, which is
2.7 % of the loop against a report interval of 20 ms.

**Then a reset before a Set Feature stops the feature taking.** Measured
either way, three seconds each:

| sequence | rotation vectors |
|---|---|
| `reset()` then `feature()` | **0 a second** |
| `feature()` alone | **49.0 a second** |

The write's own wake handshake runs a reset when the acknowledge does not
arrive, so a Set Feature sent straight after a host reset lands on a part
that has just restarted. The loop brings the part up by itself, so the
reset was never needed: both views drop it. Draining until the part is
quiet - three empty reads rather than one, since the 276-byte
advertisement arrives with gaps - did not fix this on its own and is kept
because stopping at the first gap was wrong anyway.

### Three measurements that pointed the wrong way, and what each needed

* **`product_id` and `imu.pins()` answering SERVER DEVICE FAILURE**, read as
  a dead part. They were refused by `cmd_imu_op`, which gates every op except
  LATEST, HOLD and RESUME on the poll loop being HELD - the calls were outside
  `board.imu.configuring()`. **Needed:** hold the loop before driving the bus.
* **"H_INTN never asserts": 77 reads of PD8, all high.** Retracted. Each read
  was a Modbus round trip 15 ms apart and an H_INTN pulse is microseconds, so
  the measurement could not have caught one either way. `feature()` alone
  works, which needs the wake acknowledge, so the line does assert. **Needed:**
  sample faster than the event, or infer it from something that depends on it.
  The polled fallback stays because it is bounded and it is what the file
  always described.
* **Gating the read twice.** The first fix put `poll_due()` inside
  `Board_ImuRead` as well as the loop, so the loop spent the rate limit's slot
  and the read consumed the next one. Neither ever read. **Fixed:** one gate,
  in the loop, where the rate is decided.

Confirmed from the schematic while chasing this, MCU sheet: `SPI0.INT` is a
straight wire to **PD8** and `SPI0.SYNC` to PD9. `board_imu.c`'s file header
said "neither is assigned to a pin", contradicting its own line 55; **fixed
2026-08-28** - the header now states the pins and that the line does assert.

## The gate drivers switches 0 to 100 % with the drivers powered, and nothing trips

Measured 2026-08-27, AFE_ON **off** so the bench board's inverted gate gives
the drivers supply, break bypassed, all three legs at the same duty - so
there is no voltage between phases and no phase current, only gate charge
and whatever the legs draw from the DC link.

| duty | 1 | 2 | 5 | 10 | 25 | 50 | 60 | 75 | 90 | 98 | 100 % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| link | up | up | up | up | up | up | up | up | up | up | up |

Zero overruns throughout, and five seconds held at 50 % with the link up.

**25 % used to take the board down.** The supply is the difference: raised
to 200 mA at 24 V, as the user said at the time. It closes *the gate drivers trips
were the bench supply's limit* - they were, and the limit moved.

Nothing here was measured with an instrument, and nothing here is a phase
current: with all three legs in phase the motor sees zero volts. What it
proves is that the switching itself is clean at every duty, which is what
had to be true before commutation is worth writing.

### The dead time, checked three ways before any of it

The 2EDL8034 has **no interlock** - the datasheet is explicit that the
inputs are independent - so TIM1's dead time is the only thing between the
two FETs of a leg.

| | |
|---|---|
| `.ioc` | `TIM1.DeadTime=19` |
| board's own report | `gate drivers.state()['deadtime']` = 19, read from `TIM1->BDTR & TIM_BDTR_DTG` |
| **silicon, over SWD** | `BDTR = 0x02001C13` -> DTG **19**; `CR1 = 0xB1` -> CKD **00**, so t_DTS = t_CK_INT |

PSC 0 and ARR 2375 at 237.5 MHz, so **19 x 4.2105 ns = 80.0 ns**. The same
read confirmed CCER 0x555 - all six outputs enabled, no inverted polarity -
BKE 1, BKP 0, AOE 0 and MOE 0 at rest.

Against that, what the gate needs: 15.5 V down through 4.99 + 2.2 ohms into
5.48 nF to the 2.8 V threshold is 1.71 time constants of 39.4 ns, about
67 ns; the incoming device reaches its own threshold 8 ns after its edge;
the driver's worst-case delay matching is 6 ns. **About 65 ns needed, 80 ns
present** - and the user's own LTspice half-gate drivers runs at `tdead=30n`.

`rig.gates.check()` re-reads DTG on every arm and refuses at zero, because
a `.ioc` regeneration and a CubeMX mode name bound to the wrong channel have
both silently moved TIM1 in this repository before.

## Open, seen once: the board stops answering Modbus while the core runs

2026-08-27, during a full suite run. The MCU was alive - `STM32_Programmer_CLI
-c port=SWD mode=HotPlug` read `Device ID 0x450` - and COM4 existed, but no
Modbus request got a reply: `open_session()` fell back to the stand-in, so
`test_parity` reported 0 checks and `test_conformance` 1. A re-flash with
`--start` brought it straight back.

The shape is invariant 5's: a latched ORE ends reception until ICR clears it.
`dev_uart.c` does clear it - `DEV_ERR_CLEAR` includes `USART_ICR_ORECF`, and
the comment beside it records why reading RDR is not enough - so either that
path was not reached or this is something else.

Not reproduced. What ran just before it: the full suite twice over, the two
python_examples, and the gate driver view. Nothing in the state afterwards
pointed anywhere - the task was stopped, the ring disarmed, MOE clear.

The run before this one failed differently for what may be the same reason:
`test_mcp` lost three checks, two of them AFE ones, while every other suite
passed. Recorded together because a board that answers intermittently looks
like a different bug in every suite that meets it.

## The gate snapshot is one instant, and averaging it lies when the sync is armed

Measured 2026-08-27, all three legs at 50 % duty, CCR 1187 of ARR 2375:

| | high side reads | CNT median |
|---|---|---|
| sync disarmed | 50.8 %, 53.5 % | 1155, 1075 |
| **sync armed** | **89.5 %** | **387** |

The snapshot reports `GPIOE->IDR` and `TIM1->CNT` together, which is exactly
right for "what are the six signals doing now". It is not a duty
measurement, and with the sync armed it is not even an unbiased sample: the
injected conversion fires at CCR5 near the top of the period, its handler
runs, and the Modbus reply is served a fixed distance behind it - so CNT
lands in the same narrow band every time. 387 against a compare of 1187 is
the high side on, and 89.5 % is what that looks like averaged.

The per-leg symmetry measurements in this session were taken with the sync
disarmed and stand - 600 samples, CNT median 1188, all three legs 50.0 %. That
held by luck rather than by design, so the rule is written down: **anything
that averages the snapshot has to check `pins_at` is spread across the period,
or disarm the sync first.**

Two earlier readings fell into the same trap: 42 / 51 / 43 % across the three
legs at 80 samples, read as a difference when 1 sigma was 5.6 %; and 86.5 %
straight after an arm, read as the waveform having changed.

## The six gate signals were writable through the GPIO test path

Found 2026-08-27 while hunting an asymmetry between the three legs.

`Board_PinUsable` walks `s_digital` and, for a pin that is not in it,
returns true - "nothing on this board claims it, so a fixture may have it".
**PE8..PE13 were not in it.** The table held AFE_ON, nFAULT, UART5_TERM,
KEEPALIVE and the buses; the gate signals were in neither that table nor the
reserved list, which reported 19 pins and not one of them a gate.

So `gpio_pin` write on PE12 was allowed. It calls `HAL_GPIO_Init` on the
pin, which takes it off TIM1 and leaves it driven by ODR - one FET of a half
bridge latched on, with the other still switching against it. **The dead
time cannot help there**, because the pin is no longer the timer's to
sequence.

Fixed by claiming all six in `s_digital` with `usable` false. The reserved
list reports 25 pins now, and a raw wire request past the host's own guard
is refused by the board:

    PE8   board refused: ModbusException
    PE12  board refused: ModbusException
    PE13  board refused: ModbusException
    gates after: 000000

Nothing in this repository was writing them, so this is not the cause of the
15 C between the legs. It is a hole that was one stray `gpio_pin` from
latching a half bridge, and the default that opened it - "not in the table,
so a fixture may have it" - is right for a spare pin and wrong for every pin
an alternate function owns.

## What the intermittent silence is NOT

The board stops answering Modbus now and again while the core keeps running -
`unit 1, fc 0x??: silence` past twenty retries, three times in one afternoon,
recovered once by a re-flash and otherwise by itself. Not reproduced. What
was tried, and the numbers that ruled each one out:

| hypothesis | test | result |
|---|---|---|
| AFE_ON transitions stall the loop | 60 reads in each state, six toggles between | **0/60** silence throughout, worst main-loop gap 57.9 us |
| main loop starved past RTU's t1.5 | worst keepalive gap, every condition | **57.6 - 58.0 us** against 143 us |
| switching noise couples into the UART | 120 reads armed at 50 %, drivers dead then live, twice | **0/120** each |
| the port sleeps through a long quiet | 30 s and 60 s silent with the drivers live and switching | answered on **attempt 1** every time |

600 requests across every condition and not one failure. Whatever it is, it
is rarer than that and does not follow switching, the supply gate, loop load
or idle time.

What it costs is long runs: three thermal series died on it, one of them six
minutes in. Anything that sleeps between requests needs a bounded retry -
`blocks()` and `Board.probe()` carry one, ad-hoc scripts have to bring their
own.

## The W leg did not switch: its two gate pins were one node

**Root cause: the series resistor array on the gate lines has too tight a
footprint**, so two channels of the same package bridge - which ties PWMWH to
PWMWL. That is why both boards had it: same footprint, same assembly, same
bridge. Reworked 2026-08-28 by wiring over the pads; new arrays on order.

What the board reported before and after, from the probe that found it:

| | before | after |
|---|---|---|
| `gate_shorts` | `('W',)` | `(none)` |
| W low / high / both high | 99.13 / 99.13 / **99.13 %** | 51.6 / 47.0 / **0.0 %** |
| U and V, same test | 49.5 / 49.6 / 0.0 % | 51.6 / 47.0 / 0.0 % |

CNT spread 1..2373 of ARR 2375 across the samples, so that is not the
aliasing trap. Thermally, 20 s at 50 % on all three legs moved the NTC
**29.52 -> 33.23 C, +3.71 C**, where W had previously contributed nothing.

A per-leg repeat of the old `U +1.400 / V +4.582 / W +0.000` was started and
is not done: the first attempt read the board cooling from the run before it
- baselines climbing 33.12 -> 34.01 -> 36.34 and W scoring -0.371 C - and
the settled-baseline rerun stopped when the board lost power (ST-LINK target
voltage 0.00 V) for further rework.

The rest of this entry is what the hunt ruled out, kept because the wrong
turns are the part worth not repeating.



U and V run warm, W stays at ambient, its phase node sits at ~10 V. The
board detects it now - `0x6D` gate driver state carries `gate_shorts`, and
`gates.check()` refuses to arm a leg that reports one:

    gate_shorts: ('W',)

**The timer is not why.** TIM1_CH3 is also on PA10 and TIM1_CH3N on PB15,
neither near the W leg. Same channel, same run, same instants:

| CCR3 | PA10 (CH3) | PB15 (CH3N) | PE13 (CH3) | PE12 (CH3N) |
|---|---|---|---|---|
| 0 | 0 | 1 | **1** | **1** |
| ARR/2 | 1 | 0 | **1** | **1** |
| ARR | 1 | 0 | **1** | **1** |

The pair is correct on the spare pins and stuck high on the W pins at the
same moment. `HAL_TIM_PWM_Start` + `HAL_TIMEx_PWMN_Start` write byte-identical
registers to the manual CCER/MOE writes and behave identically, so that is
not it either.

**How fast the coupling is, which is what settles its size.** Drive one pin,
poll the neighbour, count DWT cycles at 475 MHz:

| pair | rising edge | falling edge |
|---|---|---|
| W, PE12 -> PE13 | 155 cycles | 155 cycles |
| U, PE8 -> PE9 | **never** (5e6 polls) | 119 (loop overhead) |
| V, PE10 -> PE11 | **never** | 119 |

W follows within ~36 cycles of the loop's own overhead - about 76 ns. A few
hundred k into the pin capacitance would take microseconds, so the path is
of order 10 k or less while the board is biased. A meter reads 390 k cold,
on both boards, which is what two internal input pull-downs in series
through RW1/RW2 look like and is probably normal - the same reading on the U
and V pairs would confirm that.

**It is there before any PWM has run.** Probed in `SysInit`, before
`MX_GPIO_Init` and before TIM1 exists:

| pair | before any PWM | after 1 s switching |
|---|---|---|
| W | **coupled** | coupled |
| U | no | no |
| V | no | no |

So switching does not create it and the firmware does not cause it during a
run. Both pins driven, 40000 samples, W reads 99.13 % both-high with 0.87 %
low - two dead-time windows per 20 us period, the node being `OC3 OR OC3N`.
U and V read 0.00 % both-high under the identical test.

Ruled out: `CCMR1 0x6868`, `CCMR2 0x0068`, `CCER 0x555`, `CR1 0xE1`,
`CR2 0x0` with every OIS bit clear, `CCR3 = ARR/2`, `OC3M[3] = 0`,
`GC5C1..3 = 0` (`board_sync.c` writes CCR5 with 16-bit values), `MODER` AF
and `OTYPER` push-pull on all six, `AFRH 0x10111111`, one `sConfigOC` for
CH1/CH2/CH3, `HAL_TIM_MspPostInit` covering all six pins, `MX_GPIO_Init` not
touching PE12/PE13, `HAL_TIM_Base_MspDeInit` never called, identical CCR3
write paths, and a `.ioc` symmetric across the three legs line for line.

**Also fixed on the way**, from ST's own `TIM_ComplementarySignals` notes:
BKIN on PE15 is active low and CubeMX generates it `AF_OD` with `GPIO_NOPULL`,
so an undriven fault line floats and the break fires on noise. `Board_PwmInit`
now sets a pull-up, so "nobody driving" means "no fault".

### Four measurements that were wrong, and the method each one needed

Every one of these was a method fault rather than a hardware surprise, and
together they were the bulk of the time this took:

* **A write and its check in separate SWD connections**, seconds apart with
  the firmware running in between and free to undo the write. **Fix:** one
  connection for the pair.
* **The stimulus never read back.** Only the sensed pin was captured, so a
  drive that did not take was recorded as "no coupling". **Fix:** read the
  driven pin in the same word.
* **A floating input used as a probe.** A few hundred k is ample to drag a
  floating CMOS input, so it cannot tell a short from a leakage path. **Fix:**
  bias the sensed pin against the drive, or drive both.
* **`Select-String ' : '` parsing a programmer's output**, which matched banner
  lines and silently produced a table of `$null` rendered as zeros. **Fix:**
  anchor on the address, and print how many words came back.

**Do not arm while the two collapse together.** Every switching edge is a
VDD-to-GND path through two GPIO output stages inside the MCU.

## PE15 was drivable, so the test path kept disconnecting the break

`TIM1_BKIN` is on PE15. It sat in `s_digital` with `usable = true`, so
`testrig_pin_config()` would happily call `HAL_GPIO_Init` on it - which takes
the pin off the alternate function and **disconnects the break from the
timer**, silently, until the next reset.

Caught by reading GPIOE after a conformance run, while looking for something
else entirely:

| register | after the suites | meaning |
|---|---|---|
| `MODER[31:30]` | `00` | PE15 is a plain input |
| `OTYPER` bit 15 | `1` | open drain, left over from the AF_OD setup |
| `PUPDR[31:30]` | `01` | pull-up, left over too |

So the pin still carried the shape of `HAL_TIM_MspPostInit`'s configuration
while no longer being TIM1's. The power stage had no hardware break and
nothing said so.

Same defect the six gate signals already carry a warning about in that file:
PE15 was outside the set when they were fixed. It is now `usable = false`, and
the fault level is still reported through
`Board_IoFault()` and the gate driver state, both of which read the pin
without reconfiguring it. Verified: `MODER` reads `10` with the pull-up and
AF1 intact after conformance and MCP both run, where it read `00` before.

Two `test_mcp.py` checks used E15 as their scratch pin and now use E14
(UART5_TERM, restorable with a `write`). The stand-in moved PE15 from
its drivable rows to its reserved ones, in the board's own order - `parity`
caught both halves of that, first as a row count and then as a position.

**Also fixed**: BKIN is active low and CubeMX generates it `AF_OD` with
`GPIO_NOPULL`, so an undriven fault line floats and the break fires on
noise - ST's `TIM_ComplementarySignals` notes warn about exactly this.
`Board_PwmInit` now sets a pull-up, so "nobody driving" means "no fault".

## The NTC cannot tell you which leg is switching

One sensor, one place. Running a single leg at 50 % for 20 s and reading the
rise looks like a per-leg measurement and is not: what it mostly reports is
how close that leg sits to the thermistor.

Measured after the W rework, with the gate-level check already showing all
three legs complementary and a thermal camera confirming W's driver
switching:

| leg alone, 20 s at 50 % | rise | earlier, W dead |
|---|---|---|
| U | +0.618 C | +1.400 C |
| V | **+2.946 C** | +4.582 C |
| W | +0.150 C | +0.000 C |

V dominates in both sets, before and after the fault was fixed, which is
placement rather than dissipation. Two further traps in the same test:

* **The baseline drifts through the sequence.** 35.36 -> 36.70 -> 37.40 even
  waiting for it to settle first, and U's settle took 294 s. A run without
  the wait had it climbing 33.12 -> 34.01 -> 36.34, which put W at
  **-0.371 C** - it was reading the board cooling from the previous leg.
* **The AFE has to be off to switch and on to read**, so the two never
  overlap and every point is taken after the fact.

What the NTC is good for is the whole board: 20 s at 50 % on all three legs
moved it 29.52 -> 33.23 C, +3.71 C. For per-leg attribution use the camera,
or the gate pins - `gate_shorts` and the both-high count answer the question
the thermal test was being asked.

## The link died and stayed dead - found, 2026-08-28

Not the intermittent silence recorded above. That one drops single requests
and the next one answers; this stopped answering **permanently** until the
core was reset, in BOTH modes - Modbus said nothing and so did the console.
SWD still answered (Device ID 0x450), so the MCU was running the whole time.

**Cause: nothing cleared the HAL's UART error state.** `dev_uart.c` clears
ORE in three places and is careful about it - the file's own banner says a
latched ORE ends reception until ICR clears it. But `Console_Poll` reads
through `HAL_UART_Receive`, and the HAL's blocking receive records the error
and returns WITHOUT touching ICR. Only ICR clears ORE.

That kills both modes at once, and the coupling is what makes it permanent:

* the console stops receiving, because ORE is latched;
* the way back to Modbus is a character typed at that console
  (`Console_Poll` opens the link on `m`), and there is no other path;
* so the binary link cannot be reopened, and nothing on the board notices.

**Two false trails, both worth keeping.** The first was the observer's new
sampling, which had just gained two SPI transactions and an ADC conversion at
810.5 cycles. Measured against it, 400 requests each way - 4 silent with
sampling off, 0 with it every 5 s. No correlation, and several borrows fell
inside that window. The second was console mode: `test_conformance.py`
deliberately writes holding register `0x0001 = 1` to leave Modbus, and the
suite aborting there would strand the board. Disproved by sending `m` by
hand - the console did not answer either, which is what pointed at the HAL.

**Fix:** `Console_Poll` clears ORE, FE and NE and resets `ErrorCode` and
`RxState` whenever a receive fails. The byte that overran is gone either way;
what must not be lost is the ability to read the next one. Five conformance
runs afterwards, and the link was still up.

**A trap found while chasing it:** `STM32_Programmer_CLI -c port=SWD mode=UR`
with no `--start` leaves the core halted, so the board is dead for a second
reason by the time the first is being investigated. Always end with
`--start`.

## The intermittent Modbus silence - found, 2026-08-28

The long-standing one, the one 600 requests had ruled four causes out of. It
was the IMU poll, and it cost 0.45 % of frames.

**The board loses a character of the REQUEST, not the reply.** The counters
match one for one - hammer the link and every silent call shows up as a
`char_overrun` and a `bus_comm_error`:

| requests | silent | char_overrun |
|---|---|---|
| 1393 | 7 | +7 |

**What blocks:** `Board_ImuPoll` reads a 276-byte SHTP cargo at 1.48 MHz,
which is 1.5 ms. `main.c` already said so in a comment. The `!link_busy()`
gate only looks BEFORE the poll, so a request arriving during one is lost.
Holding the IMU loop proved it: 5 silent in 1123 with it polling, **0 in 1283
with it held**.

**And there is no FIFO to absorb it.** `HAL_UARTEx_DisableFifoMode` is called
on all three ports, so the receiver holds ONE character - 87 us at 115200.
A 1.5 ms block is seventeen character times, and every one after the first is
gone. This was first written up here as the block being "longer than the RX
FIFO covers", which implies a depth that does not exist; the truth is starker
and is why moving to the interrupt fixes it completely rather than partly.

**Why a block cost anything at all:** USART3 was the only one of the three
ports that did not receive on interrupt. `dev_uart.c` set `.interrupt = false`
for it, on the reasoning that "the master on it is a person or a script, not a
bus". USART2 and UART5 both had the ISR. So the console's wire - the one the
host actually uses - was the one that could not tolerate a busy main loop.

**Fix:** USART3 receives on interrupt like the other two. Two things had to
move with it:

* it had no `USART3_IRQHandler`, so enabling the interrupt without writing one
  would have put every byte in the default handler's endless loop;
* `Console_Poll` read through `HAL_UART_Receive`, which would have lost every
  race with the ISR and left no way back to Modbus at all. It reads the same
  ring the link does now.

Measured after: **1604 requests, 0 silent, 0 overruns**, with the IMU polling.
Four full suite runs green in a row, where flakes had been one run in three.

## PB2 is a rail, not a signal

Three separate paths wrote AFE_ON straight to the pad: the Modbus coil, the
`0x6D` afe command and `testrig_pin_write`. Once the rail became reference
counted, any of them was undone by the next acquire or release - the thermal
observer borrowing the rail for an NTC sample was enough. It read back as a
coil written off that reported on, and as a pin written low that came back
high about one run in three.

All three go through `Board_PowerAcquire`/`Release` now. The `0x6D` reply
carries the holders as well, because `on` after an explicit off is true rather
than a failed write, and nothing on the wire could tell those apart.

## The IMU came back from an AFE power cycle and reported nothing

Measured 2026-08-29, four cycles in a row. `afe.disable()` then `afe.enable()`:
the loop returned `running` in 0.71 s carrying feature 5 @ 2500 us with
`pending` false, 17 cargoes arrived, and **no report ever came** - 15 s of
silence with nothing on the wire saying anything was wrong.

`pending` false is the firmware saying the Set Feature took. It had not. The
write goes out during the part's advertisement - 276 bytes over several
cargoes - and is accepted at the SHTP level and discarded. `Board_ImuWrite`
returns true either way, so `s_feature_pending` cleared and the request was
never made again.

What ruled out the alternatives:

| Tried | Result |
|---|---|
| Wait longer after `running` - 0.5, 2.0, 5.0 s - then set it by hand | worked every time, 774 reports in 2 s |
| Gate the automatic apply on one cargo having arrived | still silent: the write still landed inside the advertisement |
| Gate it on 60 ms of quiet since the last cargo | first report 0.33 s after AFE_ON, four cycles in a row |

The gate is quiet time, not a cargo count: `!intn_asserted()` is also true in
the gap between the reset wait ending and the part starting to talk.

## The IMU's rate ceiling is the transfer, not the poll

Measured 2026-08-29 with the rotation vector (0x05), link idle:

| SPI clock | asked 400 Hz | errors in 5 s |
|---|---|---|
| 1.48 MHz | 381 Hz | 28 |
| 2.97 MHz | 397 Hz | 15 |

Polling more often was tried instead and made it **worse**: at 4 kHz the rate
fell to 360 Hz and errors tripled. 2.97 MHz was previously rejected with
"every read came back FF"; that did not reproduce - 47 000 reports, zero read
errors. What is different since is the chip select held across header and
cargo.

Sustained after both changes: **393 Hz, 0 errors over 30 s**, against the
BNO085's own 400 Hz ceiling for that report. Under a hammered link it settles
at 345 Hz - `link_busy()` lets Modbus win, which is invariant 5.

The errors that remained were not errors. Every one carried report id **0x00**
- padding after the last report in a cargo. The walk was right to stop and
wrong to count it: a cargo cut short ends mid-report with real data and is
caught by the length test instead.

## Two channels report centi-degC and only one is a thermistor

`scaling.converter` picked the NTC curve off the unit alone, so the MCU die
channel - a linear sensor whose TS_CAL pair lives in the MCU's system memory,
not on this board - was cooked as a 10 k thermistor. **Measured 2026-08-29:
the dash showed -5.8 C for a die the observer had at 72.0 C.** Nothing was
visibly wrong: it is a plausible number under a C.

No arithmetic on this side can convert it - the constants are not in the
board's record and never will be. `converter` falls through to volts at the
pin for any centi-degC channel that is not the NTC, and `symbol()` says V
rather than C so the number cannot be read as a temperature. The die
temperature comes from the observer, over 0x6E device 8, which is the only
side that has the curve.

The same shape as the three millivolt channels and their three dividers: one
unit, several conversions, and `signal` is what tells them apart.

## The dead time went from 80 ns to 30 ns, by request

Asked for 2026-08-29. The arithmetic it replaces is kept, because nothing
disproved it: 59.4 ns of worst-corner gate overlap plus the 2EDL8034's 6 ns
TDMOFF is **about 65 ns needed**, and 80 ns was fitted against that. 30 ns is
under the figure and above the firmware's 20 ns floor.

t_DTS is 4.21 ns, so the request lands on **DTG 7 = 29.5 ns**, which is what
the board reports.

Written by `Board_PwmInit` now rather than inherited from the .ioc. Twice on
the same day a CubeMX regeneration moved a peripheral setting the drivers
depend on - SPI2's mode, then SPI4's - and this is the one where a silent
move shorts a leg.

Still nothing on a scope, and no current has flowed through a leg. Both
numbers are datasheet arithmetic (invariant 10).

## A teardown list written into the middle of the picture

Reported missing five times and printed every time. `paint` addresses every
row absolutely and never scrolls, so when a view stops the cursor is wherever
the last CHANGED row left it - somewhere inside the drawing. Everything
printed after that landed on top of the picture and read as part of it, and
the shell prompt landed there too.

**It was invisible to every check because a redirected stdout is not a
console.** With `console` false `paint` writes plain lines, the cursor is
already at the end, and the closing lines come out perfectly - which is what
five rounds of verification saw. Anything that only reproduces on a terminal
cannot be checked from a pipe, and nothing in this tree could.

`screen.park(rows, console)` puts the cursor on the first line below the
frame and clears from there down. Every view uses it in place of the `clear`
it used to do on the way out - clearing wiped the reading and then printed
the list onto a blank screen, where the prompt could take it with it.

## A minute of dry switching at 30 ns, and the model 20 K under it

Measured 2026-08-29. Three legs at 50 %, 60 s, DTG 7 = 29.5 ns. No trip, no
throttle, 0 overruns, and the observer stepped through the whole run at
10 Hz - it runs on power and time while AFE_ON is low, which is what it is
for.

| | before | after | +40 s |
|---|---|---|---|
| NTC | 36.4 C | **51.5** | 46.3 |
| MCU die | 72 C | 78 | 78 |
| worst node | mcu 22 % | mcu 18 % | mcu 20 % |

**The model's nodes came back at `drivers` 29.4 and `phases` 28.0 against an
NTC of 51.5** - and the NTC sits in the drivers' hot spot. It under-predicts
by about 20 K after a minute of load. The 3 to 4 K gap the four-state rig
found is the same defect at a hundredth of the excitation.

The loss constant is the suspect: switching loss was modelled at 1.20 W and
the dead surface measured 2.04 W.

## The dead time was in three places at once

Measured, in one hour: the .ioc said DTG 19, `Board_PwmInit` wrote a #define
of 30 ns, and the board reported 79 ns because the binary on it predated the
change. Three answers to one question, and the one that mattered was the
flash nobody had refreshed.

It lives in the calibration record now, id 13, CAL_VERSION 5. Set it, save
it, reset, and the board carries it - proven: 45 ns asked, 42 ns held (10
counts of 4.21 ns), read back from flash after a reset.

Two things this turned up:

* `Board_PwmInit` cannot read the record. It runs before `Board_CalInit` -
  it has to, its job is driving six gate inputs to their idle level and that
  cannot wait on flash - so reading the record there got zero and the floor
  turned it into 21 ns. main() applies it after the record loads; the .ioc's
  value stands for those few microseconds and nothing is armed.
* `BOARD_CAL_PARAM_COUNT` is what op 0 walks. Adding an id without moving it
  is a field the board holds and never reports.

## The supply tripped its OCP at 29.5 ns of dead time

Measured 2026-08-29, and the first over-current this board has caused. Three
legs at 50 %, dry - no motor, no load - with DTG 7 = 29.5 ns. A 30 s run and
a 60 s run had already passed at the same setting; the trip came during a
longer one.

**Dry switching should draw almost nothing.** An over-current with no load is
shoot-through: both FETs of a leg conducting through the dead time, which is
the one thing the dead time exists to prevent and the 2EDL8034 has no
interlock against.

This is the arithmetic that 80 ns was fitted against, arriving:

    59.4 ns worst-corner gate overlap + 6 ns TDMOFF ~ 65 ns needed

29.5 ns is less than half of it. **The number was asked for and delivered;
the trip is what says the arithmetic was not decoration.**

Two things changed after it:

* `Board_PwmSetDeadTime` ROUNDS UP. It truncated, so a request for 30 ns
  became 7 counts of 29.5 - under what was asked for, in the one direction
  that is unsafe. The floor beside it had rounded up since it was written,
  for exactly this reason. 30 ns now lands on 8 counts = 33.7 ns.
* The value is in the calibration record, so raising it is a write and a
  reset rather than a rebuild.

**Still under 65 ns, and nothing has been on a scope.** The trip is one data
point at one duty on one supply.

## A reset under the broker takes every session with it

Measured 2026-08-29. The console handover happens once, when the broker takes
the port - so a board that resets under it comes back in its text console and
answers nothing ever again. Every session sharing that port went silent on
0x41 together, and the broker had no way to know.

It re-opens the link and retries now, but **only on silence and only once**:
a refusal is an answer and must not be retried, and a board that is simply
gone should say so rather than double every timeout.

Three things the same hour turned up beside it:

* **The way out could raise.** `demos.py --leave` opens a rig to see what was
  left running, and a board that will not answer turned quitting the menu
  into a traceback over the prompt. Leaving is not a thing that fails: it
  reports and exits 0.
* **A refusal is not a No.** `stand_down` raised the board's sentence instead
  of returning False, so a caller asking *did the port come free* had to
  catch the explanation that it had not.
* **A stage was left armed.** The run whose teardown could not reach the
  board kept switching, and was found at 57 C on the way to 63 - which is
  what `--leave` exists to catch, and did once the link was back.

`session.py --force` is the escape: a broker refuses to stand down while
sessions hold the port, which is right until the LINK is dead - then the
sessions being protected cannot talk either, and the refusal is the only
thing between the bench and a working port.

## Releasing a serial port is not instantaneous

`test_conformance` crashed with `could not open port` whenever an earlier
suite had opened a session: the broker says it has stood down, then the
socket closes, then the thread unwinds, then pyserial lets go. The open is
retried for five seconds now.

## Where the shoot-through cliff is, dry, and what it is worth

Measured 2026-08-29, three legs at 50 %, no load, no motor. The bench
supply's OCP is the instrument: shoot-through is the only current there is,
so the trip point moves with what the supply is allowed to deliver.

| OCP | last dead time that held | first that tripped |
|---|---|---|
| 300 mA | 33 ns | 29 ns |
| 400 mA | **27 ns** | **25 ns** |

About 100 mA of shoot-through per 4 ns of dead time removed - one DTG count.

**THE TRIP DETECTOR IS THE OBSERVER'S UPTIME**, not the DC link. The link is
sampled with the stage down, and by then the supply has recovered: it read
24.89 V at the very step whose uptime had gone from 569 s to 13. A counter
that was larger a minute ago and is smaller now says the board restarted,
which is a fact rather than a threshold (invariant 10).

`drivers` climbed 21 to 26 % across the whole sweep without a knee - that is
the run heating up, not the dead time shortening. Shoot-through would break
the curve upward at the step that caused it, and did not: by the time it
draws enough to see thermally, it has already tripped the supply.

### 33 ns is aggressive, and dry is the easy case

It sits **6 ns above the cliff** at 400 mA, a step and a half of DTG, and the
cliff moves up as the supply is allowed to give more. The datasheet
arithmetic asks for 65 ns - 59.4 ns of worst-corner gate overlap plus the
2EDL8034's 6 ns TDMOFF - so this is half of it.

Every number above is at ZERO PHASE CURRENT. Turn-off slows with current as
the Miller plateau lengthens, and it is the charge still in the channel at
the transition that causes shoot-through. The gate behaviour these agree with
is the part that does not change much under load. The part that does has no
data points at all.

### The lead-lag test was invalid

Skew ±1 at 25 ns was tried and proved nothing: all three rows read an uptime
of 24 s, so the board had restarted in every step including the two marked as
holding. The baseline for each step was taken straight after the previous
step's reset, so the comparison measured nothing.

Skew stays at 0, which is the UNTESTED default and not a measured result. It
is calibration parameter 14 now, so a trim that is measured can be stored
without a rebuild.

## The rail was flipping at every connect, and the observer every five seconds

Reported from the bench 2026-08-29: AFE_ON on most of the time, and the LED
it drives blinking. Two causes, and the observer borrowing the rail was not
one of them - that is what it is for.

**`Coaxial63100` switched the rail as a side effect of connecting.**
`power_afe` defaulted to true, so every rig that opened switched it on and
every one that closed switched it back. Ten call sites inherited that,
including `switch.py`, which then turned it straight off again to arm. It is
false by default now and said at the seven call sites that read analog
channels - a rail change is visible where it is asked for.

**The observer sampled every 5 seconds.** The board's time constant is 6.8
minutes, so that was 80 samples a tau: eighty rail toggles for resolution
nobody was short of. 30 s is 13 a tau, and puts the front end's duty at
0.5 s in 30 rather than in 5.

A leaked hold made it worse: the host's reference has no lease by design, so
a script killed mid-run leaves AFE_ON high for ever. `power.release_all()`
clears it, and `demos.py --leave` reports it.

## The thermal picture: oval board, cross-shaped bore, blocky bottom

Three separate faults in one drawing, all measured 2026-08-29.

* **Oval.** Both renderers are square in CELLS - the ramp spends two
  characters and one row, the half-block one character and half a row - but
  a terminal character is about 9 x 20 pixels, so a cell is a tenth taller
  than wide and the board stood up. `CELL_ASPECT` stretches the field's row
  spacing instead, and the board comes back at 30 cells wide by 28 rows.
* **A cross, not a bore.** At 2.0 cells the raster draws 2-4-4-2 cells, which
  reads as an upside-down cross. A superellipse was tried at three exponents
  and changed nothing - the grid is too coarse to care. Size is the only
  knob: 2.4 cells, and it only bites on the plain ramp, since the colour
  renderer affords enough cells for the physical 5 mm to resolve.
* **Blockier at the bottom.** The half-block renderer pairs field rows two to
  a character row, so an odd count leaves the last one unpaired and draws it
  as a solid background block. `_fit` returns an even count now.

The cap on cells was 44, which is what made the circle a staircase: the
raster is the only antialiasing there is. 88 costs 7744 field evaluations and
draws in 42 ms, which at 2 Hz is nothing.

## AFE_ON kept coming back on, and the fix was the class not the instance

Reported from the bench three times, and cleared by hand three times before
the cause was addressed. The host's power reference is deliberately UNLEASED
so a session can keep a rail as long as it likes - and that is exactly why a
script killed mid-run left AFE_ON high for ever, warm and blinking, with
`users: ['host']` and nothing to expire it.

**Silence is the evidence.** `link_rx_count()` counts bytes in on any port;
`Board_PowerPoll` keeps the time and drops the host's holds after
`BOARD_POWER_HOST_QUIET_MS` without one. A session polls at 2 Hz in a view
and far faster in a test; a process that is gone sends nothing.

A COUNT and not a timestamp, because `link.c` has its clock injected through
the device and none of its own - whoever wants to know how long ago keeps the
time themselves.

Proven: took the rail, killed the process without a teardown, waited 16 s,
and the board had released it - `AFE False, held by nobody`.

## The bottom of the board was a cell coarser than the top

The half-block renderer pairs two field rows per character row. An edge cell
has only one of the two, and the upper-only case was drawn as a
background-coloured SPACE - which paints the whole cell, where half was
meant. So the top edge resolved at half a cell and the bottom at a whole one.

Measured on the outline: spans of 16, 24, 28, 32 down from the top against
10, 20, 26, 30 up from the bottom. With an upper half block for that case,
both read 16, 24, 28, 32.

Beside it, the grid stopped being square. A round board needs fewer rows than
columns because a cell is taller than it is wide, and the rows have to SPAN
the board rather than be a square grid whose blank margin is trimmed - the
trim lands on whole rows and leaves one more at the top than the bottom.

## A killed client left the broker holding the port for ever

Measured 2026-08-29, and the killed process was this tool's own: the session
running the switching went away with the process that started it.

`_Handler.finish` called the base class first and decremented after. The base
flushes and closes, which RAISES on a peer that was killed - so the decrement
never ran, the count stuck at one, and the broker could not take itself down.
`stand_down` then refused for a session that no longer existed.

The count comes down first now, in a `try`, and the close is what may fail.
Proven: a client made a real request, was killed mid-run, and the broker
released the count and stopped on its own.

**The stage was left armed by the same event** - all three legs at 50 %,
switching unattended, found at 51.1 C. `demos.py --leave` named it and put it
back, which is the whole reason that path exists. It is the only thing
standing between a killed run and a bridge nobody is watching.

## The observer read the DC link at mid-scale and scaled its losses by it

Measured 2026-08-29. Invariant 9 says AFE_ON decides what a reading MEANS,
and the thermal observer was the one place still reading a channel without
asking. Switching needs AFE_ON low, so **every estimate taken while the stage
runs** had an unpowered ADC reference under it.

The DC link is single-ended, so mid-scale is not zero - it is 1.65 V through
the 49.9k/2.2k divider:

    mid-scale link   39.08 V      against a supply on 24.9
    scale applied     1.588       switching loss goes as link volts
    driver U rise     14.5 K      predicted from the fake voltage
    driver U rise     14.9 K      what the observer actually reported
    driver U rise      9.1 K      what the camera calibration says

That factor of 1.6 is the whole of the ~20 K the observer was over-predicting
under load. It was invisible while the drivers and phases were one lumped
node each, because nothing else in the picture had the right magnitude to
disagree with.

The link voltage does not move when the rail toggles, so `board_thermal.c`
now updates it only while `Board_AfeOn()` and carries the last real one
otherwise. Never measured falls back to `switch_volts`, the voltage the
switching figure was calibrated at - a scale of 1, not a scale of nothing.

The phase shunts are DIFFERENTIAL and needed no such guard: an unpowered
reference puts them at mid-scale, which centred is zero amperes. Zero is also
what the hardware gives, since AFE_ON low leaves those amplifiers unsupplied.

## Switching one leg heated all three in the estimate

Measured 2026-08-29 with the IR camera: U at 50 %, V and W idle, and the
camera showed U's half-bridge alone. The observer showed all three the same.

`thermal_power_estimate` scaled switching loss by how many legs were driven -
`legs_driven / 3` - and then wrote the total into ONE `drivers` node and ONE
`phases` node. The information about which leg was there and was thrown away
one line later.

Ten nodes now, three drivers and three phases. The split keeps the bulk
exactly: a third of the capacity each and three times the resistance to
board, which in parallel is what the camera measured, so the four-state
calibration still holds and only the placement changed. One leg alone now
rises three times as far and three times as fast - which is the point.

Two things fell out of it:

* The NTC anchors `driver_v`, its physical neighbour, not the lump. An idle
  leg's estimate used to follow a neighbour that was switching.
* `thermal/test/check.c` stepped at 0.5 s against the board's own 0.1 s. With
  a third of the capacity per leg, 35 W crossed the whole ceiling inside one
  step and the budget was asked to warn about something already over. It
  steps at THERMAL_STEP_MS now.

## The observer's own sample latches the break

Measured 2026-08-29, twice. The observer borrows AFE_ON every 30 s to read
the NTC - and AFE_ON high removes the gate drivers' supply. With MOE set the
stage sees that as a fault, latches, and everything goes to idle.

`demos.py sample()` already handles it: stand down, raise the rail, measure,
re-arm. A bare script that only writes a duty does not, and its run ends
silently at the first sample - the estimate simply stops rising, which reads
like a modelling fault rather than a stage that tripped.


## The IMU sits at 90 degrees, and the 180-degree mount hid an order bug

Settled 2026-08-29 by a three-observation bench court: with the display
tared, a rotation about board X drew as Y, board Y drew as X, and CCW yaw
drew CCW. An axis SWAP with Z clean is no mirror - it is a 90-degree mount,
and no combination of component flips (the empirical dial) can produce it,
which is why every earlier dial finding contradicted the next.

Ruled out on the way, each with the evidence that killed it:

* **Mirrored axes** (the flip dial): mirrors cannot swap X for Y.
* **A conjugate report from the part**: would reverse Z too; yaw drew true.
* **The mount applied twice** (an FRS record in the part's flash): nothing
  in this tree ever wrote one, and the swap says 90, not double-180.
* **"Roterad 180 grader"** - the eyeball read of the layout. U13's pin-1
  dot sits in the board-frame LOWER-LEFT corner; a 180 mount puts the
  datasheet's upper-left corner at lower-RIGHT. Lower-left is +90 CCW, and
  the 3.8x5.2 outline lying long-side along board X says the same.

The mount that follows is the datasheet fig 4-3 row X=North Y=West Z=Up ->
(w,x,y,z) (sqrt2/2, 0, 0, sqrt2/2): `MOUNT = Rz90` in `orientation.py`.

The bug the 180 estimate HID: the display sandwich must wrap the body-frame
change in MOUNT on the left - `MOUNT * (conj(tare) * q) * conj(MOUNT)`. The
reversed order passed every numeric check under Rz180, because a 180 is its
own conjugate and both orders coincide. Any future mount change re-runs the
court in `attitude()`'s docstring; a 180 proves nothing about the order.

## A duty write's round trip is 8 ms longer after 85 ms of silence

`tools/pulse.py --on 0.1`, 2026-08-30, over the probe's COM port. Two
compare writes back to back: the second lands in 15.0-16.6 ms (23 pulses,
one run of twenty). The same write issued after 85 ms of idle: 23.5 ms,
twice (109.3 and 108.5 ms measured for a 100 ms hold, exact wait). The
extra 8 ms is on the wire or in the VCP driver waking, not in the sleep -
the wait was spun to the microsecond the second time.

Not monotonic in the idle, though: the same off write after 985 ms of
silence (a 1 s hold, 18:35) landed in 14.2 ms - 999.2 ms measured for
1000 asked. Two samples at 85 ms idle, one at 985: the 8 ms is real at
85 ms and absent at 985, and what it tracks is not settled.

What it means for timed switching: the host measures until the reply is
back, and where in that round trip the frame LANDED on the board is not
knowable from here, so a hold asked for at 100 ms is 93-108 ms at the
FETs. A hold counted in PWM periods by the board itself - a cycle count
on the duty op, cleared by the update ISR - is the only way to a number.

Also seen: `rig.write(analog=...)` cost three `state()` reads (31 ms each)
before the duty went out - `armed()`, the period and the held duties read
separately. One read now; 110 ms became 15.

## The pair alternates: op 10, and what the board said while it did

Proto 2.1, 2026-08-30. `0x6E` device 4 op 10 takes two compare triples;
TIM1's update ISR writes the other one at every OVERFLOW - DIR already
reads down there - so the preloaded compares land at the underflow and
each period, both slopes, is one triple. Written at the underflow instead,
a period would carry A up one slope and B down the other and the two high
sides would overlap. The dither is off while it runs; SetAll, SetAllFine
and Disable end it.

Proof from the board rather than the scope: 5 % U against V low as A,
V against U low as B, `state()` read twelve times mid-run at ~50 ms -
`duty` (118,0,0) five times, (0,118,0) seven, nothing else. Then 10, 30,
30 and 60 s runs, 31 V link, 8 ohm across U and V: every one with the
break clear under the bypass, 0 overruns, no gate shorts, a clean disarm.
Both half-bridges switching on the scope.

The NTC while it ran, 5 % duty, same mean power in the resistor as one
direction: 41.9 C before, 48.5 after 10+30 s, 50.9 after 30 more, 56.0
after 60 more - about +5 C a minute and not flattening, against +1.7 C
per 30 s one-directional. Two high sides switch instead of one. Recorded,
not explained; the thermal observer shows the margin to the record's
ceiling.

The observer charged U alone while it ran, though both legs switched: it
samples `Board_PwmGetDuty` at its own 10 Hz, that returned the compare
mirror the ISR swaps at 50 kHz, and the sample sat phase-locked on A.
Fixed 2026-08-30: while alternating, `Board_PwmGetDuty` returns each leg's
mean over the pair, (A+B)/2; `state()['duty']` still shows the triple of
the moment. Measured, 20 % U<->V for 20 s, observer read every 5 s:
driver_u 29.19 -> 40.51 C, driver_v 29.20 -> 40.51, driver_w 29.19 -> 29.62,
`duty` (475,0,0) at 5 and 10 s and (0,475,0) at 15 and 20 s.

## The DC link, spanned against a meter

2026-08-30, the first number on this board held against an instrument. A
DMM on the DC link read 30.05 V while the board, on the schematic's
49.9k/2.2k, read 31.04 - 3.2 % high, more than 1 % resistors explain on
their own; the reference sits in the same record and was not checked.
`calibration.span(5, 30050)`: the board answered 26044 (the code it
held) and stored gain -32,418 ppm on channel 5; read back 30.037 V,
saved, reloaded from flash, 30.040 V. `stored` went False to True - the
record is calibrated now, not the schematic's arithmetic. One point at one
voltage: offset and gain are not separated by it.

## The way back to the menu, timed

2026-08-30. ESC in a view to the front page drawn again felt slow. Measured:

| where | was | now |
|---|---|---|
| SESSION's teardown hold on the way to the menu | 2.0 s, `TEARDOWN_HOLD`, for lines a shell prompt would follow | none - the menu repaints over them; Q still holds |
| menu, page on screen | 2.03 s | 0.15 s - the turntable's solids build in a thread |
| menu, board on the stand | 2.03 s | 0.98 s |
| STL parse + centre | 0.60 s, twice - the LOD-32 solid and the shadow casters each parsed | 0.46 s once a process: `mesh.loaded`, `iter_unpack`, flat min/max, output bit-identical |
| `broker.clients()` with nothing serving | 2.0 s - a loopback connect to a closed port times out on Windows, no refusal | not asked unless `serving()` says so |

The stale WHERE file came out of the same look: a broker killed with its
view leaves the file, `serving()` returns it, `clients()` returns None after
the 2 s, and `count + 1` read LIVE 1 SESSION on a page with no broker. None
is nobody now.

Two things about the bands and the strip, both rich: a right-justified cell
is rstripped, so the air before a band's end and a chip's own trailing space
were never drawn - the right column is sized to its text instead; and the
boot strip's text column sat before the bar, so every milestone moved the
bar by the difference in length - bar first, text after in brackets.

`mesh.facets()` was still writing `Coaxial 63100.facets` beside the STL for
the photo and toon paths. Gone: the facets live in memory for the process.

## The board cleans up after a dead host, and the broker speaks for a live one

2026-08-30. Asked from the chat to turn the AFE on, the model did - and the
firmware turned it off again ~10 s later. Board_PowerPoll's host deadman
(BOARD_POWER_HOST_QUIET_MS, 10 s of link silence) drops every HOST rail
claim: right for a killed script, wrong for an operator thinking between
turns, where the link is naturally quiet.

Two halves, both measured on the bench:

* The broker answers for attached clients: one 0x41 read per 3 s of quiet,
  only while clients > 0 (`_Server.tick`). AFE asked on, then 15 s of
  client silence: still on, users ['host']. 13 s after a clean detach:
  off, users [] - with no client the keepalive stops and the deadman
  does exactly its job.
* The target's cleanup got wider: on the quiet edge the firmware now also
  runs Board_PwmSessionDrop - MOE down, break bypass back in force, once
  per transition. A process that armed the stage with the bypass on and
  died in os._exit: 13 s later pwm_enabled False, break_bypassed False,
  all six pins low. The observers never stop either way; they hold their
  own leases.

The keepalive shares the transport lock, so a timed write can queue behind
one version read - ~30 ms worst case. The cycle-counted duty (TODO item 8)
is still the honest fix for exact holds.

## The drive: a control law, written, and what the bench said about it

2026-08-31. `drive/` - dq current loop, dead-time table, min-max SVM,
square-wave injection with its demodulator, a two-state PLL in Kalman
form, a back-EMF error above a crossover speed, I/f, a polarity pulse -
behind `0x6E` device 10, host-tested through ctypes against a PMSM model
(`test_drive_core.py`, 59 checks). Nothing has run into a motor. What the
bench could say with the drivers unpowered and the AFE on:

| | |
|---|---|
| one step at -O0 | **10 040 cycles, 21 us** against a 20 us period; worst keepalive gap 16 ms |
| the interrupt path at -O2 | 6 756 cycles, 71 % of the CPU - the UART interrupt slipped behind it and 17 of 19 requests failed CRC while `char_overrun` stayed 0 |
| plus the conversions cached (`Board_PhaseScale`) | 3 874 |
| plus one trig pair per step and the ring conversion gated | **2 922 cycles, 6.2 us**, worst keepalive gap 519 us |

A control law at -O0 is not a thing: the four files on the interrupt path
carry `-O2` whatever the preset (CMakeLists.txt).

**A 71 % interrupt corrupts frames without an overrun.** The bytes arrive
on interrupt with their timestamps; delayed variably behind the ADC
interrupt, the recorded gaps crossed t1.5 and the frames split -
`bus_comm_error` +17, `char_overrun` +0, and 0x41 (two bytes) answering
where 0x6E (four) did not. Read over SWD while the link was like that:
`s_cycles_last` 0x1A64.

**Two replies outgrew the PDU on the same afternoon.** 0x41 at 205
characters of description - every connect answered SERVER DEVICE FAILURE
straight after the flash; the string is under 170 now and says why. Then
device 3 op 0 with forty-five parameters: 310 bytes. Op 0 keeps its
fifteen, byte-identical, and op 8 pages the rest.

**Two injected ranks need scan mode.** With `ScanConvMode` off the HAL
discards `InjectedNbrOfConversion` and writes JSQR from rank 1 alone:
JSQR read 0x2A0 over SWD - JL 0, no JSQ2 - while PCSEL already carried
channel 10, and the DC link read 0 beside a meter reading 31.05 V.
`Board_SyncArm` switches ADC3 to scan once; the regular sequence keeps its
length of one. Rank 2 then read **26037 raw, 31.06 V** against the
meter's 31.05.

**The sample-point scan is noise without switching.** Gates off, twelve
points across the period: the argmin landed on 990 of 2376. The scan needs
the zero vector under it, and refuses without the stage now.

**The AFE, gates off, AFE on, CCR5 2360, 2000 periods:** U 54.9, V 55.6,
W 74.9 codes rms - 0.35, 0.35, 0.47 A - sigma_i 0.40 A, ENOB 8.3 bits. The
floor 2026-08-27's sweep found (0.35-0.41 A). W is a third noisier than U
and V and nobody has asked why.

**The demodulator re-seeded every cycle** and its window was 2n+1 samples
against a sign pattern of 2n; at fs/4 a window could hold two of one sign
and the fundamental's slope leaked in as inductance. Found by the host
model: 99 estimates in 500 periods where 125 were due. Continuous now: 2n
consecutive differences see n of each sign whatever the alignment.

**The dead-time error of a phase-aligned vector is 4/3 V_dt in dq**, not
V_dt: I on phase a and -I/2 on b and c give (-f(I), +f(I/2), +f(I/2)),
which Clarke puts on d as (2/3)(f(I) + f(I/2)). The model's 0.767 V at 2 A
was right and the first test expectation was not; the identification
sweeps I and unfolds f from that.

**Two headers had `\r\r\n` line endings** - board_limits.h, 196 lines,
comms_limits.h, 71 - a stray CR on every line, committed. Normalised.

**`Board_CalSetParam` refused zero for every id**, including the dead-time
skew, so a skew could never be set back to none through the record. The
refusal covers the thirteen divisors now.

## The caches were off, and every cycle count before today with them

2026-08-31, chasing the drive's interrupt. CubeMX generated neither
`SCB_EnableICache` nor `SCB_EnableDCache` and nothing had asked, so the M7
fetched every instruction from flash at four wait states. The instruction
cache is on now (main.c, USER CODE Init). Same binary otherwise:

| | cache off | cache on |
|---|---|---|
| drive interrupt, mode off, converters | 2 922 cycles | **1 780** |
| interrupt entry after the ADC trigger (`at`) | 965 ticks, 4.06 us | **385 ticks, 1.6 us** |
| main loop worst keepalive gap, idle | ~500 us | **120 us** |
| sensorless on the model, injecting, spinning: the step | 8 316, past the period | **4 688** |
| the whole interrupt, trigger to exit | wrapped - past 20 us | **12.3 us** |

The trig went to a polynomial in the same change (`drive_sincos`, 3e-6
worst, where four libm calls a period had been); the two were not
separated. The data cache stays off: `Board_CalSave` reads the sector back
through a pointer after programming, which a cached line would answer with
the old record; .data and .bss are in DTCM, which no cache touches; there
is no DMA. Every number in this file before this entry was taken cacheless.

**Where the virtual step goes**, cache on, through the probe device 10 op 0
carries in raw cycles: the model's sample ~510, the law 1 700 idle to
2 230 spinning, the model's advance ~1 100 with four Euler sub-steps and
one tanhf.

**The model as the observer's source, on the bench.** Profile
`outrunner_14p.json`, 2 V fs/2 injection, a 60 Hz PLL, the AFE on, no
stage: locked from 0.3 rad away to 0.002 rad; spun on 0.6 A of q current to
441 rad/s electrical with omega_hat 441 and 0.009 rad of error - read from
one reply (op 12), because two 15 ms apart are six radians of rotor at
that speed. The NTC rides ADC1 rank 2 and read 40670 raw beside the
meter's ~40500 while the sync held the converters.

**The thermal observer's sample under the drive**, sampling every 5 s for
the test, the sync armed throughout: with the AFE on it read the NTC at
37.5 C and the A1335's die at 36.5 through the rank and the bus; with the
AFE off and the stage idle it borrowed the rail - `users ['thermal']`,
leased - read 37.52, and let go. The MCU die is the one it loses while the
sync is armed: an 810-cycle sampling on ADC3 that cannot ride the injected
sequence, refused like every other meter read, and the model runs on
without it as it always did for a silent sensor.

## The chooser opened BOARD CHAT on every start, and the first fix was a guess

2026-08-31. ESC from a view under MOTOR CONTROLLER or BOARD CHAT was to
reopen the front page on that question (`menu.py --open`), so the chooser
got a list of those views, `$Asked`. From then on every start went
straight into the chat page. First answer: a PowerShell script reads the
caller's variables when its own are unset, so a `$view` left in the
session leaked in - demonstrated (`$view='chat'; & { $from = $view }`
reads `chat`), an initialiser added, and the symptom stayed.

What found it: the front page alone in a hidden console, every key it saw
logged - 60 frames, 9.39 s, no key, returned 0. So not input. The chooser
itself in a hidden console, its child processes listed: the first child
was the chat page. `$asked = $null; $Asked = @(...)` then printed `$asked`
as the list - **PowerShell variable names are case-insensitive**, `$Asked`
and `$asked` (the `-Name` parameter) are one variable, `$view` became the
list, `while (-not $view)` skipped the front page, and `$Views[$view].Chat`
on an array is truthy. Renamed `$SubViews`; the same run then showed
`python -X utf8 tools/menu.py --port COM4` as the first child, no `--open`.

And the append that recorded this truncated the file: `io.open(p, 'wb')`
opened FINDINGS for writing before the expression it was to write raised
(`str + bytes`), 1901 lines went to zero, two more scripts built on the
empty file, and commit 8a61a3d carried it. Restored from 86f433f plus the
two later entries replayed from their scripts. Open for writing LAST,
after every value exists - and read `git diff --numstat` before pushing.

## A lingering broker over a dead board, and the view that traced back

2026-08-31, the board deliberately unpowered to try simulated mode.
ROTOR OBSERVER died of a ConnectError traceback instead of falling back
to the stand-in. `Voltage 0.00V` off the probe confirmed the board was
off; the probe itself enumerated fine (STLINK_V3S, VCP COM4).

The chain, each link measured:

1. A probe that finds no board still spawns a broker for the port, and
   the broker idles **45 s** before freeing it (`_Server.linger`).
2. Inside that window `open_session(simulated=None)` asked
   `session._answers`, which attached to the broker's SOCKET and closed
   it - proving the process was there and nothing about the board.
3. So `auto` committed to a real port, `session.board` connected through
   the broker, the board said silence, and ConnectError escaped: six of
   the seven views called `.open()` with no guard at all.

Three fixes. `_answers` now asks the BOARD, through a new broker op
`answers` that is **a look, not a use** - the design's own words: a
`request` makes a client one of the sessions holding the port, and
`test_broker` states outright that asking must not be what takes it
down. A first attempt used the ordinary `request` op and did exactly
that: 22 passed 0 failed became 21/1 (the check ran against HEAD to
prove the regression was mine). `screen.open_rig` says the board's own
sentence and returns None, so no view dies of a traceback - it catches
OSError beside RigError, because a socket that times out and a port
another process holds are not RigErrors and mean the same to a reader.

**The front page now says which.** `session.board_answers` is the
decision `open_session` makes, asked by `menu.py` on its own thread, so
the chooser's chip and a view's origin cannot disagree. It is not free:
a full probe with the board unpowered measured **8.42 s**, against 0.00 s
for `port_state`, so the page shows LINK: PROBING and flips to SIMULATED
when the answer lands - re-asked every 30 s, skipped entirely under
`--simulated`.

One more, found on the way: a suite that could not run recorded **0**
checks over its real size, and the quoted total fell 2114 -> 2080 with
four documents suddenly wrong. `counts.record` now drops a zero - a
suite that ran nothing measured nothing.

## The accumulator closes on the clock, measured

2026-08-31, the board powered. `accumulate = 0` lets the converter run
free and closes a record on `interval_us`. Asked for 100 records a
second on two channels: **33 to 89 sweeps a window, mean 69.9** against
the 66 the loop's 13.2 k conversions/s predicts, 89 records/s delivered,
0 dropped, `interval_us` 10 000. The record's own count is what a host
divides by: sum/count gave **39 610.9 codes against an independent
burst's 39 614.0**, 0.008 % apart. Stride 14 as the formula says.

## A known tone through the whole path

`tools/daq_integrity.py`, same session. A tone is generated ON THE BOARD
in the converter's place, filtered by the chain `coaxial.bessel`
designed, decimated into the ring and read back - so the host knows what
every output sample should be, and a record that fell out shows up as a
phase that jumped rather than as nothing at all.

Chain: boxcar 250 x decimate 8 off a 1 MHz generator, two biquads,
cutoff 100 Hz, 500 records/s. Two passes, the alias placed so it folds
onto the in-band tone's own output frequency - indistinguishable in the
record, so only the filter can have stopped one.

| | measured |
|---|---|
| in band, 61 Hz | **10 601.9 codes against the design's 10 621.5** - 0.2 % |
| phase over 36 windows | worst step **0.0163 rad**; one lost record would be 0.7665 |
| out of band, 250 061 Hz | **-261.6 dB**, against a prediction of -224 |
| the ring | 0 dropped, peak 8 of 1170 records held |
| every record | 250 samples, no exceptions |

**Two things had to be got right before those numbers meant anything,
and both were the test rather than the board.**

The generator burst starved the link. A round trip between `tone` and
`start` is 15 ms, which at 1 Msps owes 15 000 samples; the first bound
was 4096 and one burst of them is milliseconds, so the next `0x6E`
answered silence - RTU discards a frame whose characters arrive more
than t1.5, 143 us, apart. `BOARD_DAQ_TONE_BURST` is 256: ~40 cycles a
sample is 22 us, and a loop turning at 15 kHz still sustains 3.8 Msps.
What the clamp drops is dropped rather than owed, or the next turn
bursts again and never catches up.

And the filter's settling was being judged as signal. Started from rest
it meets a step - the tone's DC offset - and its answer to that is not
the tone: judging from the first record put **0.2957 rad** in the phase
track where every other window sat at 0.016, and read a stopped alias as
**-39.6 dB** that was really the transient. Three time constants of the
cutoff are dropped now, and the same run gives 0.0163 rad and -261.6 dB.

One real defect on the board's side: the task's `accumulate` is the
chain's first stage, so what goes into the biquads is the MEAN it
produced. Pushing the sum instead multiplied every reading by the count
and a 32 768-code tone arrived as 8.2 million - `filter_push_value` is
the entry point that takes an already-averaged value.

**What this does NOT say.** Nothing analog was involved: with a tone on,
the meter is not read at all. It says the ring, the link and the
arithmetic are honest, not that the front end's noise folds the way the
design predicts. And the converter is still polled one channel per main
loop turn - 13.2 k conversions/s - so the 3.75 Msps the silicon can do
needs a DMA path that does not exist yet.

## Every record exact, and what the chain costs the main loop

2026-08-31. The sine passes judge a gain and a phase, which are
aggregates. A ramp - `offset + (n * step) mod modulus` - is an integer
sequence a host computes in closed form, so every record can be checked
EXACTLY. A float rotation cannot be: reproducing single-precision
arithmetic on the host to the last bit tests two compilers, not a link.

`tools/daq_integrity.py`, 64 samples summed and every 4th boxcar kept,
the ramp at 4093 (prime, so its period shares no factor with the record
length):

| | measured |
|---|---|
| the transport | **600 records, one candidate start, and under it every record is the exact integer** |
| both fields | the same sample in all 600, so no stride slipped |
| the sample count | 64 on every record, no exceptions |
| through 2 biquads | worst **0.0110 codes of 4093** against the same filter in float64 - 2.7 ppm, which is float32 in the biquad state |
| the ring | 0 dropped, peak 4 of 1170 |

**Which boxcar the first record came from is not on the wire**, and one
record does not pin it down: a ramp's sum over a window is piecewise
linear in where the window starts, so several starts give the same
total - three, over 8192 searched, all the same phase one period apart.
So the question asked is the one that matters: is there a place these
records could have come from where every one is exactly right? A stream
with a record missing, repeated or altered has none. Searching 512
boxcars - the first is `decimate-1`, and 512 is under one ramp period -
leaves exactly one.

Two criteria had to be made criteria first. Asking for the sine passes'
1 MHz made 3906 records a second against the couple of hundred the link
drains, and the ring said so exactly - **1192 dropped, peak 1170 of
1170**; an exactness test on a stream with holes in it tests nothing.
And the filter error was read relative to the sample, which near the
start is nearly zero: 1.3e-3 at record 0 against nothing wrong. In codes
it is 0.011.

## What the chain costs the main loop

Off the board's own keepalive gap, the instrument that already answers
this. t1.5 is 143 us at 115200 and is the budget: past it RTU discards a
frame.

| | worst gap | samples/s |
|---|---|---|
| idle, no task | 24.2 us | - |
| the converter polled, accumulate 64 | 34.6 us | 8 626 |
| ramp, no biquads | 86.4 us | 580 017 |
| ramp + 4 biquads | 84.9 us | 586 000 |

**The biquads are free and the boxcar is the cost.** They run on what
the boxcar dumped - one call per 250 samples - so an eighth-order chain
measures the same as none at all, twice, and 3 MHz asked for gives the
same 580 k as 1 MHz: the ceiling is the loop, not the arithmetic.

**A generated sample costs 440 cycles** through `feed()` with two
fields, not the 40 that was estimated - which is why the burst bound
matters. At 256 the worst gap was **262 us**, over t1.5 by itself; at 64
it is 86 us. Throughput fell 870 k to 580 k for it, because the loop is
generator-dominated and the rest of a turn is only ~51 us. That is the
right trade: a test source must not endanger the link it is testing.

**The converter is still polled one channel a turn** - 8 626 sweeps a
second - so the 3.75 Msps the silicon can do (16 bit, 1.5-cycle
sampling, 37.5 MHz ADC clock) needs continuous conversion and DMA, which
this firmware does not have. The generator's 580 k is what a main loop
can carry, not what the ADC could.

## Every channel through the chain, and what each one costs

2026-08-31. All ten analog channels and the three drivable pins, filtered
and decimated on the board. `tools/daq_allchannels.py --sweep`, every
number read off the board rather than assumed:

| ch | stride | sweeps/s | link/s | boxcar x d | out/s | cutoff | alias |
|---|---|---|---|---|---|---|---|
| 1 | 13 B | 3787 | 292 | 2 x 8 | 236.7 | 46.7 Hz | -31.2 dB |
| 4 | 25 B | 2140 | 152 | 2 x 9 | 118.9 | 24.3 Hz | -32.0 dB |
| 7 | 37 B | 1485 | 102 | 2 x 9 | 82.5 | 16.3 Hz | -32.3 dB |
| 10 | 49 B | 1129 | 77 | 2 x 9 | 62.7 | 12.3 Hz | -32.3 dB |

**More channels is a longer record is fewer records a second is a lower
cutoff.** Nothing chooses that; it falls out of two measured rates - what
the loop sweeps at, and what the link carries.

A ten-channel run: 300 records in 8.63 s, 0 dropped, peak 2 of 334, and
the readings where they should be - NTC 40.7 C, DC link 30.9 V, +5V
5.083, Phase V -42.5 A (the op-amp fault HARDWARE records).

**A digital pin through the chain is its DUTY.** A level sampled once and
decimated is aliased by construction: KEEPALIVE toggles at ~100 kHz and
read as a coin toss. Counted high over the window instead, it comes back
**50.0 %** - which is the square wave the main loop makes, and a byte a
pin where there used to be one snapshot word.

### Three defects the numbers found

**A chain designed for a rate the board does not have.** `fs` was
hardcoded at 1 MHz while the polled loop gives 1129 sweeps/s on ten
channels, so the boxcar wanted 8117 samples a record and delivered ONE
record in fifteen seconds. The sweep rate is measured now.

**Measuring that rate free-running measures the link, not the loop.**
With `records=0` and no rate asked for the board substitutes what the
link carries and gates the triggers to it - so accumulate 1 read 279
sweeps a second where a finite burst gives 3787. A run that ends is left
alone, because it ends.

**And the substitution did not know about the filter.** It multiplied
`decimate x accumulate` and the chain's own decimation was not in the
config, so ten channels asked for 62.8 records a second and made 8 - the
missing factor of 9 being the filter's. `Board_DaqTriggersPerRecord()`
counts them on the board now, and the substitution runs from BOTH the
configure and the filter op so the order they arrive in does not matter.
That needed one more thing: the board remembers that the rate was
AUTOMATIC rather than re-deriving it from the answer, because after the
first substitution the config no longer reads as zero and the second call
returned early. 8 records a second became 35.

## The board filters harder when the link cannot keep up

2026-08-31. A ladder of whole chains goes down to the board; it climbs
when its ring fills and comes back down when the link has caught up. What
a slow link costs is then BANDWIDTH rather than records.

Three channels, 1450 sweeps a second, a link carrying 181:

| rung | boxcar x dec | records/s | cutoff | alias |
|---|---|---|---|---|
| 0 | 2 x 10 | 144.9 | 29.0 Hz | -32.5 dB |
| 1 | 2 x 19 | 71.8 | 14.5 Hz | -34.3 dB |
| 2 | 15 x 5 | 36.4 | 7.2 Hz | -30.6 dB |
| 3 | 151 x 1 | 18.1 | 3.6 Hz | -15.6 dB |

**Measured, the whole cycle**: reading, the ring sat at 6 of 780. The
host then stopped reading for five seconds - the ring climbed to 590 and
the board went 0 to rung 3. The host read hard again: the ring emptied
and over the next twenty seconds the board came back down to rung 0. Six
changes, and **0 records dropped in the whole run**.

It climbs all three rungs in a few records rather than one at a time, and
that is the right response rather than a defect: a ring at six eighths and
rising means the link is gone, and the evidence is the drop count. Coming
down is deliberately slow - 64 records below an eighth per step, which at
rung 3's 18 records a second is three and a half seconds a rung. A ring
empties the instant a host reads it, so one look says only that it just
drained, and every change costs the filter its settling.

**Rung 3's -15.6 dB is the honest cost of the bottom of a ladder.** Four
rungs at a factor of two is 8x, and by the last one the ratio has run out
of factors to split between a boxcar and a decimation: 151 x 1 leaves the
biquads nothing to work with. A ladder that needs its bottom rung to be
clean wants more rungs, closer together, or a converter rate with more
divisors - the design reports the number either way, which is the point.

## METER BRIDGE read as hung, and its own buffer box said why

2026-08-31, straight after the view was moved onto real blocks. Three
frames took **17.4 s** - 5.8 s each against the eight a second asked for.

The BUFFER box diagnosed it without anything being added: `took 513`. The
view asked the board for 200 records a second, drained up to 512 of them
a frame - about a hundred round trips at five records a reply - and the
board made another five hundred while it did, so the cap was hit every
frame forever. The ring never overflowed and nothing was dropped; the
view was simply doing a logger's work at a meter's frame rate.

**A meter wants one averaged reading a frame**, which is exactly what the
chain is for: the rate follows `--hz`, the board sums the whole frame's
worth of samples into one record, and the drain cap is 32. Measured
after: 40 frames in 9.3 s, `peak 2 of 356, waiting 1, took 2`.

Two more things the fix needed. The state read for the buffer box was a
round trip a frame - a third of the budget for a gauge that moves slowly
by construction - and is twice a second now. And the wrapper still had
the old default: `demos/adc.ps1` passed `-Rate 200` over the view's new
one, so through the chooser the ring sat at **339 of 356** while a direct
run sat at 1. A default changed in one of two places is not changed.

## The link off the draw loop, and both buffers on screen

2026-08-31. A view that reads the board in its draw loop runs at the
LINK's pace: METER BRIDGE spent three round trips a frame, 190 ms of a
125 ms budget, and every one of them was the terminal sitting still.
`screen.Feed` runs the reading on its own thread and hands the drawer
whatever the last one produced. **40 frames went 9.3 s to 6.7 s.**

**No lock, and that is the design rather than a shortcut.** The reader
builds a whole object and assigns it to ONE attribute; the drawer reads
that attribute once. An assignment is atomic under the GIL, so a frame
draws one consistent snapshot and never half of two, and a lock would put
the latency back in the draw loop - the thing being removed. The records
cross in a `deque`, whose append and popleft are atomic for the same
reason. ONE THREAD TOUCHES THE LINK: a serial transport is not
re-entrant, so the drawer reads `latest` and nothing else, and the feed
is stopped before anything starts putting the board back.

**Both ends of the pipe are on screen**, because either can be the one
that fills - and the first arrangement blamed the wrong one. With the
reader's period tied to the frame rate, a terminal drawing once a second
read twice a second and the TARGET ring overflowed: **356 of 356, 208
dropped**, for a slowness that was entirely the terminal's. The reader's
job is to keep the board's ring empty, which is the link's business and
not the screen's, so its period is 10 ms and fixed. Same run after:
**HOST peak 129, TARGET peak 8, 0 dropped.**

The host's queue also had to become a real one. The reader replaced its
last result each pass, so whatever the frame had not drawn went with it -
a silent host-side drop. It is a bounded deque now and counts what it
has to evict.

**And the meter runs a real low-pass.** It was a clock-closed window,
which averages but shapes nothing; it designs a Bessel against the loop
rate it measures and shows what it got - cutoff 1.60 Hz at order 4,
-26 dB of what would fold, sum 47 and keep one in three. The box reports
the loop's rate LIVE off the board's own trigger count beside the figure
the chain was designed against: **1398 sweeps/s against 1127 designed**,
because the measuring run and the drawing run do not load the link the
same way. Where the loop cannot be measured at all - the stand-in
produces only when read, so it measures zero - there is nothing to design
a chain against, and the window is the fallback rather than a traceback.

## Where a 115200 link actually goes

2026-08-31. 115200 8N1 is 11 520 B/s of line. The acquisition path was
getting 3 800, and the reason is not the baud rate.

**Half of that 3 800 was a measurement artefact.** With `records=0` and
no rate asked for, the board gates itself to what the link carries, so
the ring never holds a full PDU and every read came back a quarter full -
10.3 records of a possible 24. Told an explicit interval so the ring
overfills, every read is a whole 240-byte PDU and the payload is
**5128 B/s, 45 % of the raw rate**.

The other half is a fixed cost per round trip, not a bandwidth:

| read | payload | round trip | line time | waiting |
|---|---|---|---|---|
| gated | 103 B | 33.05 ms | 10.03 ms | 23.0 ms (70 %) |
| full PDU | 240 B | 46.80 ms | 21.88 ms | 24.9 ms (53 %) |

**About 25 ms whatever the payload** - a VCP driver's latency timer and
the host's scheduling, the same thing the clock probe measured from the
other side (a 16-byte reply is 1.7 ms of line and 35.9 ms of round trip).
A Modbus PDU caps a reply at 253 bytes, so there is no bigger read to
amortise it with: the ceiling is 240 bytes per round trip, and the round
trip is mostly waiting. Raising the baud rate shortens only the 22 ms
that is line - at 921 600 the round trip would be ~28 ms rather than 47,
which is 8.6 kB/s and still latency-bound.

## The ring moved into the 512 K that was standing empty

The acquisition ring was 16 K of DTCM - 334 records at ten channels, five
seconds, and a terminal that stopped drawing for six overflowed it. AXI
SRAM, 512 K of it at 0x24000000, was entirely unused.

A `.buffers` section in the linker script and the ring is **256 K**:
26 214 records of one channel, 5349 of ten, and **DTCM fell from 39 % to
26 %** into the bargain. Nothing on an interrupt path touches it - the
main loop fills it a byte at a time and a command handler empties it the
same way - and with no DMA and the data cache off there is no coherency
question, which are the two things that usually make that region awkward.
NOLOAD, so a quarter megabyte of zeroes is not carried in the image:
flash is unchanged.

## The ADC clock is on the wire now

Asked what the converters run at, the answer had to be read out of
`main.c`: PLL2 at M2 N12 P2 off a 25 MHz HSE is a 150 MHz VCO and a
**75 MHz kernel**, and each ADC's DIV2 prescaler makes **37.5 MHz**. That
is a second answer of exactly the kind this tree deletes, so
`Board_AdcClockHz()` reads it out of RCC and the ADC's own CCR and the
clock op appends it. The board says 37.500 MHz. Every sampling time here
is quoted in ADC cycles, and this is what turns one into seconds.

## The TIM1 clock does not need the gates, and now carries five channels

2026-08-31. FOC wants its samples at a known point in the PWM period, and
that path already existed - `clock='tim1'` feeds the ring from the
injected sequence at `trigger`, through the same filter chain as
everything else. What was not obvious is that it needs no switching:
**MOE is a separate thing from the sync**, so arming the injected group
with the stage down still triggers at the PWM period.

| | measured |
|---|---|
| TIM1 clock, gates DOWN | **49 239 samples/s**, 985 records/s at accumulate 50 |
| triggers over 1.03 s | 50 749 - the 50 kHz period, one sample each |
| software clock, same channels | 1129 sweeps/s, and whatever the loop's scheduling did that second |

Forty-three times the rate, and jitter-free, for the price of arming the
sync. So there is no second timer to add: TIM1 runs whether or not
anything is switching, and it is the same clock the drive samples on.

**And it carries more than the phases now.** The injected sequence has
converted the DC link (ADC3 rank 2) and the NTC (ADC1 rank 2) since the
drive needed them, latched at the same instant as the triple, but
`Board_AdcPhaseSlot` mapped U and V and returned `phase[2]` for
everything else - silently wrong for any other channel, which is why the
configure refused all but the phases. `Board_AdcInjectedSlot` takes the
whole latched sample. Measured, all five: **49 300 samples/s, 0 dropped**,
NTC 42 072 codes and the DC link 26 039 beside the triple.

## Where the 115200 line actually went, phase by phase

Measured 2026-09-01 on the port itself, no broker, a four-record reply at
stride 55:

| phase | ms |
|---|---|
| write | 0.11 |
| board turnaround | 2.70 |
| reply body | 19.84 |
| **total** | **22.65** |
| t3.5 before each | 1.75 |

The reply body takes 19.84 ms against 19.88 ms of theoretical line time,
so **the wire is saturated while it streams** and the dead time is
4.56 ms. The Modbus frame ceiling is 92.8 %: 220 of every 237 bytes on the
wire are records.

Three things were costing the rest, largest first.

**Every transaction waited out 8 ms of silence.** Nothing in a Modbus
reply says where it ends, so `QUIET_TIME` was paid at the end of each one.
A DAQ read's length IS knowable - the first payload byte is the count and
the stride is in hand - so the transport stops on the last byte. 43 % to
56 % of the line.

**`CMD_LINK_SHARE_PCT` was 33, measured before that.** Re-measured: 75.

**Eager reads were a feedback loop.** Sampling and the Modbus handler
share `main()`, so every read steals acquisition time, which leaves one
record per read, which needs more reads. Bottom of it: 95 reads/s at 1.00
records each. The reader waits a COMPUTED time now - the shortfall over the
rate it is seeing. Two guesses were measured and backed out: a fixed 20 ms
gave up just short of four records (1.55 a read), and counting a zero
backlog with a fixed 50 ms took 140 records/s down to 76.

| board ahead | rec/s | reads/s | rec/read | kbit/s | % line |
|---|---|---|---|---|---|
| 2x | 104.7 | 26.2 | 4.00 | 61.0 | 53 % |
| 4x | 124.7 | 31.2 | 4.00 | 72.6 | 63 % |
| 6x | 135.2 | 33.8 | 4.00 | 78.7 | 68 % |

**Not the optimiser.** The Release preset puts every file at -O2 and moved
the figure from 45.4 to 44.4 kbit/s. The hypothesis is dead.

**Not the broker either.** Direct port 162.8 records/s, through
`open_session` 162.1 - 0.4 %.

## The board spent 72 % of its loop watching the UART

Measured 2026-09-01: the board made **477.4 records/s with the link idle
and 133.1 while serving it**, so a link that could carry 194 was fed by a
board that could no longer make them.

`u_put` spun per byte waiting for TXE. A 229-byte DAQ reply is 19.9 ms of
line time and the acquisition loop spent all of it waiting. The spin polls
the acquisition now, and it is safe because of WHERE it is: TXFNF means
the FIFO has room, so the wait only happens when the FIFO is FULL -
sixteen bytes, 1.39 ms still queued against a channel read of about 78 us,
so nothing opens a gap past t1.5 inside the frame. **The loop went from
380 to 1880 sweeps/s.**

With that headroom: 144.5 records/s, 4.00 a read, 84.2 kbit/s, **73 % of
the line**, ring flat and nothing dropped, with the chain running.

## A chain costs sweeps, and the passband was lying

Decimating by N spends N sweeps of the loop for one record, and those
sweeps come off the link. Measured at ten channels, stride 55: the chain
at ratio 3 moved 44.5 kbit/s, at ratio 1 moved 48.8, and the clock-closed
window with no chain at all moved 72.1.

And `bessel.design` set its passband from the rate ASKED for, not the one
the integer ratio produces. **Asked 400 records/s off a 288 Hz loop, the
chain made 144 and put -3 dB at 80 Hz against a Nyquist of 72** - buying
throughput by moving the cutoff past the fold it exists to stop. It
follows the achieved rate now.

**A closed loop was tried and measured worse.** Three rounds of "run it,
measure what the board and the link did, design again" settled at 43 %
where a fixed ask holds 65 %: the loop rate has to be read while the
reader is still reaching its pace, so it overstates - 2780 sweeps/s
against the 635 a stream settles at - and every round over-decimates. A
window long enough to be honest costs ten seconds of startup.

## A rung change was visible in the data

`take_rung` zeroed every channel's filter state, so a step the board took
to keep up appeared in the measurement as every channel falling to zero
and climbing back. `filter_prime()` solves each section at DC and sets the
two states that hold it there.

Measured over **2856 records and 8 rung changes**: the four largest steps
are 13854, 2112, 436 and 59 codes at records 1 to 4 - the task's own
settling, which has no earlier value to be primed from - and after that
nothing exceeds 59 codes in a run spanning 36742.

## The stand-in was its own benchmark, twice

Emulating a fast link needs the stand-in to charge time for its bytes.
Two things made a measurement against it measure the simulator instead.

**`time.sleep` cannot honour a sub-millisecond wait on Windows.** A reply
at 10 Mbit/s is 292 us; slept per reply it spent 2.878 s of a 4 s run.
Banked and paid past 2 ms: 46 % to 84 % of the line.

**Inventing the data cost more than decoding it**: 401 200 gauss and
361 080 randint calls in four seconds, top of the profile, with the
library's own decode not appearing at all. Noise drawn once into a pool of
1021 and cycled: 84 % to 87 %.

The host stack, through the full library path:

| baud | rec/s | reads/s | rec/read | kbit/s | % line |
|---|---|---|---|---|---|
| 115200 | 196 | 39 | 5.00 | 115 | 100 % |
| 921600 | 1544 | 309 | 5.00 | 902 | 98 % |
| 3000000 | 4869 | 974 | 5.00 | 2844 | 95 % |
| 10000000 | 14868 | 2974 | 5.00 | 8683 | 87 % |

At 10 Mbit/s that is 336 us a transaction of which 292 is line, so 44 us
is host. **The library is not what limits a fast link.**

## A lingering broker halves the bench, and a held one serves nobody

`test_bench`'s `round_trips_per_s` read 31.2 against a 64.2 baseline,
reproducibly, and nothing in the tree had changed: a broker left over from
an earlier run was sharing the port, and a second client on the segment
halves the round-trip rate. With the port unshared: 64.2 of 64.2. A
measurement taken while something else drives the same bench is not a
measurement.

`session.py --force` stood a broker down that had **no clients** and the
tool said so first. Forcing one that a session holds takes that session
with it - which happened once here, mid-view, and produced a traceback in
`broker.py:_ask` that looked like a library fault and was not.

## The rotor observer's limit is acceleration and a bandwidth window, not speed

`tools/observer_run.py` builds `drive/` with the host gcc and runs the
firmware's own observer against `coaxial.motor.PLATINUM_5230SL` - the
estimated 5230SL, propeller loaded, 37 V link, currents only. Model
arithmetic, not a measurement: R, Ld, Lq and J are size-class estimates
and the saliency is what an injection observer lives on.

| iq A | rpm | f_e Hz | PWM/rev | start deg | run deg | |
|---|---|---|---|---|---|---|
| 5 | 1805 | 421 | 119 | 1.5 | 1.5 | locks |
| 20 | 3776 | 881 | 57 | 3.0 | 3.0 | locks |
| 35 | 5051 | 1179 | 42 | 4.1 | 4.1 | locks |
| 45 | 5493 | 1282 | 39 | 12.1 | 12.1 | locks |
| 50 | 27 | - | - | 180 | - | stalls at the handover |
| 58 | -80 | - | - | 180 | 178 | reverses |

**Torque at standstill is the limit, not speed.** 45 A holds
(230 krad/s^2 electrical); 50 A never reaches the 984 rad/s crossover.
Traced millisecond by millisecond at 58 A: the injection lock is excellent
(0.014 rad), the rotor accelerates correctly for 2 ms, the angle error
grows ~80 deg/ms past 90 deg and the torque reverses. `eps` bounces of
order 1 rad - the demodulator is swamped while the rotor leaves the
injection region faster than it can follow. The board's 100 A rating and
the motor's 112.5 A are both far above this: **a q-current step is the
wrong way to start this machine**, which is what the I/f ramp is for.

**The PLL has a window, at 35 A**: 60 Hz 11.0 deg, 150 Hz 4.0, 332 Hz 4.1,
and 600 Hz and above lose the rotor at the handover. The Kalman fixed
point (332 Hz, `sensorless.kalman_gains`) sits at the top edge of what
works, not in the middle of it.

Two wrong turns worth not repeating. **The polarity pulse was missing**
from the tool, not from the firmware: injection locates the d axis and
only saturation says which end carries the magnet. Adding
`drive_set_mode(POLARITY)` before the run is what made 60 Hz lock at all.
It did **not** rescue 50 A or 58 A - the failure there is the one above.
**One worst-case figure hid the answer**: the handover transient and the
steady tracking are different quantities, and a single number read as "the
observer is bad at 40 A" when it meant "the start is hard and the run is
fine" - 40 A peaked at 34.9 deg and still reached 5413 rpm. A stalled
rotor also scored 0.00 deg steady, having never been sampled once.

## The controller schedule over the link sweep, and the sensorless floor

`tools/montecarlo.py` runs the firmware's compiled control law - loop,
demodulator, observer, dead-time table - against plants drawn around the
5230SL's estimates and a stage drawn around `coaxial.inverter`'s, 16 draws
a candidate, cost `sigma_theta + speed_err + 10*trip` scored mean + p90.
6 240 runs over 23-63 V (`notebook_examples/foc_montecarlo.ipynb`, 164 s at
16 processes; 0.21 s a run through ctypes). Model arithmetic end to end.

* **The injection optimum is volts, not a fraction**: 2.0-4.9 V across the
  whole sweep while the knob (a fraction of Vdc/sqrt 3) falls from 0.20 to
  0.05. The observer sits at 54-78 Hz at every link voltage.
* **Zero trips in 240 verification runs** on fresh plants, worst angle
  error p90 0.27-0.56 rad, speed error under 2 % rms.
* **The sensorless floor is the descent**: with injection cut at the top
  of the hold, the back-EMF observer alone loses the rotor at a median
  24-69 rpm (p90 36-175, worst at 63 V because the same one-second descent
  falls from a higher top). With injection left on, every descent reached
  rest locked. `sensorless.crossover`'s envelope (margin 3, 20 % residual)
  says ~190 rpm - the optimised blend holds 3-5x lower.

## One numpy scalar in a job took down two pools, then a machine

The Monte Carlo's round 2 died with `OSError 1455` / OpenBLAS allocation
failures in the workers while round 1, the same code, always survived.
Ruled out first: worker count (61 vs 16, both died), pandas in the workers
(only the parent imports it), the DLL build (loads fine). The cause:
round 2's knobs are read out of a pandas `score()` row, so they were
**numpy.float64**, and unpickling ONE such scalar imports numpy - so every
worker imported numpy+OpenBLAS in the same instant, and the simultaneous
buffer-pool commit spike exhausted a machine with no page-file headroom.
The threadripper went down for good during exactly this. Fix in
`montecarlo.py`: jobs carry builtin floats only (`float()` at both places
a DataFrame row leaks into a job), `OPENBLAS_NUM_THREADS=1` in the worker
initializer as the backstop, and one pool per session instead of one per
round - respawning 61 interpreters three times was its own commit spike.

## gemma4:12b stopped loading on the laptop; the blob was not the fault

`test_live_model.py` crashed on this machine twice: first `CUDA error: a
PTX JIT compilation failed` (0xc0000409 in llama-server), then, retried,
`Failed to load CLIP model from ...sha256-675ad...`. Corruption is ruled
out: the named blob was deleted and re-pulled, came back byte-identical
(175 115 584 B, same digest), and failed the same way - so the fault is
ollama 0.33.1's loader/CUDA build against this RTX 4060 Laptop (8 GB,
driver 591.66), not the download. `llama3.1:8b` runs correctly on the same
daemon, GPU included, and `capability` picks it for this machine. A winget
upgrade of ollama was cancelled mid-flight; the GPU driver update was in
progress when this was written - re-run `test_live_model.py` after either.

## The stand-in could not rehearse an identification, for four reasons

Found preparing the auto-tune notebook: driving the stand-in's rotor and
recording it through the DAQ front door - the exact flow a real motor will
get - produced currents 17x off, duties flat at zero, and an omega in
megaradians. All four causes were the stand-in's, none the flow's:

* **`Coaxial63100.open()` was not idempotent.** `daq.open()` opens the
  device it belongs to, so `device.open()` followed by `daq.open()` built
  a SECOND SimulatedSession - commands went to one board's drive while
  records came from the other's, whose rotor never moved. Ruled out
  first: the wiring (`daq.drive` was correct on both). open() now keeps
  a live session; on a real port the second open was a collision anyway.
* **Three rest-point tables for one channel.** `NOMINAL` (analog + DAQ
  records: U 900, V -8650), `SimulatedDrive.CENTRE` (moments: 1400,
  -8030) and `CENTRE_DCBUS_V` (31.0 against a DC code reading 24.8 and a
  drive reporting 24.0). `offsets()` tared through the moments and the
  frame subtracted it from records centred 500 codes away: -3.2 A of
  phantom on every phase. One table now (`NOMINAL`, the documented
  reference-board values) and one link voltage (`DCBUS_V`, the rest code
  through the divider).
* **Stamps at the sweep, production at the line.** A free-running task
  stamped every record 47 us apart while the emulated line paced ~300 a
  second: half a second of run spanned 9 ms of timestamp, and the omega
  differentiated off a frame came out in megaradians. Records now span
  the wall time their batch covered, floored at the sweep cost; a
  clock-closed config keeps its interval, as the board does.
* **The record's u8 duties are a real limit, not a bug.** 1/255 of a
  24.8 V link is 97 mV a leg, against omega L i terms of tens of mV - an
  identification off recorded duties is quantisation-limited at low
  modulation ON THE REAL BOARD TOO, and correlated within a steady
  segment, so it does not average away. The commissioning's window path
  (the controller's own vd/vq, unquantised) is the sysid backbone;
  `sysid.from_frame` earns its keep at high modulation, or when a shaft
  angle rides in records (TODO 0).

The suites could not have caught the first three: invariant 10 keeps
expected physical values out of the tests, and every value here was
plausible alone - only closing the identification loop, which needs all
of them at once, showed them. `test_daq_api.py` now closes two of the
loops (idempotent open, stamps against the wall).

## Where the write-class transaction's 15 ms goes, and the 8 ms that left

Anatomy of a small request at 115200, off the 2026-09-01 phase
measurements and the two ends' own constants - the class `CLAUDE.md`
quotes as "a compare write lands in 15 ms":

| phase | ms | owned by |
|---|---|---|
| host t3.5 before TX | 1.75 | the spec: fixed above 19200 baud |
| request on the wire | ~0.3 | the line |
| board t3.5 + loop + parse | ~2.7 | the spec again, plus the poll |
| reply on the wire | ~0.5 | the line |
| host QUIET_TIME after the last byte | **8.0** | nothing - the frame was whole |

The 8 ms was the price of not knowing where a reply ends, and for the
`u8 took` class it IS knowable: `1` alone, or `0` and a length-prefixed
refusal. `transport.ACK` is that shape, `Subsystem._ack` passes it, and
all 24 pure-ack ops go through it - audited against every handler in
`comms/src/cmd_*.c` (23 write only through `cmd_took`; the bypass op
writes a bare 0/1, whose refusal degrades to the quiet read and nothing
worse). An exception frame is now sized under any shape: one code byte.
Arithmetic says ~7 ms a write; the bench re-measures the day a cable is
back. NOT taken, again: stopping on a valid CRC - measured at one false
frame end in 4096 prefixes (above), and the VCP's per-byte chunking makes
nearly every byte boundary a candidate.

What remains is the spec's, not slack: two fixed 1.75 ms silences per
transaction and the board's parse-at-the-loop. Both ends are this
repository's, so a shorter t3.5 on a closed link is possible - but it is
a framing constant on real silicon, and nothing here changes framing
without a scope on the wire and a bench to prove it.

## The examples became notebooks, and two more stand-in defects fell out

Porting the eleven `# %%` examples to executed notebooks ran every one of
them against the stand-in - several for the first time, since their
`SIMULATED` default was False and the bench always had the board. Two
defects only that execution could show:

* **The virtual rotor diverged at poll cadence.** `_advance_model` took
  ONE Euler step over the wall-clock gap between reads; the mechanical
  constant j/b of the placeholder profile makes (1 - dt b/j) = -4 at a
  0.2 s poll, and rotor_observer_session read +1896, -5770, +24964 rad/s
  on consecutive polls. Substepped now - each slice a tenth of the
  constant - and the same 0.5 A settles at the friction equilibrium,
  367 rad/s electrical, whatever the cadence.
* **`SimulatedDrive.state()` lacked the MINOR 2 appendix** (`cycles`,
  `exit_ticks_max`): a KeyError on the stand-in for code the board
  answers. The parity suite holds the two together but needs a board to
  run; an example executed dry is what caught it.

The structure suite parses notebook code cells as modules now (BESIDE),
so the AST net that once caught a stale `daq.read` in
gate_drivers_session still covers the examples in their new form -
523 checks, 12 of them the notebooks'.

## Motion at link rate: what a 7 ms write can and cannot close

Building `coaxial.motion` against the stand-in's rotor, three control
lessons, each measured before it was believed:

* **A per-pass position loop pumps the resonance it cannot see.** The
  load-angle spring on the placeholder profile rings at ~37 Hz; the link
  corrects at ~25. P alone: six converging passes, then a pole slip into
  a 50 rad/s freewheel. PD: worse (the derivative of an aliased signal).
  The shape that works is the closed-loop stepper's: slew smoothly,
  let the ring die, read the sensor as a short MEAN (the ring is
  symmetric about the load's equilibrium - one read froze up to its
  amplitude into the reference frame), correct what the load stole.
* **An overstated inertia is an overstated gain.** The velocity loop
  with j assumed five times the plant put per-pass loop gain at 2.5:
  +900 rpm asked, -1552 delivered, the sign alternating and doubling.
  Defaults are now the SMALLEST plausible machine - understating is
  merely sluggish.
* **Energize softly, count from the detent.** Full current onto an
  unknown rotor is a yank of up to half a pole that an underdamped rotor
  rides through pole after pole; ramped over six writes it detents and
  stays. The angle frame is incremental from there - absolute waits on
  an encoder-offset commissioning that does not exist yet.

And the stand-in physics that had to become real before any of it could
be tested: HOLD is a load-angle spring now (a stepper that follows,
rings, and slips), the lazy rotor integrates up to EVERY input change
(180 setpoint writes used to arrive as one 45-degree leap), sensorless
dq carries the back-EMF term (a power measurement read 0.34 W where the
shaft alone bore 34), `model_param` reaches the RUNNING motor as the
firmware's own model does, and the substeps are symplectic so the spring
rings instead of exploding. The shaft sensor and the DAQ records read
the same rotor the drive torques - one rotation, now seen three ways.

## The RS485 pair ran at eighty times the number in every report

Found 2026-09-02, the same hour the THVD1450's 50 Mbps rating went into
HARDWARE.md - reading where the runtime baud was applied, to add a
`link_baud` parameter, found nothing applying one.

**Believed**: "CubeMX carries 9216000 baud on the RS485 pair. The
firmware sets 115200 at init, as it sets the SPI word sizes"
(docs/HARDWARE.md, and `DEV_UART_BAUD 115200U` in comms_limits.h saying
"the runtime value is this one"). **True**: nothing ever wrote the
UARTs. `MX_USART2_UART_Init`/`MX_UART5_Init` left `Init.BaudRate` at the
.ioc's 9 216 000 and no board or comms code re-initialised either port -
the SPI word sizes ARE re-set by the drivers, the UARTs never were. The
wire ran at 9.216 Mbaud while `dev_uart_baud()` told the RTU timing, the
link report and `cmd_link_records_per_second` 115200.

**Why nothing caught it**: `0x6E` device 2 op 0's echo check transmits
and receives on the SAME port at the SAME setting - an absolute rate
cannot fail a loopback against itself, and all four patterns came back
clean on both ports (2026-08-28, recorded above). No external RS485
master has ever been on the segment; every bench session rode USART3,
whose CubeMX value happens to be the same 115200 the firmware assumes.
The RTU silences were computed for 115200 (t3.5 = 1750 us) and applied
to a 9.2 Mbaud wire, where a whole frame fits inside one silence.

**Fix**: CAL_VERSION 9 adds `link_baud` (id 45, default 115200, bounded
9600..921600 at the write), and main() applies it to USART2/UART5 after
the record loads, before `link_init()` derives the RTU silences -
`dev_uart_port_baud(index)` now answers per port. USART3 is deliberately
outside the parameter: the debug probe stays the recovery path whatever
the record says. NOT yet verified against an external master - no RS485
master exists on this bench; the pilot-tone hardware day is the first
chance to see the pair at a known rate from the far end.
