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

The register map is not in `datasheets/AngleSensor` - that datasheet defers
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

