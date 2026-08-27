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
* **`.port` Read as a COM Port:** `SimulatedSession.port` is a bus label (`AX`), never `None`. `link_diagnose` guarded on `configured is None`, so a session that fell back to the stand-in spent 15 s on an SWD probe and then reported "Configured port AX: not among the ports above - the cable may be unplugged". The `simulated` marker that fixed the same mistake in `_interface` already existed. Step 4's closing advice also opened with "Powered" whatever step 1 concluded.
* **`check_power` Discarded Its Own Reading:** With no target the programmer takes 30.3 s - a second connect attempt at 8 MHz - against a 15 s budget, and `TimeoutExpired.stdout` already holds `Voltage: 0.00V`. The handler returned `None`. Now parsed from the killed run.
* **`-Ask` Pinned the Card:** A one-shot was exempt from the unload on prompt exit while the same script passes `--keep-alive 30m`, so four smoke tests left 8.4 GB resident with nobody at the prompt. `-Ask` now takes a list: one load, N questions, one release.
* **`--sections` Read Only Under `--match`:** `run_tests.py --live --sections tools` ran all three - `tools` and `all` both returning 176 checks in 255 s. Coverage tiers were unaffected; they set the sections directly rather than through the flag.
* **Model Hallucinations:** `llama3.1:8b` fabricates telemetry when tools fail and invents physical constants (NTC B=3950 instead of 3380). `gemma4:12b` remains the default; it is slower but declines to lie.

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

  Explained, and the earlier explanation was wrong. It blamed an unpowered
  open-drain output on the gate drivers. **A 2EDL8034 has no fault pin** -
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

Three bugs, all mine, all found by comparing two paths that must agree
rather than by reasoning about hardware.

| Symptom | Cause |
|---|---|
| injected triple read (-31344, 24587, -32355) where the meter read (1423, -8285, 392) | `Board_SyncOnInjected` cast JDR to `int16_t`. JDR is **offset binary**, 32768 = 0 V. Now goes through `Board_AdcDifferential`, which is the one definition (invariant 7). `board_adc.c` already carried a comment about this exact bug from an earlier session. |
| `pilot_ok` and `level_ok` false for every call ever made | `STO_ReadOne` passed `NULL` for `Board_AdcRead`'s `scaled` argument, which refuses a NULL before reading anything. |
| `Board_SyncArm` could never succeed | `Board_SyncReady` required `JSQR != 0`, but JSQR is written by `SYNC_ConfigPhase` **inside** Arm. Ready now means "a timer to trigger from" and nothing else. |

Measured after the fixes, AFE on, bridge off: injected (1433, -8136, 390)
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

## The STO interlock works, and the bridge cannot be enabled

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
channel by name: `index_of`, `channels()` and therefore `configure_daq` all
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

## Closed: the bridge trips were the bench supply's limit

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
that is not the bridge. Recovering needed the bench supply's limit raised -
about 200 mA at 24 V, which is roughly what the board draws running, so the
limit was tight rather than the board being damaged.

**Until this is understood, `bypass_break` is arming a power stage and not
a configuration flag.** `python_examples/daq_session.py` has the bridge off
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
conditional on somebody intending to arm the bridge, which changes what the
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
| off | **powered** - the bridge can switch | meaningless: every channel reads mid-scale |
| on | unpowered - nothing switches | valid |

**Switching and measuring are mutually exclusive on this board.** Nothing
below was taken with a live power stage, and nothing can be until the patch.

## The current path is consistent, with the bridge inert

AFE on, break bypassed, bridge enabled, sync armed. Duty swept 0-100 % equal
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
