# Coaxial BLDC Inverter (63 V / 100 A)

"Coaxial" is the mechanical stator mount, not cabling. Three consequences: the
board chokes thermally, so the NTC is a control input and not a diagnostic; the
phase sense sits inside a switching gate driver, so idle noise figures are
worthless; and 100 A is a peak SOA limit, not a continuous rating.

## The Board Itself

`electronics/` is the authority on what is fitted and how it is wired. This
document records what the firmware has to know and what was measured; it does
not repeat the netlist.

* **`Coaxial 63100 Schematics.pdf`** - 21 sheets. `IMU.SchDoc` and
  `AngleSensor.SchDoc` are the two SPI sensors, `Regulators.SchDoc` the rails.
* **`Coaxial 63100 BOM.csv`** - 380 lines, designator to manufacturer part.
* **`ENABLE_AFE` gates a regulator on `Regulators.SchDoc`**, the sheet that
  makes `+15V7`, `+5V7`, `+5`, `+3V3_ref`, `+1V65_bias` and `+3V3D`. U13 runs
  from `+3V3D`/`VDDIO`, U14 from `+5`. Measured: both stop answering with `PB2`
  low, which is what the `PB2` trap below is about.

## Schematic Names Are Not MCU Names

The sheet symbols are logical and zero-based; the nets on `MCU.SchDoc` carry
ST's peripheral names; the firmware and the board's own channel map use neither,
they use what the signal *is*. Three vocabularies for one wire, so the mapping
is written down once, here, and traced from the netlist rather than assumed.

| Sheet symbol | MCU peripheral | Part |
|---|---|---|
| `SPI0` (IMU.SchDoc) | SPI2 | U13 BNO085 |
| `SPI1` (AngleSensor.SchDoc) | SPI4 | U14 A1335 |
| `UART0` (RS485.SchDoc) | UART5 | U6 THVD1450 |
| `UART1` (RS485.SchDoc) | USART2 | U5 THVD1450 |
| none | USART3 | debug probe VCP, no transceiver |

Traced on `MCU.SchDoc`: net `UART2_RO` lands on sheet port `UART1`, `UART5_RO`
on `UART0` - the indices are not in peripheral order, which is exactly why
guessing them is not safe.

The RS485 sheet also names the transceiver's own pins `DI`, `RO`, `DE` and `RE`,
which are the part's, not the MCU's: `DI` is the MCU's TX and `RO` is its RX.

**The analog channels have a third set of names again.** The schematic numbers
the differential pairs `ADC0P/ADC0N` through `ADC5P/ADC5N` in pin order; the
firmware names each by what it measures - Phase U, Phase V, Phase W, Clevel,
NTC, DC bus, Cinj. Command `0x6D` kind 0 is the authority on which pin is which,
and nothing above the firmware should carry a second answer.

## Silicon & Clocks

* **MCU:** STM32H753VIT6, Rev V. The hardware revision is strictly necessary to
  support the 950 MHz VCO.
* **Clocks:** 475 MHz SYSCLK driven by a 25 MHz HSE. The ADC kernel clock is
  decoupled (75 MHz async), rendering sampling times immune to SYSCLK
  reconfigurations. No LSE/RTC.
* **HSE error, measured against UTC: -11.62 ppm** (900 s window, 1.11 ppm floor,
  2026-08-27). SYSCLK is therefore 474.994 MHz, not 475.000 - 7 ms of skew
  across a ten-minute capture, which is why a timestamp uses the rate
  `clock.sync()` measured and never `sysclk_hz`. The PLL is an exact ratio, so
  this is the crystal. Against UTC and not against the host, because the bench
  PC is not a reference either - FINDINGS has its offset and its rate, and both
  windows they were fitted over.

## Supply senses

Traced off the MCU sheet 2026-08-27. R113 is a 10 k array whose four elements
are GND, +5, +15V7 through R119 47 k, and GND:

| channel | pin | divider | ratio | expected at the pin | measured |
|---|---|---|---|---|---|
| `+5V` | PA4 | 10 k / 10 k off +5 | **2.00** | 2.50 V | 2.552 V, so 5.10 V |
| `Vgate` | PA5 | 47 k + 10 k over 10 k off +15V7 | **6.70** | 2.34 V | 0.052 V, so 0.35 V |

`Vgate` reads near zero with AFE_ON on, which is right: this board's gate is
inverted, so the drivers are unpowered exactly when the converter reference is
up. Both report `ADC_UNIT_NONE` - a unit is a promise that the scaling behind it
is in the calibration record, and these two dividers are not in it yet
(invariant 7), so a host reads volts at the pin.

`DAC0`/`DAC1`/`DAC2` are crossed out on the same sheet, so PA4 and PA5 are not
DAC outputs on this board however much the pin names suggest it.

## AFE & The `PB2` Trap

The internal ADC VREF is disabled. The reference is driven externally by the
AFE.

* **`PB2` (`AFE_ON`):** Powers the amplifier chains, the ADC reference *and both
  SPI sensors*. Polling channels with `PB2` low returns exact mid-scale
  (yielding a phantom 25 °C on the NTC).
* **A sensor without `AFE_ON` is worse than dead.** It still drives MISO, still
  resets, and still returns a valid 276-byte SHTP advertisement - so every read
  looks healthy. What it never does is act on a write: `Set Feature` starts no
  stream, and executable `ON`, `SLEEP` and `RESET` all produce the identical
  answer, which is only possible if none of the payloads arrived. Firmware
  refuses `Board_ImuInit` while `PB2` is low, and losing `PB2` clears the ready
  flag, because a part that has lost its supply needs a reset rather than a
  resume.
* **`PE15` (`nFAULT`):** Intended active low - high is normal, low is a fault.
  Measured, it tracks `AFE_ON` inversely and reads logic `0` when the AFE is
  powered, which by that intent means a fault is asserted exactly when the front
  end is on. The two do not agree and the conflict is unresolved - FINDINGS,
  Open Anomalies. The pin is also `TIM1_BKIN` now, so this decides whether the
  gate drivers can run at all.

## Safe Torque Off, and the pilot that unlocks it

Gate driver supply is not the MCU's to switch. It is released by a safety chain
on `STO.SchDoc` that watches for a **common-mode pilot tone the master injects
on the RS485 pair** - not something this board generates. The board detects it,
and the chain is a dead man's switch: leaky integrators with a charge and a
discharge path, so the tone has to keep arriving or the level decays and the
supply drops.

Extraction off the pair, on `RS485.SchDoc`:

| Stage | Parts | What it does |
|---|---|---|
| Common-mode tap | R36, R37 (10 kΩ each) across A1/B1 | Two equal resistors to a midpoint - the differential data cancels, the common mode does not |
| Coupling | C75 (33 nF, 100 V) | DC blocked; only a tone gets through |
| Band pass | R44 15 kΩ, R41 3.30 kΩ, R45 33.0 kΩ, C98/C99 1.2 nF | 1 kHz to 10 kHz, per the sheet's own note |
| Clamp | D3, D4 | 0.7 V, −0.35 V to 0.75 V at +IN |
| Detector | **U16 TLV3492**, dual nanopower comparator | Two thresholds, both with hysteresis |
| Charge pump | R54, C101 (47 nF), D11/D12 | The "Charge Path" and "Discharge Path" the sheet marks |

The comparators are wired as two independent level detectors, each with positive
feedback: comparator A takes its threshold from R73/R87 and its hysteresis from
R122 (18.0 kΩ); comparator B takes its reference from R86/R88 and its hysteresis
from R123 (3.01 MΩ). Hysteresis is what keeps a noisy pair from chattering the
chain.

**Reading, not extracted:** with one comparator above the tone and one below,
only a signal that alternates works both outputs, and only both outputs working
keeps the pump charging. A common mode stuck at any DC level satisfies at most
one and pumps nothing - which is what makes this a liveness test rather than a
level test. Read off the netlist and the values, not off the drawing: the sheet
is a PDF with no renderer here.

Then on `STO.SchDoc`: **CINJ** in, two leaky integrators, a TPS3840PL30
supervisor with a timing capacitor, and an NL7SZ97 configurable gate producing
**KEEPALIVE** and **FAULTOUT**. **CLEVELOUT** is the integrator level brought
back out.

Two places on that sheet are marked *Bypass safety system - No BOM component*.
The bypasses are deliberately unfitted.

### What the MCU can see of it

Both ends of the chain are already ADC channels, and both have sat in the
channel table as `ADC_UNIT_NONE` with nothing said about them:

| Channel | Pin | Is |
|---|---|---|
| `Cinj` | ADC3 IN11, PC1 | The recovered pilot, off the detector |
| `Clevel` | ADC2 IN5, PB1 | The integrator level - how near the chain is to dropping out |

`Clevel` is the useful one: it is the margin. `PE15` is `TIM1_BKIN`, labelled
**(STOP)** on the MCU sheet, so a fault stops the gate drivers in hardware
without the firmware being involved. Neither channel can be read by asynchronous
single shots - see FINDINGS.

### Two conditions, not one

The chain needs both, and they are independent:

| Condition | Source | Spec |
|---|---|---|
| **Pilot tone** | the master, on the RS485 pair | 3-15 kHz, 5 kHz nominal, amplitude-windowed: ON 0.7-2.1 V, off below 0.6 V **and** above 2.2 V (`electronic_simulations/sto/sto.asc`) |
| **KEEPALIVE** | this board, PA10 | a square wave into a charge pump |

Windowing the pilot at both ends means a stuck-high rail reads as "off" exactly
like silence does.

`KEEPALIVE` is PA10 through R72 330 Ω into C71 100 nF and on to D10/D14/D15
- a **diode charge pump**, so only edges deliver anything and a held level is
  worth what a stopped CPU is worth. R48 18 kΩ pulls it down when the MCU
  releases the pin. It feeds `VLATCH`, which gates `DCDC_ENABLE` and so the gate
  driver supply itself.

**No timer may generate it**: a free-running timer keeps toggling after the
firmware hangs, which is the one thing the chain exists to catch.
`Board_StoKeepalive()` runs at the top of the main loop, above every branch -
`link_active()` does a `continue`, so anything below it stops the moment Modbus
gets busy.

Measured rates, same binary:

| Load | toggle | square wave |
|---|---|---|
| Idle | 214 kHz | 107 kHz |
| Host polling Modbus | 124 kHz | 62 kHz |

The sim drives its `MCU_PWM` at 100 kHz, so the loop lands on the dimensioning
without anyone tuning it. **Both figures are means.** The worst case is what
decides: a 276-byte SHTP cargo at 1.48 MHz is 1.5 ms, 320x the idle half-period.
What `VLATCH` tolerates is not yet measured.

## The gate drivers, and why the dead time is 30 ns

TIM1 centre-aligned, ARR **2375** off 237.5 MHz = **50.000 kHz** exactly, RCR 1,
CKD DIV1 so one dead-time tick is **4.2105 ns**. `BDTR.DTG` = **19** = **80.0
ns**.

The gate drive is resonant, not a plain RC. Per FET, off the schematic:

HO --[R9 0.47R]--[L6 120nH]--+-- gate  (D5 Schottky across R9) +-- D1 CDZV15B 15
V clamp +-- R7 4.99R + C7 3.9nF -- source

**R7/C7 is a damper to source, not the gate resistor.** The gate path is 0.47 R
+ 120 nH into C_gs 5.48 nF; the damper takes the resonance without slowing the
  DC drive, which is why the numbers below barely move with load.

Where 80 ns comes from, simulated on the models in `electronic_simulations`
(`IAUCN10S7N021` VDMOS, `LQW18CAR12J00D`, `2EDL8034F5.lib`). The criterion is
gate overlap - when the outgoing gate crosses V_th against when the incoming one
does - because that is independent of the power loop inductance, which sets how
big the shoot-through current gets but not whether there is one.

| V_th | V_DD | LS off -> HS on | HS off -> LS on |
|---|---|---|---|
| 2.2 V | 14.9 V | 49.9 ns | 56.3 ns |
| 2.2 V | 16.5 V | 52.1 ns | **59.4 ns** |
| 2.8 V | 15.7 V | 44.3 ns | 50.3 ns |
| 3.4 V | 14.9 V | 36.3 ns | 41.0 ns |

Tj 125 C, 100 A, 63 V. Over +/-100 A the spread is 1.5 ns and over 27->125 C it
is 1 ns, so one fixed DTG is enough - no adaptive dead time.

59.4 ns   worst-corner gate overlap
   + 6.0 ns   TDMOFF max, 2EDL8034 (the absolute 50 ns delays are common to both
     channels and cancel to within the matching spec) ------- 65.4 ns  floor
     80.0 ns  DTG 19, 22 % over the floor and exactly on a code

**The driver has no interlock.** 2EDL8034 datasheet p.1: *"Independent inputs
allow controlling high- and low-side domains independently."* HI and LI may both
be high and it will drive both gates. `BDTR.DTG` is the only shoot-through
protection on this board, which is what makes `Board_PwmSetAll` all-or-none and
DTG safety-critical rather than a tuning knob.

**It has no fault pin and no enable either** - PG-DSO-8, and the eight are VDD,
HB, HO, HS, HI, LI, VSS, LO. `nFAULT` on `PE15` comes from the STO chain, not
from the drivers. UVLO is 7.3 V rising / 6.7 V falling on VDD and 6.3 / 5.7 V on
VHB-HS.

**Minimum pulse.** TPW is 40 ns - a shorter input pulse changes nothing at the
output - which is 10 ticks. With DTG 19 the smallest high-side pulse that exists
is 29 ticks, about 122 ns, so `CCR >= 15` of 2375: **0.63 % duty**. That is the
floor for low-speed saliency injection, not a rounding detail.

## Sensors on SPI

Both are polled by the main loop into shared memory; the host reads that and
never drives a bus while a loop runs. Both die without `PB2`. What is fitted
comes from the board - `0x6D` kind 4 - not from this table.

| Part | Bus | Pins | Frame | Reads |
|---|---|---|---|---|
| BNO085, 9-axis IMU (U13) | SPI2, mode 3, 2.97 MHz | `PB12` CS, `PB13` SCK, `PB14` MISO, `PB15` MOSI, `PD8` H_INTN, `PD9` PS0/WAKE, `PD10` NRSTN, `PD11` BOOTN | SHTP, 8-bit stream | up to 394 rotation vectors/s, Q14 |
| A1335LLETR-T, magnetic angle (U14) | SPI4, mode 3, 1.86 MHz | `PE2` SCK, `PE4` CS, `PE5` MISO, `PE6` MOSI | 20-bit packet as four 5-bit words | ANG, STA, ERR, XERR, TSEN, FIELD; 12 bits each |

* **The A1335's answer lags one frame.** The address arrives on MOSI bits 17..12
  while MISO has already shifted out bits 19..16. Measured: asking TSEN, FIELD,
  TSEN in turn returned the previous register's value every time.
* **Mode 3 comes from the driver, not from the .ioc.** `Board_ImuInit` and
  `Board_AngleInit` set every field of `hspi2`/`hspi4` and call
  `HAL_SPI_Init` themselves, so what CubeMX generated is overwritten before
  either part is spoken to. Both were set to mode 3 in CubeMX on 2026-08-29
  and now agree on that. The rest of the .ioc is still dead: it says 8-bit
  where SPI4 runs 5-bit words, and 7.42 MBit/s where the driver derives
  1.86 MHz from the kernel clock.

* **SPI4 cannot carry a 20-bit word.** `IS_SPI_HIGHEND_INSTANCE` names SPI1,
  SPI2 and SPI3 only, and `HAL_SPI_Init` returns `HAL_ERROR` above 16 bits on
  the rest. Four 5-bit words under one chip select put exactly twenty clock
  edges on the wire, which is what the part counts.
* **The A1335's register map is not in the datasheet here** - it defers to the
  Programming Manual. Addresses come from
  `github.com/ScranchNew/Allegro-A1335-Sensor-library`, and `0x6E` device 1 op 5
  sets which one the poll loop reads, so a better address needs no rebuild.
* **`FIELD` says whether the angle means anything.** Measured 3 gauss with no
  magnet in front of the part, and the angle is then noise; the views say so
  rather than drawing a confident pointer.

## Scaling & Calibration

The board reports raw codes and judges nothing. It also carries the arithmetic
that turns a code into a quantity, and a record a rig can correct - a
calibration belongs to one physical board, and a host holding it answers for the
wrong board the moment it is pointed at a second. `0x6E` device 3 is the record;
PROTOCOL.md is the wire.

Every figure below was traced off `electronics/` on 2026-08-26. **None has been
measured.** They are the record's defaults for exactly that reason.

| Channel | Chain | Result |
|---|---|---|
| **DC link** (`PC0`) | R12 49.9 kΩ / R11 2.2 kΩ (ERJ-PB3B, 150 V ±0.1 %), C117 680 pF, R42 15 Ω | 78.15 V full scale - 24 % over the 63 V rating, and deliberate (invariant 11) |
| **NTC** (`PB0`) | R100 10.0 kΩ (ERA-3AEB103V, 0.1 %) to `+3V3_ref`, NTC1 NCU18XH103**D60**RB to GND, C116 15 nF | ratiometric, so VREF cancels. The `D60` suffix *is* B₂₅/₅₀ = 3380 K. ~0.2 °C parasitic self-heating |
| **Phase U/V/W** | RU1‖RU2 in the phase conductor - two Vishay `WSHM28187L000FEA`, 7 mΩ each, 3.5 mΩ parallel - tapped by RU3/RU4 49.9 Ω into a THS4551, Rg 330 / Rf 1.5 k | 4.5455 V/V × 3.5 mΩ = 15.909 mV/A. 207.4 A full scale, so 100 A sits at 48 % of the differential span |
| **Reference** | U2 REF2033 drives `+3V3_ref` **and** `+1V65_bias` | one part sets VREF and the differential mid-point, which is why they track |

**The phase channels are current, not voltage.** The sense element is in the
phase conductor. Firmware reports them in milliamperes off its own channel map;
nothing above it carries a second answer.

* **The gain is bounded, not just traced.** 100 A across 3.5 mΩ is 350 mV, and
  3.3 V over that is **9.43 V/V** - a chain with more gain than that could not
  represent the board's own rating at all. The THS4551's 4.5455 fits; the ×18.5
  and ×10 an Altium net dump appeared to put on the ADA4891 quad do not, by two
  orders of magnitude. **What that quad does in this path is unresolved** -
  buffer, level shift, or the `PH_CURR` protection tap on `Inverter.SchDoc`. It
  is not further gain into the ADC.
* **Nothing here has been measured.** Both numbers come off a PDF. Zero and span
  each phase against a clamp meter before believing an ampere from this board -
  that is what `0x6E` device 3 exists for.
* **Phase V Anomaly:** the 0.85 V offset on the reference board is a localized
  op-amp failure. In amperes it reads as -52 A with nothing connected. `zero`
  would make that number go away and the fault with it - **do not zero Phase V
  on this board.** Zeroing is for a channel's own offset, not for broken
  silicon.
* **Zero before span.** Spanning an un-zeroed channel folds the offset into the
  gain, which then reads right at the reference point and nowhere else.
* **Span is refused where the conversion is not linear in the code** - see
  PROTOCOL.md, device 3.

## I/O & Link

Three Modbus ports, one slave. A request is answered on the port it arrived on;
the unit id belongs to the board, not to the wire.

| Port | Pins | Transceiver | Receives | Console |
|---|---|---|---|---|
| USART3 | `PB10` TX, `PB11` RX | none - debug probe VCP | interrupt | yes, or Modbus |
| USART2 | `PA1` DE, `PA2` TX, `PA3` RX | U5 THVD1450 | interrupt | no |
| UART5 | `PC8` DE, `PC12` TX, `PD2` RX | U6 THVD1450 | interrupt | no |

* **Direction control is the peripheral's.** Both RS485 ports run
  `HAL_RS485Ex_Init` with hardware DE - no pin to toggle in software, no window
  where the driver is late.
* **Both hear themselves.** RE is tied to GND on U5 and U6 (RS485.SchDoc: pin 2
  on the GND net), so the receiver stays on while DE drives. Every byte
  transmitted returns, and `put()` purges afterwards. Measured: `0x6E` device 2
  op 0 sends 00, FF, 5A, A5 on a port and all four come back on both - which is
  also the check that the driver, the receiver and the wiring between them work
  with nothing else on the segment.
* **All three receive on interrupt, each byte carrying the tick it arrived
  at.** RTU delimits by silence, and polling from the main loop timestamped a
  byte when the loop reached it - a 276-byte IMU cargo is 1.5 ms, seventeen
  characters at 115200. The ring holds a whole frame; `ring_dropped` is non-zero
  if the loop ever stopped draining.

  USART3 was the exception until 2026-08-29, on the reasoning that the master
  on it is a person or a script rather than a bus. Measured, that cost 0.45 %
  of frames: 1393 requests, 7 silent, and `char_overrun` +7 to match. FINDINGS
  has the working.
* **No RX FIFO on any of the three** - `HAL_UARTEx_DisableFifoMode` is called
  on each. The receiver holds one character, 87 us at 115200, so a single
  overrun drops the frame and a latched ORE ends reception until ICR clears
  it. That is what makes the interrupt not optional.
* **CubeMX carries 9216000 baud on the RS485 pair**, which is not a Modbus rate
  on any bus. The firmware sets 115200 at init, the same way it sets the SPI
  word sizes rather than trusting them.