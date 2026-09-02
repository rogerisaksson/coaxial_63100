# Coaxial BLDC Inverter (63 V / 100 A)

"Coaxial" is the stator mount, not cabling. Consequences: the board chokes
thermally, so the NTC is a control input; the phase sense sits inside a
switching gate driver, so idle noise figures are worthless; 100 A is a peak
SOA limit, not a continuous rating.

## The board

`electronics/` is the authority on what is fitted. This file holds what the
firmware has to know and what was measured, not the netlist.

| | |
|---|---|
| `Coaxial 63100 Schematics.pdf` | 21 sheets. `IMU.SchDoc`, `AngleSensor.SchDoc` the two SPI sensors, `Regulators.SchDoc` the rails |
| `Coaxial 63100 BOM.csv` | 380 lines, designator to manufacturer part |
| `ENABLE_AFE` | gates a regulator on `Regulators.SchDoc` - the sheet making `+15V7`, `+5V7`, `+5`, `+3V3_ref`, `+1V65_bias`, `+3V3D`. U13 runs from `+3V3D`/`VDDIO`, U14 from `+5`. Measured: both stop answering with `PB2` low |

## Schematic names are not MCU names

Three vocabularies for one wire - the sheet symbols (logical, zero-based),
ST's peripheral names on `MCU.SchDoc`, and what the signal *is* (firmware,
the board's own channel map). Traced off the netlist, never guessed: net
`UART2_RO` lands on sheet port `UART1`, `UART5_RO` on `UART0`.

| Sheet symbol | MCU peripheral | Part |
|---|---|---|
| `SPI0` (IMU.SchDoc) | SPI2 | U13 BNO085 |
| `SPI1` (AngleSensor.SchDoc) | SPI4 | U14 A1335 |
| `UART0` (RS485.SchDoc) | UART5 | U6 THVD1450 |
| `UART1` (RS485.SchDoc) | USART2 | U5 THVD1450 |
| none | USART3 | debug probe VCP, no transceiver |

The RS485 sheet names the transceiver's own pins - `DI` is the MCU's TX,
`RO` its RX. The analog pairs are `ADC0P/ADC0N`..`ADC5P/ADC5N` in pin
order on the schematic; the firmware names them by what they measure -
Phase U, V, W, Clevel, NTC, DC bus, Cinj - and `0x6D` kind 0 is the
authority on which pin is which.

## Silicon and clocks

| | |
|---|---|
| MCU | STM32H753VIT6 Rev V - the revision the 950 MHz VCO needs |
| SYSCLK | 475 MHz off a 25 MHz HSE; ADC kernel clock 75 MHz async, so sampling times survive a SYSCLK change. No LSE, no RTC |
| HSE error | **-11.62 ppm against UTC** (900 s window, 1.11 ppm floor, 2026-08-27): SYSCLK is 474.994 MHz, 7 ms of skew over a ten-minute capture. A timestamp uses the rate `clock.sync()` measured, never `sysclk_hz`. The PLL is an exact ratio, so this is the crystal. Against UTC because the bench PC is no reference either - FINDINGS has its offset and rate |

## Supply senses

Traced off the MCU sheet 2026-08-27. R113 is a 10 k array: GND, +5, +15V7
through R119 47 k, GND.

| channel | pin | divider | ratio | expected at the pin | measured |
|---|---|---|---|---|---|
| `+5V` | PA4 | 10 k / 10 k off +5 | **2.00** | 2.50 V | 2.552 V, so 5.10 V |
| `Vgate` | PA5 | 47 k + 10 k over 10 k off +15V7 | **6.70** | 2.34 V | 0.052 V, so 0.35 V |

`Vgate` near zero with AFE_ON on is right: this board's gate is inverted, so
the drivers are unpowered exactly when the reference is up. Both report
`ADC_UNIT_NONE` - a unit promises the scaling is in the calibration record,
and these dividers are not in it yet (invariant 7); a host reads volts at
the pin. `DAC0`/`DAC1`/`DAC2` are crossed out on the sheet: PA4 and PA5 are
not DAC outputs here.

## AFE and the PB2 trap

The internal VREF is disabled; the AFE drives the reference.

* **`PB2` (`AFE_ON`)** powers the amplifier chains, the ADC reference *and
  both SPI sensors*. Low: every channel exact mid-scale, a phantom 25 °C on
  the NTC.
* **A sensor without `AFE_ON` is worse than dead.** It drives MISO, resets,
  and returns a valid 276-byte SHTP advertisement; it never acts on a write -
  `Set Feature` starts no stream, and `ON`, `SLEEP`, `RESET` all produce the
  identical answer, possible only if no payload arrived. `Board_ImuInit`
  refuses while `PB2` is low, and losing `PB2` clears the ready flag.
* **`PE15` (`nFAULT`)** is meant active low. Measured, it tracks `AFE_ON`
  inversely - logic `0` with the AFE powered - so by intent a fault is
  asserted exactly when the front end is on. Unresolved (FINDINGS, Open
  Anomalies). The pin is also `TIM1_BKIN`.

## Erratum: cutting AFE_ON makes STO read a-ok

**On this board, until modded.** The AFE can be brought up without the STO
chain by design; the fitted board does the reverse - `AFE_ON` low leaves the
STO detector reading **a-ok**, so the gate drivers arm with no pilot tone on
the pair. Deliberately useful: a bench run needs switching, the reference
sits on the AFE, and the two would otherwise be mutually exclusive.

Two statements in this tree therefore disagree and are both right: `0x6D`
kind 4 says the 2EDL8034 and the FETs are powered by the **STO chain** (the
schematic); `tools/show_session.py` says `AFE_ON` high removes their supply (this
board). The parts list is what stays true after the mod.

| While it stands | |
|---|---|
| Switching runs with `AFE_ON` low | every analog channel reads mid-scale, invariant 9 |
| The DC link is single-ended | mid-scale is **39.1 V**, not zero - FINDINGS |
| The phase shunts are differential | mid-scale centred is 0 A, which is also what unpowered amplifiers give |
| `Board_PowerPoll` refuses the rail while MOE is set | a sample mid-switch would drop the drivers with six inputs moving |

## Safe Torque Off, and the pilot that unlocks it

The gate driver supply is not the MCU's to switch. A chain on `STO.SchDoc`
releases it on a **common-mode pilot tone the master injects on the RS485
pair**; the chain is a dead man's switch - leaky integrators with a charge
and a discharge path, so the tone has to keep arriving.

Extraction off the pair, `RS485.SchDoc`:

| Stage | Parts | What it does |
|---|---|---|
| Common-mode tap | R36, R37 (10 kΩ each) across A1/B1 | the differential data cancels, the common mode does not |
| Coupling | C75 (33 nF, 100 V) | DC blocked |
| Band pass | R44 15 kΩ, R41 3.30 kΩ, R45 33.0 kΩ, C98/C99 1.2 nF | 1-10 kHz, per the sheet's note |
| Clamp | D3, D4 | 0.7 V, −0.35 V to 0.75 V at +IN |
| Detector | **U16 TLV3492**, dual nanopower comparator | two thresholds, both with hysteresis: A from R73/R87 with R122 18.0 kΩ, B from R86/R88 with R123 3.01 MΩ |
| Charge pump | R54, C101 (47 nF), D11/D12 | the sheet's "Charge Path" and "Discharge Path" |

Reading, not extracted: with one comparator above the tone and one below,
only an alternating signal works both outputs, and only both keep the pump
charging - a liveness test, not a level test; a common mode stuck at any DC
level pumps nothing. Then on `STO.SchDoc`: **CINJ** in, two leaky
integrators, a TPS3840PL30 supervisor with a timing capacitor, an NL7SZ97
gate producing **KEEPALIVE** and **FAULTOUT**; **CLEVELOUT** is the
integrator level brought back out. Two spots are marked *Bypass safety
system - No BOM component* and are deliberately unfitted.

### What the MCU can see of it

| Channel | Pin | Is |
|---|---|---|
| `Cinj` | ADC3 IN11, PC1 | the recovered pilot, off the detector |
| `Clevel` | ADC2 IN5, PB1 | the integrator level - the margin to dropping out |

`PE15` is `TIM1_BKIN`, labelled **(STOP)** on the MCU sheet: a fault stops
the gate drivers in hardware. Neither channel can be read by asynchronous
single shots - FINDINGS.

### Two conditions, not one

| Condition | Source | Spec |
|---|---|---|
| **Pilot tone** | the master, on the RS485 pair | 3-15 kHz, 5 kHz nominal, amplitude-windowed: ON 0.7-2.1 V, off below 0.6 V **and** above 2.2 V (`electronic_simulations/sto/sto.asc`) - a stuck-high rail reads "off" like silence |
| **KEEPALIVE** | this board, PA10 | a square wave into a diode charge pump: R72 330 Ω, C71 100 nF, D10/D14/D15, R48 18 kΩ pull-down. Only edges deliver anything; it feeds `VLATCH`, which gates `DCDC_ENABLE` |

**No timer may generate KEEPALIVE** - a free-running timer keeps toggling
after the firmware hangs, the one thing the chain exists to catch.
`Board_StoKeepalive()` runs at the top of the main loop, above every branch.

| Load | toggle | square wave |
|---|---|---|
| Idle | 214 kHz | 107 kHz |
| Host polling Modbus | 124 kHz | 62 kHz |

The sim drives `MCU_PWM` at 100 kHz. Both figures are means; the worst case
decides: a 276-byte SHTP cargo at 1.48 MHz is 1.5 ms, 320x the idle
half-period. What `VLATCH` tolerates is not yet measured.

## The gate drivers, and where the dead time lives

TIM1 centre-aligned, ARR **2375** off 237.5 MHz = **50.000 kHz**, RCR 1, CKD
DIV1: one dead-time tick is **4.2105 ns**.

**The dead time is a calibration parameter, not a #define**: param 13
(skew 14, untested and 0), applied from `main()` once the record loads,
floored at 20 ns in `board_limits.h`; `Board_PwmSetDeadTime` rounds UP - it
truncated once, and under-delivering is the unsafe direction. The default
asks 30 ns; DTG 8 holds **33.7 ns**. Bench-trimmed 2026-08-29 with the
supply's OCP as the instrument: 33.7 ns held a 240 s three-leg run at 50 %
with a 300 mA limit; DTG 7 = 29.5 ns tripped it (FINDINGS, *The supply
tripped its OCP*).

What simulation says is NEEDED is more. The gate drive is resonant, per
FET off the schematic:

    HO --[R9 0.47R]--[L6 120nH]--+-- gate   (D5 Schottky across R9)
                                  +-- D1 CDZV15B 15 V clamp
                                  +-- R7 4.99R + C7 3.9nF -- source

R7/C7 is a damper to source, not the gate resistor: the gate path is
0.47 Ω + 120 nH into C_gs 5.48 nF, the damper takes the resonance, which is
why the numbers barely move with load. Simulated on `electronic_simulations`
(`IAUCN10S7N021` VDMOS, `LQW18CAR12J00D`, `2EDL8034F5.lib`); the criterion is
gate overlap - outgoing gate crossing V_th against the incoming one -
because it is independent of the power loop inductance.

| V_th | V_DD | LS off -> HS on | HS off -> LS on |
|---|---|---|---|
| 2.2 V | 14.9 V | 49.9 ns | 56.3 ns |
| 2.2 V | 16.5 V | 52.1 ns | **59.4 ns** |
| 2.8 V | 15.7 V | 44.3 ns | 50.3 ns |
| 3.4 V | 14.9 V | 36.3 ns | 41.0 ns |

Tj 125 C, 100 A, 63 V. Over ±100 A the spread is 1.5 ns and over 27->125 C
1 ns: one fixed DTG, no adaptive dead time.

    59.4 ns   worst-corner gate overlap
   + 6.0 ns   TDMOFF max, 2EDL8034 (the 50 ns absolute delays are common
              to both channels and cancel to within the matching spec)
   ---------
    65.4 ns   what the tables call for

33.7 ns runs under that on one OCP data point at one duty on one supply,
nothing yet on a scope; the simulation is worst-corner, the bench a cold
board switching dry. The tension is stated here once.

* **The driver has no interlock** - 2EDL8034 datasheet p.1: *"Independent
  inputs allow controlling high- and low-side domains independently."*
  `BDTR.DTG` is the only shoot-through protection on this board, which is
  why `Board_PwmSetAll` is all-or-none.
* **No fault pin, no enable** - PG-DSO-8: VDD, HB, HO, HS, HI, LI, VSS, LO.
  `nFAULT` on `PE15` is the STO chain's. UVLO 7.3 V rising / 6.7 V falling
  on VDD, 6.3 / 5.7 V on VHB-HS.
* **Minimum pulse.** TPW 40 ns = 10 ticks. With DTG 8 the smallest
  high-side pulse is 18 ticks, ~76 ns, so `CCR >= 9` of 2375: **0.38 %
  duty**, the floor for low-speed saliency injection.

## Sensors on SPI

Both polled by the main loop into shared memory; the host never drives a
bus while a loop runs. Both die without `PB2`. What is fitted comes from
`0x6D` kind 4, not this table.

| Part | Bus | Pins | Frame | Reads |
|---|---|---|---|---|
| BNO085, 9-axis IMU (U13) | SPI2, mode 3, 2.97 MHz | `PB12` CS, `PB13` SCK, `PB14` MISO, `PB15` MOSI, `PD8` H_INTN, `PD9` PS0/WAKE, `PD10` NRSTN, `PD11` BOOTN | SHTP, 8-bit stream | up to 394 rotation vectors/s, Q14 |
| A1335LLETR-T, magnetic angle (U14) | SPI4, mode 3, 1.86 MHz | `PE2` SCK, `PE4` CS, `PE5` MISO, `PE6` MOSI | 20-bit packet as four 5-bit words | ANG, STA, ERR, XERR, TSEN, FIELD; 12 bits each |

* **The A1335's answer lags one frame** - the address arrives on MOSI bits
  17..12 while MISO has shifted out bits 19..16. Measured: TSEN, FIELD, TSEN
  in turn returned the previous register every time.
* **Mode 3 comes from the driver, not the .ioc.** `Board_ImuInit` and
  `Board_AngleInit` set every field of `hspi2`/`hspi4` and call
  `HAL_SPI_Init` themselves. The .ioc agreed on mode 3 since 2026-08-29 and
  is otherwise dead: 8-bit where SPI4 runs 5-bit words, 7.42 MBit/s where
  the driver derives 1.86 MHz.
* **SPI4 cannot carry a 20-bit word** - `IS_SPI_HIGHEND_INSTANCE` names
  SPI1-3 only and `HAL_SPI_Init` returns `HAL_ERROR` above 16 bits on the
  rest. Four 5-bit words under one chip select put exactly twenty edges on
  the wire.
* **The A1335's register map** is not in the datasheet here; addresses come
  from `github.com/ScranchNew/Allegro-A1335-Sensor-library`, and `0x6E`
  device 1 op 5 sets which one the loop reads.
* **`FIELD` says whether the angle means anything.** Measured 3 gauss with
  no magnet; the angle is then noise, and the views say so.

## Scaling & Calibration

The board reports raw codes and judges nothing, and carries the arithmetic
that turns a code into a quantity plus a record a rig can correct - a
calibration belongs to one physical board. `0x6E` device 3 is the record;
PROTOCOL.md the wire. Every figure below was traced off `electronics/` on
2026-08-26 and **none has been measured**; they are the record's defaults
for that reason.

| Channel | Chain | Result |
|---|---|---|
| **DC link** (`PC0`) | R12 49.9 kΩ / R11 2.2 kΩ (ERJ-PB3B, 150 V ±0.1 %), C117 680 pF, R42 15 Ω | 78.15 V full scale - 24 % over the 63 V rating, deliberate (invariant 11) |
| **NTC** (`PB0`) | R100 10.0 kΩ (ERA-3AEB103V, 0.1 %) to `+3V3_ref`, NTC1 NCU18XH103**D60**RB to GND, C116 15 nF | ratiometric, VREF cancels. `D60` *is* B₂₅/₅₀ = 3380 K. ~0.2 °C self-heating |
| **Phase U/V/W** | RU1‖RU2 in the phase conductor - two Vishay `WSHM28187L000FEA`, 7 mΩ each, 3.5 mΩ parallel - tapped by RU3/RU4 49.9 Ω into a THS4551, Rg 330 / Rf 1.5 k | 4.5455 V/V × 3.5 mΩ = 15.909 mV/A. 207.4 A full scale; 100 A is 48 % of the differential span |
| **Reference** | U2 REF2033 drives `+3V3_ref` **and** `+1V65_bias` | one part sets VREF and the mid-point, which is why they track |

* **The phase channels are current, not voltage** - the sense element is in
  the phase conductor; the firmware reports milliamperes off its own map.
* **The gain is bounded, not just traced.** 100 A across 3.5 mΩ is 350 mV,
  and 3.3 V over that is **9.43 V/V** - more gain could not represent the
  rating. The THS4551's 4.5455 fits; the ×18.5 and ×10 an Altium net dump
  put on the ADA4891 quad do not, by two orders of magnitude. **What that
  quad does in this path is unresolved** - buffer, level shift, or the
  `PH_CURR` protection tap on `Inverter.SchDoc`; not further gain.
* **Phase V anomaly:** a 0.85 V offset on the reference board, a localised
  op-amp failure - -52 A with nothing connected. **Do not zero Phase V on
  this board**: zeroing is for a channel's own offset, not broken silicon.
* **Zero before span** - spanning an un-zeroed channel folds the offset
  into the gain. Span is refused where the conversion is not linear in the
  code (PROTOCOL, device 3).

## I/O & Link

Three Modbus ports, one slave; a request is answered on the port it arrived
on, and the unit id belongs to the board.

| Port | Pins | Transceiver | Receives | Console |
|---|---|---|---|---|
| USART3 | `PB10` TX, `PB11` RX | none - debug probe VCP | interrupt | yes, or Modbus |
| USART2 | `PA1` DE, `PA2` TX, `PA3` RX | U5 THVD1450 | interrupt | no |
| UART5 | `PC8` DE, `PC12` TX, `PD2` RX | U6 THVD1450 | interrupt | no |

* **Direction control is the peripheral's** - `HAL_RS485Ex_Init` with
  hardware DE.
* **Both hear themselves.** RE is tied to GND on U5 and U6, so every byte
  transmitted returns and `put()` purges afterwards. Measured: `0x6E`
  device 2 op 0 sends 00, FF, 5A, A5 and all four come back on both - the
  check that driver, receiver and wiring work with nothing else on the
  segment.
* **All three receive on interrupt, each byte stamped with its tick.**
  Polling from the main loop stamped a byte when the loop reached it - a
  276-byte IMU cargo is 1.5 ms, seventeen characters at 115200. USART3 was
  the exception until 2026-08-29; measured, that cost 0.45 % of frames:
  1393 requests, 7 silent, `char_overrun` +7 (FINDINGS). The ring holds a
  whole frame; `ring_dropped` counts a loop that stopped draining.
* **No RX FIFO on any port** - `HAL_UARTEx_DisableFifoMode` on each. One
  character held, 87 µs at 115200: an overrun drops the frame, and a
  latched ORE ends reception until ICR clears it.
* **CubeMX carries 9216000 baud on the RS485 pair.** The firmware sets
  115200 at init, as it sets the SPI word sizes.
