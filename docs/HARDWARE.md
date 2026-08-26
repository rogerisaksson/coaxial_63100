# Coaxial BLDC Inverter (63 V / 100 A)

"Coaxial" dictates the mechanical stator-mount, not cabling. Consequences are absolute: thermal choking is inherent (making the NTC a mandatory control input, not a diagnostic luxury), and phase sensing lives inside a switching bridge (making idle noise figures useless). 100 A is a peak SOA survival limit, not a continuous rating.

## The Board Itself

`electronics/` holds the schematic and the BOM, and they are the authority on what is fitted and how it is wired. This document records what the firmware has to know and what was measured; it does not repeat the netlist.

* **`Coaxial 63100 Schematics.pdf`** - 21 sheets. `IMU.SchDoc` and `AngleSensor.SchDoc` are the two SPI sensors, `Regulators.SchDoc` the rails.
* **`Coaxial 63100 BOM.csv`** - 380 lines, designator to manufacturer part.
* **`ENABLE_AFE` gates a regulator on `Regulators.SchDoc`**, the sheet that makes `+15V7`, `+5V7`, `+5`, `+3V3_ref`, `+1V65_bias` and `+3V3D`. U13 runs from `+3V3D`/`VDDIO`, U14 from `+5`. Measured: both stop answering with `PB2` low, which is what the `PB2` trap below is about.

## Schematic Names Are Not MCU Names

The sheet symbols are logical and zero-based; the nets on `MCU.SchDoc` carry ST's peripheral names; the firmware and the board's own channel map use neither, they use what the signal *is*. Three vocabularies for one wire, so the mapping is written down once, here, and traced from the netlist rather than assumed.

| Sheet symbol | MCU peripheral | Part |
|---|---|---|
| `SPI0` (IMU.SchDoc) | SPI2 | U13 BNO085 |
| `SPI1` (AngleSensor.SchDoc) | SPI4 | U14 A1335 |
| `UART0` (RS485.SchDoc) | UART5 | U6 THVD1450 |
| `UART1` (RS485.SchDoc) | USART2 | U5 THVD1450 |
| none | USART3 | debug probe VCP, no transceiver |

Traced on `MCU.SchDoc`: net `UART2_RO` lands on sheet port `UART1`, `UART5_RO` on `UART0` - the indices are not in peripheral order, which is exactly why guessing them is not safe.

The RS485 sheet also names the transceiver's own pins `DI`, `RO`, `DE` and `RE`, which are the part's, not the MCU's: `DI` is the MCU's TX and `RO` is its RX.

**The analog channels have a third set of names again.** The schematic numbers the differential pairs `ADC0P/ADC0N` through `ADC5P/ADC5N` in pin order; the firmware names each by what it measures - Phase U, Phase V, Phase W, Clevel, NTC, DC bus, Cinj. Command `0x6D` kind 0 is the authority on which pin is which, and nothing above the firmware should carry a second answer.

## Silicon & Clocks

* **MCU:** STM32H753VIT6, Rev V. The hardware revision is strictly necessary to support the 950 MHz VCO.
* **Clocks:** 475 MHz SYSCLK driven by a 25 MHz HSE. The ADC kernel clock is decoupled (75 MHz async), rendering sampling times immune to SYSCLK reconfigurations. No LSE/RTC.

## AFE & The `PB2` Trap

The internal ADC VREF is disabled. The reference is driven externally by the AFE.

* **`PB2` (`AFE_ON`):** Powers the amplifier chains, the ADC reference *and both SPI sensors*. Polling channels with `PB2` low returns exact mid-scale (yielding a phantom 25 °C on the NTC).
* **A sensor without `AFE_ON` is worse than dead.** It still drives MISO, still resets, and still returns a valid 276-byte SHTP advertisement - so every read looks healthy. What it never does is act on a write: `Set Feature` starts no stream, and executable `ON`, `SLEEP` and `RESET` all produce the identical answer, which is only possible if none of the payloads arrived. Firmware refuses `Board_ImuInit` while `PB2` is low, and losing `PB2` clears the ready flag, because a part that has lost its supply needs a reset rather than a resume.
* **`PE15` (`nFAULT`):** Tracks `AFE_ON` inversely. It reads logic `0` when the AFE is powered.

## Sensors on SPI

Both are polled by the firmware's main loop into shared memory; the host reads that and never drives a bus while a loop runs. Both die without `PB2`. What is fitted comes from the board - command `0x6D` kind 4 - not from this table.

| Part | Bus | Pins | Frame | Reads |
|---|---|---|---|---|
| BNO085, 9-axis IMU (U13) | SPI2, mode 3, 1.48 MHz | `PB12` CS, `PB13` SCK, `PB14` MISO, `PB15` MOSI, `PD8` H_INTN, `PD9` PS0/WAKE, `PD10` NRSTN, `PD11` BOOTN | SHTP, 8-bit stream | ~35 rotation vectors/s, Q14 |
| A1335LLETR-T, magnetic angle (U14) | SPI4, mode 3, 1.86 MHz | `PE2` SCK, `PE4` CS, `PE5` MISO, `PE6` MOSI | 20-bit packet as four 5-bit words | ANG, STA, ERR, XERR, TSEN, FIELD; 12 bits each |

* **The A1335's answer lags one frame.** The address arrives on MOSI bits 17..12 while MISO has already shifted out bits 19..16, so a register read is two packets. Measured: asking TSEN, FIELD, TSEN in turn returned the previous register's value every time.
* **SPI4 cannot carry a 20-bit word.** `IS_SPI_HIGHEND_INSTANCE` names SPI1, SPI2 and SPI3 only, and `HAL_SPI_Init` returns `HAL_ERROR` above 16 bits on the rest. Four 5-bit words under one chip select put exactly twenty clock edges on the wire, which is what the part counts.
* **The A1335's register map is not in the datasheet here** - it defers to the Programming Manual. Addresses come from `github.com/ScranchNew/Allegro-A1335-Sensor-library`, and `0x6E` device 1 op 5 sets which one the poll loop reads, so a better address needs no rebuild.
* **`FIELD` says whether the angle means anything.** Measured 3 gauss with no magnet in front of the part, and the angle is then noise; the views say so rather than drawing a confident pointer.

## Scaling & Calibration

The board still reports raw codes, and still judges nothing. What it now also
carries is the arithmetic that turns a code into a quantity, and a record a rig
can correct - because a calibration belongs to one physical board, and a host
that holds it answers for the wrong board the moment it is pointed at a second.
`0x6E` device 3 is the record; `docs/PROTOCOL.md` is the wire.

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
  represent the board's own rating at all. The THS4551's 4.5455 fits; the
  ×18.5 and ×10 an Altium net dump appeared to put on the ADA4891 quad do not,
  by two orders of magnitude. **What that quad does in this path is
  unresolved** - buffer, level shift, or the `PH_CURR` protection tap on
  `Inverter.SchDoc`. It is not further gain into the ADC.
* **Nothing here has been measured.** Both numbers come off a PDF. Zero and
  span each phase against a clamp meter before believing an ampere from this
  board - that is what `0x6E` device 3 exists for.
* **Phase V Anomaly:** the 0.85 V offset on the reference board is a localized
  op-amp failure. In amperes it reads as -52 A with nothing connected. `zero`
  would make that number go away and the fault with it - **do not zero Phase V
  on this board.** Zeroing is for a channel's own offset, not for broken
  silicon.
* **Zero before span.** Spanning an un-zeroed channel folds the offset into the
  gain, which then reads right at the reference point and nowhere else.
* **Span is refused where the conversion is not linear in the code**: the
  thermistor is logarithmic, and a channel with no unit has nothing to be told.

## I/O & Link

Three Modbus ports, one slave. A request on any of them is answered on that one; the unit id belongs to the board, not to the wire.

| Port | Pins | Transceiver | Receives | Console |
|---|---|---|---|---|
| USART3 | `PB10` TX, `PB11` RX | none - debug probe VCP | polled | yes, or Modbus |
| USART2 | `PA1` DE, `PA2` TX, `PA3` RX | U5 THVD1450 | interrupt | no |
| UART5 | `PC8` DE, `PC12` TX, `PD2` RX | U6 THVD1450 | interrupt | no |

* **Direction control is the peripheral's.** Both RS485 ports run `HAL_RS485Ex_Init` with hardware DE, so there is no driver-enable pin to toggle in software and no window where the driver is late.
* **Both hear themselves.** RE is tied to GND on U5 and U6 (RS485.SchDoc: pin 2 on the GND net), so the receiver stays on while DE drives. Every byte transmitted returns, and `put()` purges afterwards. Measured: `0x6E` device 2 op 0 sends 00, FF, 5A, A5 on a port and all four come back on both - which is also the check that the driver, the receiver and the wiring between them work with nothing else on the segment.
* **The RS485 pair receives on interrupt, and each byte carries the tick it arrived at.** RTU delimits frames by silence, and polling from the main loop timestamped a byte when the loop reached it - a 276-byte IMU cargo is 1.5 ms, seventeen characters at 115200. The ring holds a whole frame; `ring_dropped` is not zero if the loop ever stopped draining.
* **No RX FIFO on any of the three.** A single overrun drops the frame, and a latched ORE ends reception until ICR clears it.
* **CubeMX carries 9216000 baud on the RS485 pair**, which is not a Modbus rate on any bus. The firmware sets 115200 at init, the same way it sets the SPI word sizes rather than trusting them.