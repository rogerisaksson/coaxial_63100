# Coaxial BLDC Inverter (63 V / 100 A)

"Coaxial" dictates the mechanical stator-mount, not cabling. Consequences are absolute: thermal choking is inherent (making the NTC a mandatory control input, not a diagnostic luxury), and phase sensing lives inside a switching bridge (making idle noise figures useless). 100 A is a peak SOA survival limit, not a continuous rating.

## The Board Itself

`electronics/` holds the schematic and the BOM, and they are the authority on what is fitted and how it is wired. This document records what the firmware has to know and what was measured; it does not repeat the netlist.

* **`Coaxial 63100 Schematics.pdf`** - 21 sheets. `IMU.SchDoc` and `AngleSensor.SchDoc` are the two SPI sensors, `Regulators.SchDoc` the rails.
* **`Coaxial 63100 BOM.csv`** - 380 lines, designator to manufacturer part.
* **`ENABLE_AFE` gates a regulator on `Regulators.SchDoc`**, the sheet that makes `+15V7`, `+5V7`, `+5`, `+3V3_ref`, `+1V65_bias` and `+3V3D`. U13 runs from `+3V3D`/`VDDIO`, U14 from `+5`. Measured: both stop answering with `PB2` low, which is what the `PB2` trap below is about.

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

## Scaling & Telemetry (Host Domain)

The board is a dumb slave. It reports raw 16-bit ADC codes. Physical conversions, limits, and calibration strictly belong to the host infrastructure, not the firmware.

* **DC Link (`PC0`):** Absolute scaling dependent on VREF. Full-scale is 78.15 V, providing a deliberate 24% headroom over the 63 V rating. Do not "improve" the divider ratio and blindly clip transients.
* **NTC (`PB0`):** Ratiometric (Murata NCU18, B=3380). VREF error mathematically cancels out. Subject to ~0.2 °C parasitic self-heating.
* **Phase V Anomaly:** The 0.85 V offset observed on Phase V of the reference board is a localized op-amp failure. Do not write firmware offsets to calibrate around broken silicon.

## I/O & Link

* **USART3 (`PB10`/`PB11`):** Polled mode, no RX FIFO. A single overrun byte drops the frame. Carries Modbus RTU/Console via debug VCP or RS485.
* **Incomplete RS485:** The firmware configures `VM_ASYNC` without a driver-enable pin. Half-duplex RS485 direction control currently does not exist in software.