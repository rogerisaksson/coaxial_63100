# Coaxial BLDC Inverter (63 V / 100 A)

"Coaxial" dictates the mechanical stator-mount, not cabling. Consequences are absolute: thermal choking is inherent (making the NTC a mandatory control input, not a diagnostic luxury), and phase sensing lives inside a switching bridge (making idle noise figures useless). 100 A is a peak SOA survival limit, not a continuous rating.

## Silicon & Clocks

* **MCU:** STM32H753VIT6, Rev V. The hardware revision is strictly necessary to support the 950 MHz VCO.
* **Clocks:** 475 MHz SYSCLK driven by a 25 MHz HSE. The ADC kernel clock is decoupled (75 MHz async), rendering sampling times immune to SYSCLK reconfigurations. No LSE/RTC.

## AFE & The `PB2` Trap

The internal ADC VREF is disabled. The reference is driven externally by the AFE.

* **`PB2` (`AFE_ON`):** Powers the amplifier chains *and* the ADC reference. Polling channels with `PB2` low returns exact mid-scale (yielding a phantom 25 °C on the NTC).
* **`PE15` (`nFAULT`):** Tracks `AFE_ON` inversely. It reads logic `0` when the AFE is powered.

## Scaling & Telemetry (Host Domain)

The board is a dumb slave. It reports raw 16-bit ADC codes. Physical conversions, limits, and calibration strictly belong to the host infrastructure, not the firmware.

* **DC Link (`PC0`):** Absolute scaling dependent on VREF. Full-scale is 78.15 V, providing a deliberate 24% headroom over the 63 V rating. Do not "improve" the divider ratio and blindly clip transients.
* **NTC (`PB0`):** Ratiometric (Murata NCU18, B=3380). VREF error mathematically cancels out. Subject to ~0.2 °C parasitic self-heating.
* **Phase V Anomaly:** The 0.85 V offset observed on Phase V of the reference board is a localized op-amp failure. Do not write firmware offsets to calibrate around broken silicon.

## I/O & Link

* **USART3 (`PB10`/`PB11`):** Polled mode, no RX FIFO. A single overrun byte drops the frame. Carries Modbus RTU/Console via debug VCP or RS485.
* **Incomplete RS485:** The firmware configures `VM_ASYNC` without a driver-enable pin. Half-duplex RS485 direction control currently does not exist in software.