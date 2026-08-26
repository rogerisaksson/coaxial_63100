# Protocol Architecture

Modbus RTU over USART3 (115200 8N1). The bus is shared with an ASCII console at boot; switching to binary mode requires an explicit command (`0x48` or Holding Register `0x0001`).

## RTU Framing & Compliance

* **Delimiters:** Strict silence-based framing ($t_{1.5} = 750\ \mu\text{s}$, $t_{3.5} = 1750\ \mu\text{s}$ at 475 MHz).
* **Error Handling:** CRC errors yield silence, never exceptions, preventing multidrop collisions. Illegal quantities return `0x03` (checked via 32-bit math to prevent wrap-around exploits). A reading the board could not take returns `0x04`, never a zero on the wire: on a differential channel code 0 *is* 0 V, so a failure delivered as data cannot be told from a measurement. Arguments are validated before any conversion runs, so `0x03` means the request and `0x04` means the device.
* **Broadcast:** Address 0 executes silently.

## Standard Modbus Map

* **Input Registers (`0x04`):** Raw ADC codes, DC bus (mV), NTC (0.01 °C), clock frequencies, and error counters.
* **Holding Registers (`0x03`/`0x06`/`0x10`):** Unit ID (`0x0000`, applied next frame), Mode switch (`0x0001`).
* **Coils (`0x01`/`0x05`/`0x0F`):** `AFE_ON` front-end power switch (`0x0000`).
* **Discrete Inputs (`0x02`):** `PE15` / `nFAULT` status (`0x0000`).

## Custom Binary Commands (FC 65–72, 100–110)

Payloads use big-endian integers and length-prefixed strings. Floating-point math is banned on the wire.

* **`0x41` Version:** Append-only struct. Host decoding logic binds exclusively to `CMD_PROTO_MAJOR`. Adding fields bumps `MINOR` and preserves backward compatibility.
* **`0x6B` Analog Burst:** Calculates Welford statistics (mean, min, max, variance) locally in milli-codes over up to `BOARD_BURST_MAX_SAMPLES` passes, capped again at `BOARD_BURST_MAX_US` of wall time so a burst cannot outlive the master's patience. A failed conversion aborts the whole burst rather than folding a zero into the mean. Scaling is strictly delegated to the host.
* **`0x6C` Self Test:** Emits PASS/FAIL *only* for register-provable hardware states (e.g., PLL lock, PCSEL state). External or uncalibrated variables yield INFO for host evaluation.
* **`0x6D` Channels:** The definitive source of truth for the pin map and for what is fitted. Dynamically queried by the host to eliminate hardcoded assumptions. Safely segregates analog/digital I/O from reserved system pins. One section per request, selected by a leading kind byte, because two together came to 273 bytes against `MB_MAX_PDU`'s 253.

  | Kind | Section | Request |
  |---|---|---|
  | 0 | analog channels | `0x6D 00` |
  | 1 | digital I/O | `0x6D 01` |
  | 2 | reserved pins - the bus and the debug port | `0x6D 02` |
  | 3 | subsystems, one per command table | `0x6D 03` |
  | 4 | fitted parts | `0x6D 04 <first>` |

  Kind 4 is paged: six parts with their strings are 380 bytes. It answers `u8 total, u8 first, u8 count`, then per part `str name, str what, str where, str power, u8 state`. `power` names what must be on for the part to work at all; `state` is `0` not probed, `1` ready, `2` unpowered, `3` silent - measured, never asserted (invariant 10). Adding a part is one row in `Board/Src/board_io.c`; nothing above it needs telling.

* **`0x6E` Device:** Every peripheral, chosen by a leading device byte, then an op byte: `0x6E <device> <op> [payload]`. One function code for all of them because there are none left - the user-defined ranges are 65..72 and 100..110, and this board had spent all but 110. A second code answered ILLEGAL FUNCTION from the protocol layer before dispatch saw it. Adding a device is a row in `cmd_device.c` and an op dispatcher beside it.

  | Device | Part | Bus | Ops |
  |---|---|---|---|
  | 0 | BNO08X IMU | SPI2, mode 3, 1.48 MHz | 0 product id, 1 raw cargo, 2 Set Feature, 3 raw bytes off the bus, 4 reset, 5 raw write on any SHTP channel, 6 per-pin drive/pull check, 7 time H_INTN's answer to a wake, 8 shared record, 9 hold, 10 resume |
  | 1 | A1335 angle sensor | SPI4, mode 3, 1.86 MHz | 0 read register, 1 write register, 2 shared record, 3 hold, 4 resume, 5 which register the loop reads, 6 clock |
  | 2 | the three serial ports | USART3, USART2, UART5 | 0 loopback check, 1 per-port counters |

  Device 2 op 0 transmits 00, FF, 5A, A5 on the port named and answers which came back - all four on an RS485 port, none on USART3. **The port carrying the request refuses**: its own patterns land in front of the reply, and the master sees a checksum failure. Op 1 answers `bus_message` and `server_message` separately, and their difference is the traffic addressed to another node on the segment.

  **The board polls both parts from its own main loop and writes shared memory; a host reads that.** Reading a cargo per request cost 45 ms each and caught one frame in eight. Ops that drive a bus are refused unless that device's loop is held - both running is two masters on one bus. Hold, configure, resume.

  The IMU's shared record answers `u8 loop, u8 error, u32 updates, u32 cargoes, u32 errors, u8 have`, then `u8 report_id, u8 status` and four Q14 counts. The angle sensor's answers `u8 loop, u8 error, u32 updates, u32 errors, u8 have, u8 register, u16 value, u8 crc`. `updates` is monotonic in both, so a host tells a new reading from the same one read twice without guessing from the values.

  The A1335's packet is 20 bits (Figure 31): MOSI is SYNC=0, R/W, six address bits, eight data bits, four CRC bits; MISO is sixteen data bits and four CRC bits. It goes out as four 5-bit words under one chip select - `HAL_SPI_Init` refuses a data size above 16 bits on SPI4, which `IS_SPI_HIGHEND_INSTANCE` does not name. **The answer lags one frame**, so a register read is two packets. The CRC is reported and not checked: the datasheet gives the field's width and not its polynomial.

## Hardware Safeguards & Conformance

* **Reserved Pin Masking:** Critical interfaces (USART3 `PB10/PB11`, JTAG `PA13-15/PB3/PB4`) are hard-masked in the firmware during atomic `port_write` operations, preventing a host from severing its own link or bricking the debug port.
* **Independent Conformance:** Tested via `test_conformance.py`, which implements a separate Modbus stack from scratch to eliminate shared-code blind spots. Validates logical writes via physical side-effects (e.g., writing the `AFE_ON` coil and observing the hardware invert the `PE15` input).