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

* **`0x6E` IMU:** Every IMU operation, chosen by a leading op byte - the specification's user-defined ranges are spent, and a second function code answered ILLEGAL FUNCTION before dispatch saw it. The board polls the BNO08X from its main loop into shared memory; a host reads that and never drives SPI2 while the loop runs.

  | Op | Does | Needs a hold |
  |---|---|---|
  | 0 | product id | yes |
  | 1 | one raw SHTP cargo | yes |
  | 2 | Set Feature - report id, then interval in us | yes |
  | 3 | raw bytes off SPI2; a flag clocks them with chip select left high | yes |
  | 4 | reset the part | yes |
  | 5 | raw write on any SHTP channel | yes |
  | 6 | drive and release each of SPI2's pins, reporting what read back | yes |
  | 7 | time H_INTN's answer to a wake | yes |
  | 8 | the poll loop's shared record | no |
  | 9 | hold the loop | - |
  | 10 | resume it | - |

  Ops that drive SPI2 are refused unless the loop is held: both running is two masters on one bus. Op 8 answers `u8 loop, u8 error, u32 updates, u32 cargoes, u32 errors, u8 have`, then `u8 report_id, u8 status` and four Q14 counts. `updates` is monotonic, so a host tells a new reading from the same one read twice without guessing from the values. Reading a cargo per request instead cost 45 ms each and caught one frame in eight.

## Hardware Safeguards & Conformance

* **Reserved Pin Masking:** Critical interfaces (USART3 `PB10/PB11`, JTAG `PA13-15/PB3/PB4`) are hard-masked in the firmware during atomic `port_write` operations, preventing a host from severing its own link or bricking the debug port.
* **Independent Conformance:** Tested via `test_conformance.py`, which implements a separate Modbus stack from scratch to eliminate shared-code blind spots. Validates logical writes via physical side-effects (e.g., writing the `AFE_ON` coil and observing the hardware invert the `PE15` input).