# The wire

Modbus over Serial Line, RTU transmission mode, this board as server. USART3,
**115200 8N1**, unit address **1** by default. MODBUS Application Protocol
V1.1b3 plus MODBUS over Serial Line V1.02.

That one UART leaves the board two ways — the debug probe's virtual COM port,
or RS485 — and nothing below changes with which: same framing, same registers,
same timing. See [HARDWARE.md, The link](HARDWARE.md#the-link).

The authoritative source is `Comms/Inc/cmd.h` for the commands and
`Modbus/Inc/modbus_map.h` for the register map — both carry their layouts in
header comments. This document is the overview and the reasoning.

## Two ways in

The same UART carries a text console and the binary protocol, so they take turns.

- **Boot state: console.** `m` hands the line to the binary link, `r` prints
  link status, `?` prints help.
- **Back out:** command `0x48` (console), or holding register `0x0001 = 1`.
  Either way the response frame goes out *before* the switch, so the master
  always gets its answer.

While the link is open the main loop calls only `link_poll()` and never printf.
That is not tidiness: a blocking transmit inside a frame corrupts framing and
stalls reception long enough to latch a UART overrun.

## RTU framing

A frame is delimited **only by silence**. No length field, no delimiter — that
is the defining property of the mode.

- `t1.5` = 750 us, `t3.5` = 1750 us, the fixed values V1.02 prescribes above
  19200 baud. On target: **356 250** and **831 250** CYCCNT ticks at 475 MHz.
- CRC-16/MODBUS, reflected polynomial `0xA001`, init `0xFFFF`, transmitted **low
  byte first**. Catalogue check value over `"123456789"` is `0x4B37`.
- Frames shorter than 4 bytes or longer than 256 are discarded.
- **A bad CRC is answered with silence, never an exception.** Replying would put
  a frame on the bus the master cannot correlate, and on a multidrop line it may
  not have been addressed to us at all.
- **Broadcast (address 0) is executed and never answered.**

## Standard function codes

`0x01` read coils, `0x02` read discrete inputs, `0x03` read holding registers,
`0x04` read input registers, `0x05` write single coil, `0x06` write single
register, `0x0F` write multiple coils, `0x10` write multiple registers,
`0x11` report server ID.

Exceptions: `0x01` illegal function, `0x02` illegal data address, `0x03` illegal
data value, `0x04` server device failure.

Two rules that are routinely got wrong, and are got right here:

- An address outside the map is `0x02`. An illegal **quantity** is `0x03` — the
  request is malformed and the addresses were never consulted.
- `address + quantity` is checked in 32 bits. In 16 it would wrap, and a request
  straddling the top of the address space would slip past the range check.

A quantity of zero on `0x0F`/`0x10` is a well-formed 6-byte PDU declaring an
illegal quantity, so it answers `0x03` rather than falling silent. That is why
the dispatch minimum is 6 bytes and not 7.

A full-length `0x10` for 124 registers **cannot exist** on an RTU line: it needs
248 data bytes, a 257-byte ADU, past the 256 limit. The specification's
123-register limit *is* the framing limit.

## Register map

Zero-based PDU addresses, i.e. what goes on the wire. A master using one-based
`4x`/`3x` notation subtracts one.

### Input registers, FC `0x04`, read only

| Address | Contents |
|---|---|
| `0x0000`-`0x0006` | raw ADC code per channel, in channel-table order (HARDWARE.md). Differential codes signed, single-ended unsigned. |
| `0x0010` | DC bus millivolts, unsigned |
| `0x0011` | NTC in hundredths of a degree C, signed |
| `0x0020`, `0x0021` | SYSCLK Hz, high word then low |
| `0x0022`, `0x0023` | HCLK Hz, high word then low |
| `0x0030`-`0x003B` | six 32-bit RTU counters, high word first: bus message, bus comm error, server message, server exception, server no response, character overrun |

The space has holes by design — grouped for legibility rather than packed — and a
read spanning a hole is `0x02`.

### Holding registers, FC `0x03`/`0x06`/`0x10`

| Address | Contents |
|---|---|
| `0x0000` | unit address, 1..247. Takes effect on the *next* frame, so the response to the write still uses the old address. |
| `0x0001` | command register, reads back 0. `1` leaves binary mode, `2` zeroes the counters. Any other non-zero value is `0x03`. |

### Coils, FC `0x01`/`0x05`/`0x0F`

| `0x0000` | AFE_ON (PB2). Powers the front end **and the voltage reference** — see HARDWARE.md. |
|---|---|

### Discrete inputs, FC `0x02`

| `0x0000` | PE15 |
|---|---|

## Binary commands

These use the ranges the specification reserves for user-defined functions,
65..72 and 100..110. `modbus_slave.c` routes those ranges to
`model->user_function`; with no hook installed they answer `0x01`.

All integers big-endian. **No floating point on the wire** — physical quantities
are scaled integers in the units the command documents. Strings are one length
byte then that many ASCII characters, never terminated.

| Code | Name | req len | Purpose |
|---|---|---|---|
| `0x41` | version | 0 | **frozen** version record, see below |
| `0x42` | adc_table | 0 | the channel map plus a reading of each |
| `0x43` | adc_scan | 0 | one-shot scan with the board's own scaling |
| `0x44` | adc_noise | 3 | noise statistics on one ADC's phase channel |
| `0x45` | clock | 0 | live clock tree |
| `0x46` | afe | 1 | AFE_ON: read / off / on / toggle |
| `0x47` | link_stats | 0 | unit id, t1.5, t3.5, the six counters |
| `0x48` | console | 0 | hand the UART back |
| `0x64` | test_gate | 5 | open raw pin access, key `0x54455354` = `"TEST"` |
| `0x65` | echo | variable | round-trip up to 250 bytes |
| `0x66` | pin_mode | 4 | configure one pin, needs the gate |
| `0x67` | pin_read | 2 | read one pin |
| `0x68` | pin_write | 3 | drive one pin, returns the level **read back** |
| `0x69` | port_read | 1 | whole 16-bit input register |
| `0x6A` | port_write | 5 | masked atomic write via BSRR, needs the gate |
| `0x6B` | analog_burst | 8 | sample a channel set, per-channel statistics |
| `0x6C` | self_test | 0 | what the board can prove about itself |
| `0x6D` | channels | 1 | **the map**: analog, digital I/O, and what is reserved |

`pin_write` returns the level read back from the pin rather than echoing the
request. On an open-drain output, or a pin the fixture is holding, those differ —
and that difference is the whole reason a rig drives a pin.

### `0x6D` channels

```
req: u8 kind            0 analog, 1 digital IO, 2 reserved

rsp, kind 0:       u8 count, then per channel
                   u8 index, u8 adc_index, u8 channel, str pin,
                   u8 direction, u8 differential, str signal, u8 unit

rsp, kind 1 and 2: u8 count, then per pin
                   str pin, u8 direction, str signal
```

| direction | |
|---|---|
| `0` | in |
| `1` | out |
| `2` | both |

From the MCU's side. Every ADC channel is an input and says so rather than
leaving a host to assume it.

**Kinds 1 and 2 are kept apart on purpose.** Kind 1 is the digital I/O: what a
fixture may read or set without breaking anything — PB2 and PE15 on this board.
Kind 2 is USART3 and the debug port. Those are not channels, they are never to
be driven, and they are reported only so "why was PB10 refused" has an answer.
A flag on one combined list would have put them one misread away from a pin
write that severs the link the command arrived on.

Sections rather than one reply because one does not fit: measured, all of it
together came to 273 bytes against `MB_MAX_PDU`'s 253, and the writer's
overflow flag turned the first live call into an `0x04`. `system.channel_map()`
asks three times and joins them.

**This is the map, and it is the only one.** The firmware's table in
`Board/Src/board_io.c` is what `testrig.c` refuses pins from, what this command
reports, and what `Gpio._refusal` asks. A pin table in a host, a document or a
prompt is a second answer to "what is PB10", and the board is the one that is
right. `protocol.RESERVED_PINS` survives only as the fallback for a board older
than protocol 1.3.

### `0x41` version, the frozen command

```
u8 proto_major, u8 proto_minor,
u8 fw_major, u8 fw_minor, u8 fw_patch,
str device, str mcu, str build,
u16 command_count
```

The protocol major is **first** so a host of any vintage can read two bytes,
decide whether it understands the device, and stop. Fields may only ever be
**appended**: an old host decodes the prefix it knows and ignores the rest.
Reordering or resizing a field creates a new major whether that was intended or
not.

Bump `CMD_PROTO_MINOR` when appending, `CMD_PROTO_MAJOR` when anything existing
changes. A host selects its codec on the **major alone** — binding it to the
firmware version means every rebuild breaks the host. In
`host/coaxial/board.py` that lookup is one line:

```python
BOARD_CLASSES = {1: Board}
```

### `0x6B` analog_burst

```
req: u16 channel_mask, u16 samples (1..10000), u32 interval_us (0 = as fast as possible)
rsp: u16 samples, u32 elapsed_us, u8 count,
     per channel: u8 index, i32 mean_milliraw, i32 min_raw, i32 max_raw, u32 sd_milliraw
```

Raw codes only, on purpose — the host owns the scaling. Means and deviations are
in milli-codes (raw x 1000) to carry fractions without a float. `elapsed_us` is
**measured**, not assumed, so a host sees the rate it actually got. Statistics
use Welford in one pass, so no sample buffer is needed at any length. A burst
longer than 5 s is refused on both sides rather than left to outlive the master.

### `0x6C` self_test

```
rsp: u8 count, then per check: str name, u8 status, i32 value
     status: 0 pass, 1 fail, 2 info
```

**Pass or fail only where the board can prove the answer** from its own registers
or its own flash: HSE ready, PLL locked, SYSCLK derived from the crystal, the two
clock figures agreeing, the cycle counter advancing, VREFBUF disabled with VREF+
high-impedance, and at most two bits set in each ADC's PCSEL. None of those needs
a reference.

Everything else is **INFO with a value** and no verdict: the three differential
calibration factors, the flash image length and its CRC-16, SYSCLK, HCLK, and the
AFE state. The board has nothing to compare them against; a line compares them
across units and against the build it meant to load.

The distinction is not cosmetic. An earlier version of this command asserted that
each ADC's differential calibration factor was non-zero, and failed a perfectly
healthy board — a well matched ADC legitimately calibrates to an offset of zero,
and no register says whether the calibration ran. That is exactly the mistake a
limit compiled into firmware invites, so the check became an INFO value.

### Reserved pins

`testrig.c` refuses these in **every** mode, gate or no gate:

| Pin | Why |
|---|---|
| PB10, PB11 | USART3 — severing them loses the link the command arrived on |
| PA13, PA14, PA15, PB3, PB4 | the 5-pin debug port — losing it costs the ability to reflash |

`port_write` **masks reserved bits out** rather than rejecting the whole write,
so a fixture driving a bank of outputs need not know which bits the board keeps
for itself. Verified on hardware: `port_write('B', 0xFFFF, 0x0000)` left GPIOB at
`0x0C10` with PB10/PB11 intact and the link still answering, while PB2 — which is
not reserved — was legitimately cleared.

## Versions

Firmware **1.4.1**, protocol **1.3** — `0x6D channels` appended, which is a MINOR: an old host never calls it. `Comms/Inc/version.h` holds the firmware
version; the protocol pair is `CMD_PROTO_MAJOR`/`CMD_PROTO_MINOR` in
`Comms/Inc/cmd.h`, beside the command table it describes. The build string comes from `__DATE__`/`__TIME__`, which
makes the binary non-reproducible. That is paid deliberately: a production rig
that cannot tell which build a board carries cannot investigate a failure after
the fact.

## Conformance

`host/tests/test_conformance.py` — 65 checks, all passing. It is a **deliberately
independent** master: its own CRC, its own framing, not the `coaxial` library, so
a shared wrong assumption between the two sides cannot hide a defect. Its CRC is
checked against the catalogue value before any frame is sent.

The best check in it: write coil 0 (AFE_ON) and read discrete input 0 (PE15) as
an independent witness that the write reached the pin, since PE15 was measured to
follow AFE_ON inversely.
