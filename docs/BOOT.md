# Bootloader

The bootloader is the only firmware a node keeps for good. The application
runs from RAM and comes from the master over Modbus RTU: every node of a
type, by broadcast, at power-up. Flash keeps a sealed copy for a power-up
with no master. A host always runs the image it was built with, so nothing
on a node is versioned. Built and host-tested; **not yet run on a board**
(2026-09-23).

## Principle: the checksum is the gate

- An image is named by size and CRC-32, never by a version. `erase` offers
  one. A node whose RAM or store already holds it keeps it: verified at
  once, nothing streamed.
- The store is flash. It is written only when `seal` asks to persist and
  it holds another image. Its seal word is programmed last, so a store
  interrupted anywhere holds nothing.
- Nothing runs unverified. At reset the bootloader recomputes the CRC over
  RAM (a warm reset) or over the store before copying it, then jumps.
- The record is compared word by word with its sector and programmed only
  where it differs.

## Memory map (`boot.h`)

| Address | Size | What |
| --- | --- | --- |
| 0x08000000 | 128 K | bootloader (bank 1 sector 0), code runs from ITCM |
| 0x08020000 | 1792 K | store: seal word (`boot_seal_t`: 'SEAL', size, crc, type), then the image |
| 0x081E0000 | 128 K | calibration record (bank 2 sector 7) |
| 0x30000000 | 288 K | RUN: D2 SRAM1..3, where the application is linked and runs |

- The application fits: 201 K Debug, 135 K Release (2026-09-23). The app
  uses no D2 SRAM otherwise. The bootloader enables its clocks and leaves
  them on through the jump.
- Image: `boot_header_t` at +0x400 ('CXAP', size = linker `_app_size`,
  version, type). Valid = SP in DTCM, thumb reset vector inside RUN, magic
  and type match, size in range.
- The image's first word (8 vectors) is written last, on `seal`.
- Host side: `coaxial.devices.boot.image_of(elf)` cuts the image from the ELF's
  load segments. `store_of(image)` puts the seal word in front of it.

## What a node knows

- Type: compiled in (`BOOT_BOARD`; pins differ per type). Only
  `coaxial_63100` has a pin table.
- Unique id: 96 bits at `UID_BASE`.
- Handed to the app in the **handover slot** (`boot_hand_t`, top 32 B of
  DTCM, NOLOAD in both linker scripts): unit, position, flags, and the size
  and crc of the image verified. `Board_BootInit` applies unit id and
  termination. No magic (a debugger started the app) = unit 1, image unknown.
- Unit id = position down the limb. The last node closes the 120 ohm
  termination (PE14).

## Reset

1. Pins first: six gate inputs, PA10, PB2, PE14 driven low (nothing floats).
2. Clocks: HSE 25 MHz / 5 x 64 / 2 = 160 MHz, APB1 80 -> BRR 16 = exactly
   10 Mbit (VOS1). Registers only, no HAL. D2 SRAM clocks on.
3. USART2 + UART5 at 10 Mbit, USART3 (the ST-Link's port) at 115 200. All
   three serve Modbus; no text lines, since they would land inside frames.
4. The gate (`boot_ready`): RAM holding the image the slot names, or the
   store's sealed copy verified and copied. Ready and not asked to stay:
   listen 300 ms for `hold`, then jump. Otherwise wait for the master.

A blank node answers unit 247. Broadcasts are never answered; a user
function returns `MB_NO_REPLY` for a request not addressed to this uid. A
0x6E reply carries the fields alone, as the application's (`boot_pdu`).

## Master sequence (`coaxial.devices.boot.Master`)

```text
hold      broadcast repeatedly from power-up
who       prefix search on uid; a collision (CRC error) splits the prefix
assign    uid -> unit, position, terminate; unknown uid is named, not assigned
erase     one broadcast per type; RAM or store holding it: kept or copied
chunk     224 B each into RAM, 2 ms apart (t3.5)
missing   per node; re-send, three rounds (bitmap <= 165 B, one reply)
verify    per node
record    per node, 224 B pages
seal      per node, [persist]: record, first word, store where it differs
go        broadcast
```

The bootloader works inside its receive path, so the master waits
(`coaxial.devices.boot`: ERASE_S, CHUNK_S, VERIFY_S, SEAL_S, PERSIST_S).
Estimates: 200 K = 915 chunks, ~2 s at 10 Mbit, ~22 s at 115 200 on the
ST-Link's port. Persist adds 2 sectors' erase. The same image again costs round
trips only.

## The host's own build

`Coaxial63100.open()` on a real board compares `state`'s image with this host's
build (`$COAXIAL_IMAGE`, else the newest `build/*/coaxial_63100.elf`). If they
differ, `coaxial.devices.boot.load` sends `stay`, runs the master's sequence on
that one node at unit 247 with its unit, position and flags given back,
persists, sends `go`, and waits until the application names the new image. That
happens once per build; the ST-Link's port costs ~22 s. Exceptions:

- No build at hand: nothing is compared.
- Image (0, 0), meaning a debugger started the app: left alone.
- The board shared with other sessions: refused in words, not reset.
- `own_image=False` turns the step off.
- Nothing at the unit and a node waiting blank at 247: loaded as the unit, at
  position = unit (the emulator's `emulator://?nodes=1&boot=1`).

## Application side

- `stay` (dev 11 op 12): reply, wait 50 ms, write STAY, reset.
  `state` (op 10) answers sealed/valid, the handed-over identity and the
  image's size and crc (MINOR 19). The other ops are refused in words
  (`cmd_boot.c`).
- `Board_Early()` (main.c `USER CODE BEGIN 1`) sets VTOR to 0x30000000 and
  copies ITCM.

## Store and tools

```text
store/nodes.json                  {uid: {bus, position, type}}
store/images/<type>.bin
store/records/<bus>/<position>.record
```

- `tools/target/flash_nodes.py --store DIR --port COMx --bus LL [--persist]`
  runs the master; `--persist` has each store keep the image.
  `--place UID BUS POS TYPE` writes a row; `--simulated` uses four stand-in
  nodes.
- `build_and_flash.py` programs the app's sealed store over SWD (`.store.bin`
  at 0x08020000); `--boot` flashes the bootloader first.

## Tests

- `test_boot_core.py`: the C core on byte-array flash and RAM via gcc and
  ctypes. It covers the store, a warm reset, a power cycle, a torn store,
  the host's own `Boot` client byte for byte through `boot_pdu`, and a
  running node reloaded through `load` and the front door's step.
- `test_boot.py`: the master against `SimulatedBoot` / `SimulatedSegment`,
  persist, and the image cut from an ELF.
- `test_structure` holds PROTOCOL.md's device 11 table to both servers.

## Open (bench)

10 Mbit on the bench adapter; D2 SRAM execution speed against flash; erase
time per sector; what a real collision looks like; a `coaxial_63020` pin
table; the first `open()` loading a build over the ST-Link's port.
