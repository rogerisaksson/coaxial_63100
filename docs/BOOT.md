# Bootloader

Nodes are blank: image, calibration record and position live on the
master, which sends them over 10 Mbit Modbus RTU broadcast at power-up.
With a debugger and no master, reset costs 0.3 s and the app starts.
Built and host-tested; **not yet run on a board**.

## Principle: nothing on a node is versioned

The node never asks what version anything is. `erase` names an image by
size and CRC-32; a node holding a valid image with both keeps it (no
erase, stream ignored, `verify` ok at once). `seal` compares the record
word by word with its sector and programs only what differs. A boot where
nothing changed writes nothing. The header's version field is never read.

## Flash map (`boot.h`)

| Address | Size | What |
| --- | --- | --- |
| 0x08000000 | 128 K | bootloader (bank 1 sector 0), code runs from ITCM |
| 0x08020000 | 1792 K | application; `boot_header_t` at +0x400: magic 'CXAP', size (linker `_app_size`), version, type |
| 0x081E0000 | 128 K | calibration record (bank 2 sector 7) |

Valid image = SP in DTCM, thumb reset vector inside the app, header magic
and type match, size in range. **The first flash word (8 vectors) is
programmed last, on `seal`**: an interrupted stream leaves the node blank,
never bricked.

## What a node knows

- Type: compiled in (`BOOT_BOARD`; pins differ per type). Only
  `coaxial_63100` has a pin table.
- Unique id: 96 bits at `UID_BASE`.
- Everything else is assigned per boot and handed to the app in the
  **handover slot** (`boot_hand_t`, top 32 B of DTCM, NOLOAD in both
  linker scripts): unit, position, flags. `Board_BootInit` applies unit id
  and termination. No magic (bench board flashed over SWD) = unit 1.
- Unit id = position down the limb; the last node closes the 120 ohm
  termination (PE14).

## Reset

1. Pins first: six gate inputs, PA10, PB2, PE14 driven low (nothing floats).
2. Clocks: HSE 25 MHz / 5 x 64 / 2 = 160 MHz, APB1 80 -> BRR 16 = exactly
   10 Mbit (VOS1). Registers only, no HAL.
3. USART2 + UART5 at 10 Mbit, USART3 console 115 200. No interrupts.
4. Slot says STAY -> stay. Valid app -> listen 300 ms for `hold`, else fill
   the slot, return the clock tree to reset state, jump. No app -> stay,
   one console line a second.

A blank node answers unit 247. Broadcasts are never answered; a user
function returns `MB_NO_REPLY` for a request not addressed to this uid.

## Master sequence (`coaxial.boot.Master`)

```text
hold      broadcast repeatedly from power-up
who       prefix search on uid; a collision (CRC error) splits the prefix
assign    uid -> unit, position, terminate; unknown uid is named, not assigned
erase     one broadcast per type; wait 2 s a sector (the node hears nothing)
chunk     224 B each (7 flash words), 2 ms apart (flash-bound)
missing   per node; re-send, three rounds
verify    per node
record    per node, 224 B pages
seal      per node (3 s timeout: may erase the record sector)
go        broadcast
```

The bootloader erases and programs inside its receive path, so the master
waits (`coaxial.boot`: ERASE_S, CHUNK_S, VERIFY_S, SEAL_S). Estimate: 200 K
image = 915 chunks, ~2 s stream + 4 s erase; the same image again costs
round trips only.

## Application side

- `stay` (dev 11 op 12): reply, wait 50 ms, write STAY, reset.
  `state` (op 10) answers sealed/valid + the handed-over identity; the
  other ops are refused in words (`cmd_boot.c`).
- `Board_Early()` (main.c `USER CODE BEGIN 1`) sets VTOR and copies ITCM.

## Store and tools

```text
store/nodes.json                  {uid: {bus, position, type}}
store/images/<type>.bin
store/records/<bus>/<position>.record
```

`tools/flash_nodes.py --store DIR --port COMx --bus LL` runs the master;
`--place UID BUS POS TYPE` writes a row; `--simulated` uses four stand-in
nodes. `build_and_flash.py --boot` flashes the bootloader over SWD.

## Tests

- `test_boot_core.py` (C core on a RAM flash via gcc/ctypes).
- `test_boot.py` (master vs `SimulatedBoot` / `SimulatedSegment`).
- `test_structure` holds PROTOCOL.md's device 11 table to both servers.

## Open (bench)

10 Mbit on the bench adapter; ITCM execution during programming; erase time
per sector; what a real collision looks like; a `coaxial_63020` pin table.
