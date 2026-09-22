# The bootloader

A node is blank. It has a bootloader, a unique id, and nothing else worth
keeping: the firmware image, the calibration record, the link settings
and its place in the machine live on the master, and the bootloader's
job is to take them over the bus at power-up, prove they arrived whole,
and hand over. At the bench, with a debugger on SWD and no master, the
same bootloader finds a valid application and starts it a third of a
second after reset, so the build-flash-debug loop is unchanged.

Written 2026-09-22 as the design; each commit below says what of it is
built. Nothing here has run on a board yet - the section *What only the
bench can answer* lists what a flash will decide.

## The ask

Extremely slim. Waits for the master node to send firmware over 10 Mbit
Modbus RTU by broadcast, so every node of one type takes the same image
at once. Switches on which type of board it is and where it sits in the
electromechanical system - a bus, a position down the limb. Everything,
calibration parameters included, is stored on the master; the nodes are
blank at boot. And it must work at the bench with a debugger attached.

## What a node knows, and what the master holds

A node knows three things at reset:

* its **type**, compiled into the bootloader it was given at production -
  `BOOT_BOARD` is `coaxial_63100` or `coaxial_63020`, the two inverter
  types the machine already names (`host/coaxial/simulated/link.py`),
  since the pins a bootloader must open to say anything at all differ by
  type, so the type cannot arrive over the bus it opens;
* its **unique id**, the MCU's 96 bits at `UID_BASE`, which no two parts
  share and nobody programs;
* whether the **application** it holds is valid - a vector table that
  points into RAM and into the application, a header with the magic and
  the size, and the first flash word present (below).

Everything else is the master's: a table from unique id to bus,
position and type; one image per type; one calibration record per
(bus, position), kept as the opaque flash word the board itself wrote;
the link settings. A node's position and its unit id - the two are one
number, unit id is position down the limb - come from that table and
are written into its record, which is where the application reads them:
`modbus_map_set_unit_id` exists and nothing calls it today, so commit 3
gives the record a `unit_id` beside `link_baud` (CAL_VERSION 16) and the
record's load in `main` sets it. So the
identity a running node has IS its record, and the master rewrites the
record on every boot it chooses to: nothing is kept on the node that the
master did not put there.

The switches the ask names are these two: the master switches on TYPE
to pick the image and the record's layout; the node switches on POSITION
once told it - the last node on a segment closes the 120 Ω termination
on PE14, the rest leave it open - and the bootloader's own switch on
`BOOT_BOARD` picks the UART pins and the flash map for its type.

## The flash map

The H753 has 2 MB in two banks of eight 128 KB sectors. A 128 KB sector
is the smallest thing that can be erased, so the bootloader takes bank
1 sector 0 whole and the application starts a sector in:

| Address | Bytes | What |
| --- | --- | --- |
| 0x08000000 | 128 K | the bootloader, sector 0 of bank 1; some 10 K used |
| 0x08020000 | 1792 K | the application, bank 1 sectors 1-7 and bank 2 sectors 0-6 |
| 0x081E0000 | 128 K | the calibration record, bank 2 sector 7, as today |

The application's linker script moves its FLASH origin to 0x08020000
and its length to 1792 K; nothing else in it moves - DTCM, the `.itcm`
copy, `.buffers` in AXI SRAM, the record's address. Its vector table is
at 0x08020000 and the bootloader sets `SCB->VTOR` there before it jumps;
the generated `SystemInit` leaves VTOR alone (`USER_VECT_TAB_ADDRESS` is
not defined), so the setting survives into the application. A header
follows the vector table at a fixed offset, `KEEP`'d by the linker:

```text
APP_BASE + 0x400:  u32 magic 'CXAP'   u32 size (bytes, from the linker)
                   u32 version         u32 board type
```

The size is a linker symbol taken by address, so the header is right by
construction and needs no post-link step; the version and the type are
the application's own constants. What makes an image VALID is four
tests the bootloader runs at reset: the first word is a stack pointer
inside DTCM, the second is a thumb address inside the application's
range, the header's magic and type match, and the size fits the range.
A sector that was never written reads 0xFF everywhere and fails the
first test.

THE FIRST FLASH WORD IS WRITTEN LAST. A 256-bit flash word holds the
first eight vectors; the bootloader keeps that word in RAM while an
image streams in and programs it only on `seal`, after the whole image
verified. An image interrupted anywhere - power lost mid-stream, a
master that died, a bus that dropped the last chunk - leaves that word
erased, the image invalid, and the node in the bootloader waiting for
the master to try again. There is no second image slot and no rollback:
a node with no valid application is not a bricked node, it is a blank
one, which is what the ask made every node at power-up anyway.

## Reset: the decision

The bootloader runs FROM ITCM. Its startup copies `.text` there the way
the application's startup copies its sample path (the same loop, the
same linker pattern), because a core that fetches from the bank being
programmed stalls until the flash word lands - 100 µs a word - and the
USART's eight-byte FIFO covers 8 µs at 10 Mbit. Executing from RAM, the
core services the wire while the flash controller works.

At reset, in order:

1. Copy to ITCM. Stack in DTCM. No cache: nothing here is hot.
2. Clocks: HSE 25 MHz through PLL1 to 160 MHz, APB1 at 80 MHz, so a
   USART with 8x oversampling divides to exactly 10 Mbit (`BRR` 16, no
   fractional error). Five register writes and a wait for the lock.
3. The two RS485 USARTs at `BOOT_BAUD`, transmit-enable pins as the
   type's table says; the console USART at 115 200 for one line a state,
   since a bench terminal is the cheapest instrument there is. A 1 ms
   SysTick. The DWT cycle counter, for the RTU gaps (invariant 2).
4. If the application asked to be here - a word in the last sixteen
   bytes of DTCM equals `STAY`, written by the application before it
   reset itself, in a `.noinit` slot both linker scripts place at the
   same address - clear it and stay.
5. Else if the application is valid: listen for `HOLD_MS` = 300 ms. A
   master that wants the node broadcasts `hold` at power-up and keeps
   broadcasting it while it enumerates; a node that hears one stays. A
   node that hears nothing sets VTOR, loads the stack pointer and jumps.
   A debugger changes nothing: a bench with no master pays 0.3 s.
6. Else stay, and say so on the console once a second: `boot: no
   application - waiting for the master`.

In the bootloader the node is an RTU slave on both segments at once,
unit id 0 until assigned - it answers only what is addressed to it by
unique id, and broadcasts. The gate drivers are off by construction:
the bootloader never touches PA10, so the STO pump is not fed, the
drivers' supply is not released, and TIM1 is in reset with every leg's
pins high-impedance. AFE_ON stays low. Nothing switches.

## The protocol

One function code, `0x6F BOOT`, with ops in PROTOCOL.md's form. The
RTU framing, CRC, unit addressing and the length oracle are the tree's
own (`modbus/`, `comms/src/cmd_length.c`); the bootloader links the
same three portable files the application does, and `boot/src/
boot_core.c` is the state machine over them, hardware-free and
host-tested like they are.

| op | Request | Reply |
| --- | --- | --- |
| 0 hold | `u32 session` | none: broadcast. A node in the window stays; a node already staying notes the session |
| 1 who | `u8 bits, bytes prefix` | `u8 12 uid, u8 type, u8 state, u8 unit` from every node whose unique id begins with `prefix` (`bits` of it); two answering at once garble, and the master narrows the prefix |
| 2 assign | `u8 12 uid, u8 unit, u8 position, u8 flags` | `u8 took` from the one node with that uid, which answers to `unit` from now on; bit 0 of `flags` closes the termination |
| 3 erase | `u8 type, u32 size, u32 crc, u16 chunks` | none: broadcast. Every node of `type` erases the sectors `size` needs, forgets what it held, and takes chunk 0's first word into RAM; a node of another type ignores it |
| 4 chunk | `u16 index, bytes` | none: broadcast. 224 bytes at `index * 224`; a node programs it as it lands and sets the bit |
| 5 missing | - | `u16 first, u16 count, bytes bitmap` - the chunks not yet held, as a bitmap from `first` |
| 6 verify | - | `u8 ok, u32 crc` over the written range with the held first word in place; `ok` says it equals the erase's |
| 7 record | `u16 offset, bytes` | `u8 took`; the calibration record's bytes into RAM at `offset` |
| 8 seal | - | `u8 took`; the record programmed into its sector, the first word programmed, the image valid |
| 9 go | `u32 session` | none: broadcast. Every sealed node of the session jumps; an unsealed one stays and says why on its console |
| 10 state | - | `u8 state, u8 type, u8 unit, u8 position, u32 chunks_held, u32 chunks_of, u8 app_valid, u8 12 uid` |
| 11 dump | `u16 offset` | `u16 offset, bytes` - the record sector as it stands, 224 bytes a page, so the master can take a commissioned board's record into its store whole |

Refusals are the board's words, as everywhere: `assign` to an unknown
uid answers nothing (it is not that node's request); `seal` before
`verify` passed answers `u8 0` and `str "the image is not verified"`;
`chunk` for a session not erased is ignored with a counter, since a
broadcast cannot be refused.

The master's sequence, per bus:

```text
hold ×N          broadcast every 50 ms from the moment power is applied,
                 through enumeration - a node's 300 ms window sees one
who              prefix search on the unique id: 0 bits first; a garbled
                 or timed-out answer splits the prefix, 96 bits deep at
                 most, one round trip each; known uids answered directly
assign           each uid its unit and position off the table; unknown
                 uid: stop, and name it for the operator to place
erase            one broadcast per type present, then wait: a sector is
                 about a second to erase and a 200 K image spans two
chunk stream     every chunk once, in order, 2 ms apart - flash programs
                 a 224-byte chunk in 0.7 ms and the wire carries it in
                 0.25 ms, so the flash and not the wire sets the pace
missing          unicast to each node; the union of what is missing is
                 re-broadcast; three rounds, then the node is named
verify           unicast to each node; a mismatch names the node
record           unicast to each node, 224 bytes a page, from the store
seal             unicast to each node
go               broadcast
state            unicast, 400 ms later: the ones that answer stayed
```

A 200 K image is 915 chunks: 1.8 s of stream after 2 s of erase, plus
the round trips - about five seconds for a bus of four nodes taking
the same image, whatever the count. Chunk 0's first word is held back
by the node itself, so the stream's order is free.

## Identity and enumeration

Broadcast is answered by nobody, and a bus of blank nodes has no
addresses. The prefix search on the unique id is what makes the
enumeration deterministic: `who` with 0 bits asks everybody, and one
answer means one node; more than one means a garbled frame or a CRC
error at the master, which then asks for the 0-prefix and the 1-prefix
in turn, and so on down the bits until each branch holds one node - the
same walk a 1-Wire bus does. With the master's table, every known node
is asked directly by its whole uid and only a new board costs a search.
`assign` gives the node its unit id for the rest of the session and its
position for the record; the record carries both into the application.

Two nodes assigned one position is the master's table being wrong, and
the master refuses to erase until it is not: nothing on the wire can
tell two boards in one place apart, but the table can be made not to
say it.

## Recovery and anti-brick

| Failure | What happens |
| --- | --- |
| a chunk lost on the wire | the node's bitmap misses it; `missing` names it; re-broadcast |
| the master dies mid-stream | the first word is in RAM only; the image is invalid; the node stays in the bootloader; the next master starts over with `erase` |
| power lost mid-erase or mid-program | the same: an erased first word is an invalid image |
| the wrong image for the type | `erase` names the type; a node of another type ignores the whole session |
| a `go` before `seal` | the node stays, and its console says `boot: not sealed` |
| a node that never answers `who` | the master's table names it as expected on that bus; the operator reads the console |
| the record's sector fails to program | `seal` refuses in words; the master says which node |

There is no CRC in the application's header, so the bench's flash of an
ELF over SWD needs no stamping step: the programmer verified what it
wrote. The master's path verifies over the wire, before the seal.

## The bench: debugger, build, CI

* `cmake --preset Debug` builds two executables: `coaxial_63100.elf`
  (the application, at 0x08020000) and `coaxial_63100_boot.elf` (the
  bootloader, at 0x08000000). Both have size lines and both are CI
  artifacts, both presets.
* `build_and_flash.py` flashes the application as today, at its own
  address; `--boot` flashes the bootloader too. A board flashed with
  only the new application boots through whatever bootloader it has; a
  board with the old layout at 0x08000000 still runs its old
  application, since nothing is at 0x08020000 that a reset reaches.
* With a debugger and no master, reset costs 0.3 s in the bootloader
  and the application starts. Breakpoints, the console, the record's
  ops all as before. `STM32_Programmer_CLI -d app.elf` writes sectors 1
  on; the bootloader's sector is untouched.
* Device op `LINK_OP_REBOOT` (`u8 to_bootloader` -> `u8 took`) is how a
  running node is sent back for a re-flash: the application writes
  `STAY` into the shared DTCM slot and resets. The MCP tool and the
  library get a `reboot()` beside `stand_down`.
* `-Wall -Wextra -Wshadow -Wconversion`, zero warnings, like the cores.

## The master's store on disk

```text
store/
  nodes.json                { "<uid hex>": {"bus": "LL", "position": 2, "type": "coaxial_63100"}, ... }
  images/
    coaxial_63100.bin       the application image, from the build
    coaxial_63100.json      { "version": "...", "size": N, "crc32": "...", "built": "..." }
    coaxial_63020.bin       ...
  records/
    LL/2.record             the flash word the board wrote, taken by `dump`
    LL/2.json               { "taken": "...", "from": "<uid>", "version": 15 }
```

`python tools/flash_nodes.py --store DIR --port COM4` is the master:
`--dump` takes a commissioned board's record into the store, `--place
<uid> LL 2` writes the table, and the plain run does the sequence above
and prints a line per node per step. `coaxial/boot.py` is the protocol
and the sequence as a library, so the MCP tool and a future master
board's firmware follow the same steps.

## The stand-in: how it is tested without a board

* `boot/src/boot_core.c` is the state machine: chunk bookkeeping, the
  bitmap, the header tests, the ops decoded and encoded, over three
  callbacks (erase, program a flash word, read). `boot/test/harness.c`
  wires them to a RAM image and `host/tests/test_boot_core.py` drives
  it through gcc and ctypes like `test_modbus_core.py`: an image
  streamed with chunks dropped and re-sent, verify failing on a wrong
  crc, seal refused before verify, the first word absent until seal,
  a power loss mid-stream leaving an invalid image, the prefix search
  terminating on any set of uids.
* `coaxial/simulated/boot.py` is a blank node speaking `0x6F` over the
  stand-in's link, one per (bus, position) with its own uid; `host/
  tests/test_boot.py` runs `coaxial.boot`'s whole sequence against four
  of them on one bus, drops chunks, kills the master mid-stream, and
  checks every node's state.
* The wire check in `test_structure` holds the op table above to
  `boot_core.c`'s reads and writes, as it holds every device's.

## Files

New:

* `boot/inc/boot.h`, `boot/src/boot_core.c` - the portable core
* `boot/src/boot_main.c` - reset, clocks, UARTs, flash, the jump; the
  only file that touches a register
* `boot/test/harness.c`, `host/tests/test_boot_core.py`
* `boot/STM32H753xx_BOOT.ld`, `boot/startup_boot.s`
* `host/coaxial/boot.py`, `host/coaxial/simulated/boot.py`,
  `host/tests/test_boot.py`, `host/tools/flash_nodes.py`
* `docs/BOOT.md` - this file

Changed:

* `STM32H753xx_FLASH.ld` - origin 0x08020000, the header section, the
  shared DTCM slot
* `core/src/main.c` - nothing; the header is a `const` in `board/src/
  board_boot.c` with the `reboot()` the op calls
* `comms/inc/cmd.h`, `comms/src/cmd_link.c` - `LINK_OP_REBOOT`
* `CMakeLists.txt` - the second executable, its warnings, its size line
* `.github/workflows/firmware.yml` - both images sized and kept
* `host/tools/build_and_flash.py` - `--boot`
* `docs/PROTOCOL.md` - `0x6F BOOT` pointing here, and the link op
* `docs/ARCHITECTURE.md`, `docs/HARDWARE.md` - the flash map, `boot/`
  in the layout

## Commits, in order

1. This document, and PROTOCOL.md's pointer.
2. `boot_core.c` with its harness and `test_boot_core.py` - the state
   machine proven on the host before any register is touched.
3. The application relocated: the linker script, the header, the DTCM
   slot, `LINK_OP_REBOOT`; builds, flashes as before, the wire checks
   hold the new op.
4. The bootloader target: `boot_main.c`, its script and startup, the
   second executable in CMake and CI, `--boot` in the flash tool. Built
   and sized; not run.
5. `coaxial/boot.py`, the simulated blank node, `test_boot.py`,
   `flash_nodes.py` with its store.
6. The bench: the first flash of the bootloader over SWD, the
   application through it, a re-flash over the wire - FINDINGS carries
   what happened.

## Numbers

| What | Estimate |
| --- | --- |
| bootloader image | ~10 K: the RTU core 3 K, the state machine 3 K, UART, flash and clocks 2 K, startup 1 K |
| chunk | 224 bytes: 7 flash words, 252 bytes on the wire with the frame, 0.25 ms at 10 Mbit |
| programming a chunk | ~0.7 ms; the stream is paced at 2 ms a chunk |
| a 200 K image | 915 chunks, 1.8 s of stream, after ~2 s of erase for two sectors |
| a bus of four nodes | ~5 s from `hold` to `go`, whatever the count of nodes |
| worst case | three `missing` rounds at a full re-stream each: 4 × 1.8 s, then the node is named |
| the bench's reset | +0.3 s in the bootloader with no master |

## Risks, and what only the bench can answer

* The 10 Mbit figure: the record's `link_baud` was found running at
  9 216 000, and the THVD1450 is good to 50 Mbit; the bootloader's
  clock tree gives exactly 10 Mbit, and the application's 475 MHz tree
  does not - so `BOOT_BAUD` is the bootloader's alone, and the
  application keeps the record's. Whether the bench's adapter runs at
  10 Mbit is the first thing to measure.
* Executing from ITCM while programming: the stall is documented for
  the same bank; that a core in ITCM is untouched is the reference
  manual's word, and the first stream will say.
* Erase time per sector on this silicon, and whether the master's 2 s
  wait is right - `state` polling replaces the wait if not.
* The prefix search on a bus where two nodes answer at once: what the
  master actually sees is a CRC error or a timeout; both split the
  prefix, and the first bus of four blank nodes will show which.
* A node of the other type, `coaxial_63020`, needs its own pin table in
  `boot_main.c`'s switch; this design builds only `coaxial_63100`'s.
