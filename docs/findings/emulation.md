# Findings: emulation

Renode, native:// and CI: the image emulated, its peripherals, its speed. The
board's own are in [FINDINGS](../FINDINGS.md).

- Renode under the drive: 3.8-4.5 wall s a board s, 110 M guest instructions a
  board s at ~33 ns each; Debug and Release alike. The CPU thread's RIP sampled:
  translated code 41 %, coreclr's crossings 17 %, the FPU's lazy state saved at
  each interrupt 9 %, dispatch 7.5 %, softfloat 5 %, the plant 3 %; our models'
  pacing a fifth at most. The M7 runs translate-arm-experimental, 1.17.0 the
  latest release: the tty defaults to native://, Renode stays the proof
  (2026-09-28).
- emulator:// loads build/Debug: a Debug image older than the host's last
  firmware commit answered with checksum failures. Rebuild after pulling
  (2026-09-28).
- CI's FC05 after the 257-byte ADU: the console's line queued the host's next
  frame behind the ADU's tail. After the burst Renode ran at 0.1 of real time -
  0.18 s wall, 18 ms virtual, the ADU 21 ms on the wire - and FC05 joined it:
  counters +2 messages, 1 error, no exception. The line starts a host frame
  t3.5 and 0.25 ms after the last byte, as the limb's adapter did (HostLine):
  answered 1.4 ms virtual behind the ADU (2026-09-28).
- Renode's USARTs ran on a fixed 125 MHz: USART3's BRR 1031 at 121 241 baud
  against the part's 115 179 (PCLK1 118.75 MHz, coaxial_63100.ioc). The
  bootloader's PCLK1 is 80 MHz, and Renode reads an OVER8 BRR raw: its 0x10 at
  14.8 Mbit against 10. The models take the rate from the RCC and BRR
  (UART_Rate.cs) (2026-09-28).
- The 10 Mbit bus echo ran the app's port at its record's 115 200 - the
  firmware refuses a `link_baud` past 921 600 - behind a 10 Mbit adapter, and
  Renode passed every byte. The transceivers decode by rate now, 2.6 % at
  OVER8: that pairing opens nothing and logs `garbles`; the test runs the
  record's rate (2026-09-28).
- CI's lost echo, 1 of 50 with two framing errors past the open's, at either
  rate and never here: a write reached Renode in pieces 1.0 and 1.5 ms apart,
  two and three quanta, and the adapter began the second as a frame. No
  request comes that soon, waiting on its predecessor's reply, but the boot's
  broadcast chunks come 2 ms apart: no threshold between. The sockets take a
  write as a frame, its length ahead of it (`frames://`), put on the line
  whole once all of it has come (2026-09-28).
- The attitude page stood 5 s at a time: the front page's link watcher, alive
  in the same process, opened a session on native:// every 30 s mid-request.
  Asked once now (2026-09-28).
- The attitude page froze on native://, 3 reads in 80 frames: a ctypes call a
  board millisecond, each waiting on the drawing for the interpreter, held the
  board to 21 %; one call a burst (native_lockstep): 100 %, 72 (2026-09-28).
- Entering `coaxial` through `coaxial.comm.session`, `rig` took `EMULATOR_URL`
  off a half-built session: six pages threw on the emulator, the suite green.
  `sessionmod.EMULATOR_URL` at call time (2026-09-28).
- A view drawing faster than the board answers repeats a reading: 24 of 60
  frames on the emulator, and the freshness note flickered live/stale every
  other frame. Staleness is elapsed stillness now, not one repeated frame
  (2026-09-28).
- Renode 1.17.0's STM32H7_ADC has 19 inputs (a conversion on 19, Vgate, throws
  inside Renode) and drops ADC2's registers at +0x100; its terminal hands a TCP
  chunk over at once, so a 257 B ADU overran the 256 B ring. board/emu replaces
  the ADCs and paces the console at its baud: the image passes conformance
  110/110, 420 MB, 47 s (2026-09-25).
- Renode's H7 RCC assumes an 8 MHz HSE (the board's is 25): SysTick ran at
  0.32 of the clock set. The DWT counted a fixed 475 MHz, so the bootloader's
  (160 MHz) t1.5 was 253 us, and a chunk split across a 0.5 ms quantum was
  lost: 47 % of the stream. The DWT counts the core's clock now, and the
  adapter keeps t3.5 + 0.25 ms between the host's frames: none lost. Wall s a
  virtual s at 475 MIPS: app 28, bootloader 10 (2026-09-25).
- Renode's ADC keeps no JEXTSEL or JEXTEN in JSQR: HAL's InjectedStart refused,
  and seven papers stopped at "an injected group would not start". board/emu's
  ADC converts the injected group on TRGO2 (MMS2 1000, OC5REF): 6 032 triples
  in 2 s, no overrun. At Renode's 100 MIPS the drive's ISR outran its period;
  the emulator runs the part's 475 (2026-09-25).
- Conformance on `emulator://` on a Threadripper 1950X: 110/110 at 100 MIPS,
  109 at 200, 107 at 475. The three after the 257 B ADU go unanswered: the
  master's waits are wall seconds, unscaled by the port's `time_scale`
  (2026-09-25).
- Emulator speed: the image's MPU. tlib keeps no TLB entry for a page inside an
  enabled region whose subregion is disabled, and walks the MPU on every access
  there; CubeMX's 4 GB region (SRD 0x87) spans ITCM, DTCM and D2 SRAM. Its enable
  masked on the bus: idle 8.2 -> 1.4 wall s a virtual s at 100 MIPS, 23 -> 5.2
  at 475; the drive 43-62 -> 18-22; 8 idle boards in one Renode 37-49 -> 390 M
  instructions a wall second. On in the suites only. Under the drive half is the
  A1335 loop (20 -> 10.7 held): 128 k packets a virtual second, ~45 register
  accesses each, its 1 us settles spinning on CYCCNT; the quantum no factor
  there. Renode's own speed on
  this laptop: 1 200 M ALU instructions a wall second, a GPIO read 0.66 us, a
  BSRR write 1.4 us, a DWT read 1.3 us (0.56 board/emu's). Renode's TIM1 was: an
  event every update, 3.5 wall s a
  virtual s at 100 MIPS; board/emu's counts lazily (9.4 -> 5.9) at the tree's
  237.5 MHz where Renode's ran 250. The core runs 100 MIPS until an ADC waits on
  TRGO2, then the part's 475 (2026-09-25).
- The emulated board under the drive, MPU on: 43-62 wall s a virtual s at 475
  MIPS, the plant's step 4.5 us of it; TIM1's compares written every period
  were rescheduling its timer (64-76 before). A page reading in its draw waited
  a round trip a frame: on a Feed the attitude page draws 10 fps (5.6), the
  rotor observer 7.7 (0.3), capture 120 frames in 35 s (536) (2026-09-25).
- The monitor's tokenizer takes no exponent: `2e-05` in a world's line and
  Renode exited (2026-09-25).
- Renode rewrites `%APPDATA%\renode\history` after every monitor command: a
  body's five processes, each asked its load every 0.5 s, collided, and a limb
  exited on an IOException. Each process has its own `--config` and history;
  the humanoid idle, 20 boards: 2.4-2.6 wall s a virtual s (2026-09-25).
- The A1335 by DMA, its read stepped from main() without a spin: the emulated
  drive 20 -> 17 wall s a virtual s (the angle loop 9 -> 6). main() in WFI with
  AFE_ON low and MOE clear: idle at 475 MIPS 5.2 -> 1.0, real time. Renode
  takes `WfiAsNop` only after `ClearTranslationCache`, and restarts
  `ExecutedInstructions` on a reset; FastDWT anchors on the CPU's TimeHandle at
  the first read after a wake. Asleep, Renode's load reads 1: the host waits
  by the awake scale, and a socket:// rig without it gave up on replies
  (2026-09-25).
- The thermal observer's borrow of AFE_ON (0.5 s every 30 s) took test_wire's
  AFE-off scan once the idle board kept real time: the test holds sampling
  off (2026-09-25).
- The A1335 and the BNO085 paused for that borrow with the host holding
  AFE_ON beside it: the host's angle and IMU dropped 0.5 s every 30 s
  (test_emulator's angle read, None). They pause when the observer holds the
  rail alone, `Board_PowerAlone` (2026-09-26).
- The emulated drive, 50 kHz, costs a period 3 interrupts (Renode: 3.7 us an
  entry and exit, 4.8 with FPU state), ~30 register accesses (0.9 us each on
  board/emu's models) and ~2 950 instructions at -O0. Floor with the handlers
  whole: ~2 wall s a virtual s. main() skipped to 0.5 us short of each timer
  event: 16.5 -> 6; Board_PwmReady and ARR kept once read, ADC3's injected end
  without HAL's walk: 6 -> 2.5, 50 000 updates a virtual second, no overrun.
  Changing MIPS inside a Renode round lost a third of the periods; SkipTime
  keeps its accounting. CYCCNT and TIM1's count run on one virtual clock:
  instructions, sleeps and skips. The Release image ran 7: its main() polls
  more registers a slice (2026-09-25).
- The drive's triples straight into the compares (RCR 1 while it owns them,
  unskewed): no TIM1 update interrupt, 1 interrupt and ~18 register accesses a
  period. A/B, alternating on one quiet laptop: the emulated drive 4.8-5.7 ->
  3.0-3.5 wall s a virtual s; the same image read 2.5 hours earlier, so only
  alternated runs compare (2026-09-25).
- A script killed by `timeout` left its Renode spinning 4 h, skewing every
  measure since; each Renode now sits in a job that dies with its Python
  (2026-09-25).
- native://: board_pwm.c, board_sync.c, board_adc.c and board_drive.c built for
  this host over board/native's TIM1, ADCs and AFE. The drive 1 virtual s in
  0.048 wall s flat out; paced, 1.000, 50 000 updates a wall second, no
  overrun; the conformance suite 96/96 over it. The pacer at Windows' 15.6 ms
  timer and 5 ms a wake ran a third of real time: timeBeginPeriod(1), caught up
  whole each wake.
  The host's `-Wconversion` found three narrowings in board_pwm.c and
  board_sync.c the target's flags pass (2026-09-26).
- native:// with the whole board layer (power, STO, A1335 by DMA, BNO085 over
  SHTP, the drive): 1 virtual s in 0.055 wall s a board. The humanoid, 20
  boards on 5 buses, every drive holding: each limb 1.0000 virtual s a wall
  s, 50 000 triples a virtual s a board, no overrun, 2.05-2.28 of 16 cores
  (2026-09-26).
- With `proven_dispatch` the host skipped t3.5 after any proven request: the
  addressed board closes a frame on its CRC, the rest of a limb on the
  silence, so a frame for another unit ran into the last and went unanswered.
  The gap is skipped for the same unit only. Rigs on one URL bus each hold a
  Transport, none owing another's gap: the native limb keeps t3.5 as the
  bus's adapter (2026-09-26).
- The emulated heat was one lumped node, the MCU die 8 K over it; the
  observer's model puts 0.666 W through 22.5 + 40.5 K/W, 42 K. Both dies read
  colder than modelled and the NTC's inversion (x12 at 30 s, x27 at 2 s)
  threw the V patch to 5 C. The plant runs thermal.c's network as truth
  (world_heat.c): the NTC within 0.25 K under the demo's 30 A, driver U 26-100 C
  (2026-09-26). The rotor page on native: STABLE at 311 s; TH OBS 95 % is the
  laminate's capacity, sigma 0.125 of its 0.10 (2026-09-28).
- The attitude's tumble turned 140 and 280 deg/s on the emulator and native
  (2.56 s); the stand-in stepped it a read, a turn in 1.28 s at 200 reads/s.
  25.6 s on the clock: 31 deg/s (2026-09-26).
- Renode prints a peripheral's C# compile error on its console only: the
  emulator "did not answer" and the rig fell back to the stand-in (2026-09-26).
- One heat clock for every world but the bench: `coaxial.model.thermal.HASTE`
  10 on the stand-in, the plant and the emulated observer (thermal op 13), a
  step 0.1 thermal s at any clock. On a 100 ms wall poll the clamp lagged a
  leg's 1.4 s and the derate cycled 0.12-0.9. A plant hasted from power-on
  ran the MCU's node a kelvin ahead of an observer set at the open, and the
  dies' anchor dragged the legs 1-6 K: the rig sets both at once. THERMAL
  OBSERVER's load is the demo motor's 30 A rms on both, 12 s on, 24 off; the
  MCU's die reads in the off phase. `tools/dev/ab.py`, the page's reads over
  the same board seconds: 67 faults against the stand-in to 3 on native
  (2026-09-26).
- The papers on the stand-in against the emulator (`make_notebooks.py
  --compare`, 2026-09-27): a software-clocked task gave no record while a
  drive held the converters - the meter reads the latched triple's phases
  now, as it did the link and the NTC; a record's codes were trimmed on the
  board and again on the host - a phase at rest read -8.8 A - once now, at
  the source, the TIM1 clock's too; the host's tare wrote a trimmed burst's
  mean as the whole offset, a second tare undoing the first (-51 A after
  one on the stand-in's offsets) - folded into the offset now; the host dropped
  MINOR 7's sensor rows from the layout; commissioning, the ring test and
  the machine's arming waited on the wall's clock, the emulated flux spin
  never reached 300 rad/s (lambda 0.00069 against 0.00546 V.s); the
  stand-in's link gave t1.5/t3.5 as 1750/4083, not the part's cycles, and the
  console's frames on all three ports.
- native:// had no console: after 0x48 gave the line back a board went
  silent for every later session in the process, the tty's next page on the
  stand-in. The fake board takes 'm' again (2026-09-26).
- The emulated A1335s answered bare twelve bits and Clevel/Cinj read 0: the
  part's register identifiers (ANG 5, TSEN F, FIELD E, the stand-in's from
  the bench) and the unmodified board's 0.06 / 0.77 V now (2026-09-26).
- The emulator's cost is its register accesses and interrupts, not its
  models: 1.24 M accesses a virtual second idle at 0.9 us each, 17 a period
  under the drive at Renode's floor. Cut a pass: HAL_RCC_GetSysClockFreq's
  7 RCC reads (the IMU's poll), AFE_ON's 3 IDR reads (a shadow); the idle
  core at 50 MIPS, the quantum 0.5 ms. Idle with the AFE on 2.8 -> 1.0 wall
  s a virtual s, real time. The STO chain: a step an edge cost 3.4 us,
  50 000 a virtual second, now batched at 1 kHz with the edges' instants;
  the drive's ISR counted as the chain's time put the keepalive 46 us apart
  and the observer's slice 850 us (the part: 9 and 190), so the chain's
  clock is main()'s instructions stretched by the interrupts' share; the
  slice broke the hold's regime every 100 ms and the chain never held, so a
  longer gap in a held regime runs from the held state and the hold resumes:
  195 -> 40 ms a virtual s, the drive 3.1 -> 2.65. Nothing from
  SyncPCEveryInstructionDisabled or -O2 on the sample path's board files
  (2026-09-27).
- CI's runner ran the emulated board slower than the scale the host slept
  by: the STO test's 0.05 board s settle ended before the chain had let go,
  BIF stayed latched through the clear, and the Release image's board group
  ran past 240 s. An emulated board's sleep reads its virtual clock as it
  goes; a group has 480 s (2026-09-27).
- CI's firmware job red on and off since 2026-09-27, red four times from
  8112407: conformance's Renode exited on a DllNotFoundException for its
  world library (build/hosttest/world_emu_<pid>.so). Each test group's
  process swept every world library not locked before building its own;
  Windows locks a loaded one, Linux locks none, and a group's sweep
  unlinked another's before its Renode loaded it. The sweep removes only
  libraries whose process has ended (`tools.emu.world.sweep`); the log's
  tail is on the commit as a comment, readable without a login
  (2026-09-28).
- The drive's ISR called newlib's cosf and sinf ten times and atan2f four
  times a period (the observer: the PLL's angle twice over, the lead, the
  blend's two angles) and lrintf seven: drive_sincos, an atan2 polynomial
  (1.7e-6 rad) and an add-and-truncate put the emulated core's ISR at
  1 445 cycles a period from 1 876, 123 -> 99 M instructions a virtual s.
  The wall barely moved, 3.0 -> 2.95: a period's 57 us are its 17 register
  accesses (~1.3 us each on Renode's IO path), the exception (4.5), the
  plant's step and the conversions (3.5) and the instructions (~10); the
  skip 0.6, no translation flushes (~1 000 blocks a virtual s). Real time
  under the drive is out of the emulator's reach as the firmware stands:
  every access is the ISR's own (2026-09-27).
- The heat's airspeed one input (2026-10-10): `thermal_load_t` carries it and
  the core folds it once into the wash (`thermal_air_rpm` on the cfg's), for
  the board's observer and the world's truth alike; native's and Renode's
  plants fly in the host's, the world's heat step taking the whole load.
