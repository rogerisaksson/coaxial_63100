# Findings

Read this before investigating anything. It records what has been established,
what has been **ruled out**, and what is still open. Several entries here cost
hours and a lot of measurements; re-deriving them is pure waste.

Each entry says how strongly it is held. Treat *confirmed* as settled,
*hypothesis* as untested, and *refuted* as a dead end that already looked
plausible once.

One more axis matters as much as confidence: **whether a finding is about the
design or about one board.** Nearly everything here is design- or
firmware-level and will reproduce on any unit. A few entries are quirks of the
specific board these measurements came from, and they are labelled as such. Do
not build firmware behaviour around those, and do not treat them as a
specification — a component fault on one unit is not a characteristic to
calibrate against.

---

## Confirmed and fixed

### `ADC_PCSEL` accumulated and was never cleared

**HAL only ever ORs into it.** `HAL_ADC_ConfigChannel` does
`hadc->Instance->PCSEL |= (1UL << channel)` (`stm32h7xx_hal_adc.c:2905`), and a
grep of the entire HAL finds exactly two PCSEL writes, both `|=`. Nothing in the
HAL or the application ever cleared it, so every channel ever configured stayed
preselected with its analog path connected to the sampling network.

Read live off the target before the fix: ADC1 `0x288` (bits 3, 7, 9), ADC2
`0x130` (4, 5, 8), ADC3 `0xC03` (0, 1, 10, 11). After: `0x088`, `0x110`, `0x400` —
one channel each, plus the INN bit for differential.

Effect, measured: DC bus channel noise fell from **129.9 mV to 18.7 mV** at the
bus, a factor of seven, and it went from the noisiest of the seven channels to
among the quietest. The other two ADC3 channels improved 15-35 %.

The fix clears PCSEL before each `HAL_ADC_ConfigChannel`. The HAL then sets
exactly the bits needed, **including the negative input's bit** for differential
mode via `ADC_CHANNEL_DIFF_NEG_INPUT` — so the INN mapping does not need
repeating in application code. `ADC3_ScanExperimental` clears it only once, before
rank 1, because its three ranks must accumulate within one `Start`.

Register addresses if this ever needs re-checking: PCSEL is at ADC base + `0x1C`,
DIFSEL at + `0xC0`. ADC1 `0x40022000`, ADC2 `0x40022100`, ADC3 `0x58026000`.

### `ADC_ReadDifferentialVolts` never configured its channel

It did a bare `HAL_ADC_Start` / `PollForConversion` / `GetValue` and interpreted
whatever the ADC was **last set to** as a differential reading. Correct only by
accident: any other read on ADC1 or ADC2 silently turned the 1 Hz heartbeat line
into a misreading of that other channel, so an `s` or `a` console command was
already enough to break it. Adding the Modbus data model made it permanent,
because the map leaves ADC1 on IN9 and ADC2 on IN5.

Confirmed numerically, which is what pinned it: after the map read the NTC on
ADC1 IN9 at 39848 single-ended, the heartbeat reported Phase V as
**+0.7130 V raw=7080**, which is exactly `39848 - 32768` rescaled. PB1 at 975
likewise appeared as Phase W = **-3.2018 V raw=-31793**.

The function is deleted. Both call sites use `ADC_ReadOneChannel`, which
configures the channel and clears PCSEL. **If a phase reading ever looks like
another channel again, check that nothing has reintroduced a read path that skips
`HAL_ADC_ConfigChannel`.**

### `FC 0x0F`/`0x10` with quantity zero answered silence instead of `0x03`

A request for zero items is a well-formed 6-byte PDU declaring an illegal
quantity. The dispatch minimum was 7 bytes, so it never reached its handler.
Fixed by lowering the minimum to 6; the handler re-checks the length against the
byte count before touching any data. Found by conformance testing, not by reading.

### The false HSE boot warning

The old test was `sysClkSrc != RCC_SYSCLKSOURCE_STATUS_HSE`, which only accepted
SYSCLK taken *straight* off HSE and therefore warned on every boot once PLL1 was
in use. `Board_SysClkOnCrystal()` now accepts PLL1 when
`RCC_PLLCKSELR.PLLSRC` is HSE. It lives in the board layer so a test rig can ask
the same question.

### A `%`-formatting precedence bug in the MCP renderer

`'fmt' % (a, b).rstrip()` binds `.rstrip()` to the **tuple**, not to the
formatted string. Needs explicit parentheses around the whole `%` expression.
Worth remembering as a class of bug — and worth noting that the narrow exception
catch in the MCP server (`RigError`, `ValueError`, `KeyError` only) is what let
the resulting `AttributeError` surface instead of being swallowed as an empty
result. **Keep that catch narrow.**

---

### A weaker model answered from memory when a tool call failed

**Measured, 2026-08-22, board at 36.3 °C by `board temp`.** `llama3.1:8b` was
tried against `gemma4:12b` on the bench. It is genuinely faster — three
questions through `dbg.py` in **24.0 s against 31.3 s**, two model calls per
question against three, and ~1.2k prompt tokens against ~2.9k. That is not why
it lost.

Asked for the board temperature it called `analog_read` with `ch="ntc"`, a bare
string where the schema says array. `for item in "ntc"` iterates characters, so
the tool answered `unknown channel 'n'`. The model, given nothing it could act
on, replied: **"The board temperature is 25.00 C."** Three runs, same answer.
25.00 °C is the number this board reports with the AFE off — see the AFE entry
below. It was invented, it was wrong by 11 °C, and it was wrong in the one shape
a reader is least likely to question.

Two fixes came out of it, both in `coaxial_mcp/tools.py`:

- `_names` accepts a bare string, a comma separated string, and a list that
  arrived as text (`"['NTC']"`).
- `coerce` converts every argument to what the tool's own `inputSchema`
  declares, so `samples="100"` and `refresh="true"` work, and what cannot be
  converted is refused *by field name and wanted type* rather than as a
  `TypeError` three frames down. It is on the ollama path only: the MCP server
  gets the same protection from the protocol library, which validates against
  `inputSchema` before a handler is reached.

After the fix `llama3.1:8b` reads the board correctly. It still lost, for a
second reason: it passes thermistor parameters it was never given, sending
`ntc_beta=3950` where the onboard part is a Murata NCU18XH103 at **B=3380**
(`coaxial/scaling.py:94`). The reading came back **34.6 °C** against 36.3 by the
board's own path — a 1.7 °C bias, silent, from a plausible-looking number the
model supplied itself. `gemma4:12b` overrode nothing and landed within 0.5 °C,
and when asked whether the AFE was on it answered by reasoning that the NTC was
*not* reading exactly 25.00.

So: the default stays `gemma4:12b`. The 23 % that `llama3.1:8b` saves is real,
and worth having on a machine that cannot hold 8 GB — but where a model
that invents hardware constants is the expensive kind of fast.

### The local model's OOM crash is llama-server's prompt cache, not our prompt

**Measured, 2026-08-24, reproduced deliberately in three runs.** A bench
session of eight to ten questions through `dbg.py` reliably killed the model
runner partway through: ollama answered 500, reloaded 8 GB, and the session
stalled mid-answer. `%LOCALAPPDATA%\Ollama\server.log`, with the daemon's own
verbose output, says exactly what happened:

```
slot get_availabl: - checking sim = 0.255 (323/1265) > 0.100
srv   prompt_save:  - saving prompt with length 1446, total state size = 342.623 MiB
libc++abi: terminating due to uncaught exception of type std::bad_alloc
llama-server terminated  exit.code=3221226505 (0xc0000409)
```

llama-server keeps a prompt cache of up to 8192 MiB and saves the whole slot
state into it whenever a new prompt shares little of its prefix with the
cached one. Every question here is that case — `Chat` clears its history after
each answered turn on purpose, so `sim` lands at 0.25–0.33 — and the ~340 MiB
save intermittently throws, uncaught. Beside it, context checkpoints are
`320.013 MiB` each with a default ceiling of 32: 10 GB of them, against 8 GB
of weights on a 16 GB card.

`LLAMA_ARG_CACHE_RAM=0` and `LLAMA_ARG_CTX_CHECKPOINTS=0` in the daemon's
environment remove both. Capping the checkpoints at 2 rather than disabling
them was tried first and was not enough - restoring a 311.575 MiB checkpoint
threw `std::bad_alloc` on its own. Off entirely costs nothing here: this loop
clears its history every turn, so a restored checkpoint has nothing to
restore. Twelve questions, 36 model calls, **zero**
`std::bad_alloc` and one model load; the same session untuned crashed in both
attempts. `board_prompt.ps1` now sets them and restarts the daemon once if it
has to — see [MODELS.md](MODELS.md) and `$DaemonTuning` in
`board_prompt/Tuning.ps1`.

Two things worth keeping from how this was found. The variables are
`LLAMA_ARG_*`, not `OLLAMA_*`: `ollama serve --help` does not list them,
because they belong to llama.cpp's own argument parser, which reads them from
the environment ollama hands its runner (`libllama-common.dll` carries the
names). And an already-running daemon keeps the environment it was started
with, so setting them changes nothing until it restarts — including a daemon
started from a shell that was itself started before the variables were set.

---

## Confirmed, not a fault, do not fix

### JTAG connect-under-reset fails; the cabling is fine

Any connect that asserts NRST fails with `Unable to get core ID` /
`DEV_UNKNOWN_MCU_TARGET`. Four dimensions were eliminated:

- **Frequency** — identical failure at 8000, 4000, 2000, 1000, 480 and 240 kHz.
- **Mode** — `mode=UR` and `mode=Normal reset=HWrst` both fail.
- **Host tool** — same on programmer 2.22.0 and 2.23.0, while SWD `mode=UR`
  succeeds on both.
- **The board** — this is the decisive one. With JTAG connected in hotplug mode,
  a hardware reset issued *mid-session* leaves the TAP alive: flash at
  `0x08000000` still reads back the correct vector table. **TDI, TDO and NJTRST
  are correctly wired.**

What remains is the probe firmware's TAP re-initialisation during reset. On the
H7 the JTAG-DP sits in the domain NRST holds, and the probe does the continuous
re-init it does for SW-DP in SWD mode but not the equivalent TAP rescan for JTAG.

**Workaround:** `-c port=JTAG mode=Normal reset=SWrst`, or use SWD. A probe
firmware update is the only thing that might change it; the probe is on
`V3J16M9B5S1` and the `stlink-upgrader` bundle is installed.

### Two ways to leave the core halted

2026-08-24. `find_board.py --power` returned `STM32_Programmer_CLI did not
answer within 15s`. Everything serial after it was silent: `find_board.probe`
scored **0/10** consecutive connects, a raw Modbus frame written straight to
COM4 got nothing, and so did a bare `r` or `?` to the ASCII console. Target
power read **3.27 V** and SWD read **Device ID 0x450** the whole time, so the
board was powered and the debug port was fine.

`-c port=SWD mode=UR --start` brought it back in one command, and `--discover`
answered immediately after.

The check ran `-c port=SWD mode=UR -q` under a 15 s `subprocess.run` timeout.
Connect-under-reset asserts NRST; killing the programmer there can leave the
target held in it, and a halted core answers nothing on USART3. `check_power`
uses `mode=HOTPLUG` now, which never touches reset — measured, it reads the
same 3.27 V and leaves the board answering.

Worth naming what this cost: `link_diagnose` calls `check_power` as step 1 and
then asks in step 4 whether the board answers. The checklist was able to cause
the silence it reported, and an earlier entry in this document blamed the
hardware for it.

**`-hardRst`.** A full flash cycle ending in `-hardRst` programmed and verified
correctly but the serial line stayed **completely silent for 10 s**. An explicit
`--start` was needed. End with `--start`.

### Phase V sits ~0.85 V from U and W — one unit, suspected bad op-amp

Recorded only so nobody spends another afternoon on it. There genuinely is that
much across the differential pair; the board owner suspects a bad op-amp on this
specific board and will check it. **Not an ADC fault, not PCSEL, not a scaling
problem, and not a property of the design.** Do not calibrate around it.

### The NTC channel is not anomalous, it is quiet

An earlier investigation treated bit-exact repeated readings as a defect. They
are not. The node is 10k||10k = 5 kOhm against a 15 nF capacitor, about 2.1 kHz,
so Johnson noise is ~0.4 uV rms against a 50 uV LSB — roughly **120x below one
LSB**. Bit-exact readings need no explanation.

A timestamped 30-sample series settled it: the first ten averaged raw 40790 and
the last ten 40810, i.e. +20 LSB over 47 s with about +/-15 LSB scatter. That is
slow warming, not noise. NTC scatter is comparable to the *unfiltered* PB1
channel, so what remains is ADC-internal noise rather than the source node. The
earlier runs of bit-exact 40448 never reproduced. `0x9E00` looking tidy in hex
means nothing.

### The UART overrun flag latches and kills reception

If RX bytes arrive faster than they are consumed, ORE latches and from that point
`HAL_UART_Receive` never returns `HAL_OK` again — the command interface is dead
until reset. Nothing in the original code cleared it. This was hit for real by
spamming console commands.

`dev_usart3.c` clears ORE/FE/NE/PE through `ICR` and reports the fault upward, so
the protocol discards the affected frame and carries on. Note that reading `RDR`
alone does **not** clear ORE — it needs `ORECF`.

### ADC differential offset calibration is not repeatable across boots

`HAL_ADCEx_Calibration_Start(..., ADC_DIFFERENTIAL_ENDED)` runs once at boot with
AFE_ON still low, i.e. against an unpowered and undefined input. Measured over
three boot cycles with the same binary and the same hardware: Phase U landed at
+0.147 / +0.048 / +0.092 V and Phase W at +0.044 / -0.070 / -0.010 V. Roughly
**100 mV of boot-to-boot spread.**

Within a single run the readings are stable to a few mV over 45 s, and AFE
off-time (0.3 s versus 15 s) makes no difference, so it is the calibration and
not front-end settling. The DC bus, being single-ended off a real rail, is
unaffected: 24.91-24.95 V in every cycle.

**Still open.** The likely fix is to enable AFE_ON, let it settle, and calibrate
after — which would put the calibration against a powered reference. Untested.

---

## Refuted — plausible, wrong, do not revisit

### "Sampling time is too short for the NTC divider"

`ADC_SAMPLETIME_1CYCLE_5` for a 5 kOhm source looked far too short. It is not:
the node carries 15 nF, which supplies the sample-and-hold charge. The board
owner pointed this out and it is correct.

### "DIFSEL is silently not applied"

DIFSEL may only be written with `ADEN = 0`, so it looked like the
single-ended/differential selection would be dropped after the first
configuration. But `HAL_ADC_Stop` calls `ADC_Disable`, so every
`HAL_ADC_ConfigChannel` runs with the ADC disabled and the write goes through.
Confirmed empirically too: DIFSEL reads `0x8` / `0x10` / `0x2` — exactly one
correct bit each.

### "NJTRST is tied to the reset net"

Refuted by the hotplug-plus-hardware-reset test above. The TAP survives a reset.

### "A VREF sag explains the DC bus shift at 475 MHz"

The DC bus mean fell 249 mV, exactly -1.00 %, when SYSCLK went from 75 to
475 MHz. A clean round percentage on an absolutely-scaled channel suggested the
reference had sagged.

It had not. Measuring all three absolutely-scaled single-ended channels at once:
IN5 **+0.10 %**, IN10 **-1.06 %**, IN11 **-9.69 %**, with standard errors of
0.7-3.3 LSB. A common reference shift would move all three by the same amount.
They differ by hundreds of standard errors. The -1.00 % was a coincidence in the
third decimal.

**IN11 (PC1) moving 9.7 % is unexplained** and IN11 has no signal name in the
firmware. Left open.

### "PCSEL accumulation explains the Phase V offset"

It does not — see Phase V above. PCSEL-never-cleared is a verified fact about the
HAL and it did measurably affect the DC bus channel, but the Phase V offset is
analog and real.

### "The local model's OOM is the prompt getting too long"

The obvious hypothesis, and wrong. It survived long enough to nearly justify
the wrong fix: tool results were being appended to the conversation unbounded
(`build_firmware` and `run_tests` returned whole build logs), so a prompt that
grew past `num_ctx` looked like the explanation.

The crashing prompts were **1249 and 1446 tokens of an 8192-token window**,
and the state that failed to allocate was 339 MiB and 342 MiB — the same
number, because llama-server's checkpoint size is fixed by the context window,
not by how much of it is used. Shortening prompts by 30 % changed the crash
rate not at all. See "The local model's OOM crash is llama-server's prompt
cache" above for what it actually was.

The bounded tool results and the `num_ctx` budget (`coaxial_ollama/context.py`)
were kept anyway — an unbounded build log in a prompt is a real defect, and
one worth fixing on its own terms. It is just not this one, and a session that
believed it was would still be crashing.

---

## Open

| Question | State |
|---|---|
| Calibration running against an unpowered reference | Diagnosed, fix proposed and untested |
| IN11 (PC1, `Cinj`) moved 9.7 % between 75 and 475 MHz | Unexplained. The channel was unnamed when this was measured; it is `Cinj` now, and what it measures is still not recorded here |
| DC bus read twice in one sweep differs by 25-35 LSB | The two read paths give systematically different values, ~29-42 mV at the bus. Not PCSEL — it persisted after that fix. |
| Phase V op-amp offset | Board owner's, deliberately deferred |
| PE15 (`nFAULT`) reads 0 whenever AFE_ON is high | 2026-08-24, both boards. The level was measured long before the pin was named and did not change with the name; what it means did. Active low, 0 with the front end powered reads as a fault asserted. Not established whether that is a real fault, a pull following the AFE supply, or a polarity that does not match the name. Both suites still use the inverse relation as an independent witness that a pin write landed - that use holds either way. |

---

## Measurement methodology that worked

Worth reusing, because each of these caught something reasoning alone did not.

- **Two independent paths to the same number.** The binary burst command and the
  old ASCII heartbeat agree at 5.17-5.75 mV rms against 5.20-5.83 mV. Where they
  disagreed, that disagreement *was* the bug — twice.
- **An independent implementation for the test master.** `test_conformance.py`
  has its own CRC and framing, deliberately not the library's, so a shared wrong
  assumption cannot hide a defect. Its CRC is checked against the catalogue value
  first.
- **A physical witness for a logical write.** PE15 follows AFE_ON, so writing the
  coil and reading the discrete input proves the write reached the pin and not
  just the register.
- **Separate drift from noise before drawing conclusions.** Removing a linear
  trend cut the DC bus standard deviation from 33.8 to 14.1 LSB. Without that
  step the noise figure was more than twice the truth.
- **Round numbers deserve suspicion, not confidence.** The -1.00 % DC bus shift
  and the tidy-looking `0x9E00` both read as signatures and were both
  coincidences.
