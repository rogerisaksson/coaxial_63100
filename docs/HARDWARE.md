# The board

## What it is

A **coaxial BLDC inverter**: a three-phase motor drive on a PCB mounted
coaxially behind the **stator** of an outrunner. The name encodes the rating.

*Coaxial* is the mechanical arrangement and nothing else. Nothing on this board
is a coaxial cable or a coaxial connector, and the serial link is not coaxial —
see [The link](#the-link).

| | |
|---|---|
| **63** | maximum DC link voltage, 63 V |
| **100** | maximum phase current, 100 A instantaneous within the FETs' SOA |

"Instantaneous within the SOA" is the operative phrase: 100 A is a peak the
switches survive for a bounded time, not a continuous rating. Anything in this
repository that logs or limits current should be read against that.

Two consequences of the mechanical arrangement, both of which shape what the
measurements mean:

- **Thermal.** A board sandwiched behind the stator, inside a spinning rotor
  can, has poor and rotor-speed-dependent airflow, so the NTC channel is a real control input
  rather than a diagnostic nicety.
- **Electrical.** The phase sense sits inside a switching bridge. Any noise
  figure taken with the bridge idle is optimistic, and a figure taken while the
  FETs commutate is the only one worth recording.

The board also carries further subsystems beyond the analog front end. Only what
the firmware currently configures is documented below; two ADC channels are wired
and read but have no assigned signal in the firmware, and PE15 is an input whose
role beyond tracking AFE_ON is not recorded here.

## Silicon

STM32H753VIT6, Cortex-M7, Device ID `0x450`, **Rev V**. Programmed over SWD or
JTAG.

Rev V matters: the PLL1 VCO runs at 950 MHz, which is inside Rev V's 192-960 MHz
window but **outside** Rev Y's 836 MHz. The clock configuration is tied to this
silicon.

## Clock tree

Read out of the RCC registers and confirmed against `SystemCoreClock` in RAM.

| Stage | Frequency |
|---|---|
| HSE crystal | 25.000 MHz |
| /DIVM1 = 2 | 12.500 MHz PLL reference |
| xN = 76 | 950.000 MHz VCO |
| /P = 2 | **475.000 MHz** SYSCLK |
| /D1CPRE = 1 | **475.000 MHz** CPU (`SystemCoreClock`) |
| /HPRE = 2 | **237.500 MHz** HCLK |
| APB1/2/3/4, all /2 | 118.750 MHz |

Requires `VOLTAGE_SCALE0` and `FLASH_LATENCY_4`. The ADC kernel clock is
**separate and unchanged**: PLL2P at 75 MHz with `ADC_CLOCK_ASYNC_DIV2`, so ADC
sampling time in absolute terms does not move when SYSCLK does.

There is an **HSE and no RTC/LSE**, so everything on board derives from that one
crystal. The only HSE-independent frequency check is the heartbeat period against
the host clock, which measured **-20 ppm at 75 MHz and -13 ppm at 475 MHz**, both
comfortably inside a 25 MHz crystal's spec. Do not confuse that absolute accuracy
with the +/-5 ppm short-term jitter the old stability report printed.

## ADC channels

Seven configured channels, all inputs. `Board/Src/board_adc.c` holds
`s_adcTable`, the board reports it over `channels`, and the index into it is
what every `mask` and `adc_chan` argument means. The table below is what it
said on 2026-08-24; the board is what to ask.

| Index | ADC | Channel | Pins | Mode | Signal |
|---|---|---|---|---|---|
| 0 | ADC3 | IN1 | PC3_C / PC2_C | differential | Phase U |
| 1 | ADC1 | IN3 | PA6 / PA7 | differential | Phase V |
| 2 | ADC2 | IN4 | PC4 / PC5 | differential | Phase W |
| 3 | ADC2 | IN5 | PB1 | single-ended | *(none assigned)* |
| 4 | ADC1 | IN9 | PB0 | single-ended | NTC |
| 5 | ADC3 | IN10 | PC0 | single-ended | DC bus |
| 6 | ADC3 | IN11 | PC1 | single-ended | *(none assigned)* |

16-bit resolution. One LSB is **100.7 uV** at the pin for differential (offset
binary over 32768) and **50.354 uV** for single-ended (over 65536).

Indices 0-2 are the three motor phases of the inverter, one per ADC so all three
can in principle be sampled simultaneously rather than sequentially — the current
firmware reads them one at a time, which is a deliberate simplification, not a
constraint of the wiring. They are differential and sit behind AFE gain the
firmware does not know, which is why nothing in this codebase converts them to
amperes or volts-at-the-phase. At rest they read within a few tens of millivolts
of zero at the ADC pin.

Indices 3 and 6 have no signal name in the firmware, only a pin. Do not invent
one for them. Index 6 (PC1) is also the channel that moved 9.7 % across a clock
reconfiguration for reasons still unexplained.

## The ADC reference comes from outside the MCU

`Core/Src/stm32h7xx_hal_msp.c` does this in `HAL_MspInit`:

```c
HAL_SYSCFG_DisableVREFBUF();
HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);
```

The internal reference buffer is **off** and the VREF+ pin is **high impedance**,
so the ADC reference is driven externally. That is the register-level mechanism
behind everything in the next section: the reference is part of the AFE, so
switching the AFE off removes the ADC's reference, not merely its signal
conditioning.

It also means `ADC_VREF_VOLTAGE 3.3f` in `board_adc.c` is an assumption about an
external rail, not a property of the chip. Measure it if a reading needs to be
better than a percent — the DC link conversion scales directly with it.

## The AFE switch, and why it is not just a GPIO

**PB2 (AFE_ON) powers the amplifier chains AND the ADC voltage reference.**

With it off, every channel reads exact mid-scale: differential codes exactly 0,
single-ended exactly 32768. The NTC then reports **exactly 25.00 C**, because
mid-scale puts its divider at R25, which is 25 C by definition. That is a
plausible-looking number which is not a measurement — which is why it gets its
own subsystem in the host library.

What to do about it turned out to be two different answers. The library's
cooked readings — `read_all`, `ntc_temperature`, `dcbus_voltage` — still raise,
because each claims a physical quantity and with the reference unpowered there
is none to claim. The `analog_read` **tool** does not: it returns the codes
under a line saying what they are. Refusing there was tried, and it caused the
thing it was meant to prevent — asked for the raw codes with the AFE
deliberately off, a model with no numbers to report wrote "Mid-scale … 25.00 C"
out of the warning text itself. Codes under a label beat a refusal that gets
paraphrased into data.

**PE15 follows AFE_ON inversely** — measured, not assumed: 1 while the AFE is off,
0 once it is on. That makes the discrete input an independent witness that a write
reached the pin rather than only the register, and both test suites use it so.

## Scaling

### DC link, PC0 / ADC3 IN10 — absolute

External divider 49.9k / 2.2k, so the ratio is **23.68182**. One LSB is 50.354 uV
at the pin but **1.1925 mV at the link**.

The divider is sized for the rating with margin: full scale is
3.3 V x 23.68182 = **78.15 V**, which is **24 % of headroom over the 63 V
maximum**. Worth knowing before anyone "improves" the divider — losing that
headroom means a transient above 63 V clips silently instead of being recorded,
and on an inverter the transient is exactly what you want to see.

The reading scales directly with VREF, so an error in the reference is an error
in the result; pass a measured rail if you need better than a percent.

### NTC, PB0 / ADC1 IN9 — ratiometric

Murata NCU18XH103D60RB: R25 = 10k +/-0.5 %, B25/50 = 3380 K +/-0.7 %, confirmed
against Murata's published spec rather than assumed. The divider has the NTC high
side from the reference rail and a 10k fixed resistor to ground.

Because `VREF / v_node` reduces to `65536 / raw`, the resistance is
`10k * (65536/raw - 1)` and **VREF cancels out entirely**. The temperature is
immune to reference error. The DC link has no such cancellation, with a useful
consequence: a reference sag shows on the bus reading but not on the temperature,
which distinguishes a reference shift from a real change on the link.

Self-heating: the divider draws about 205 uA with roughly 1.245 V across the
thermistor, so about **256 uW** in the element. For a typical 0603 dissipation
constant of 1-2 mW/C that reads **0.13-0.26 C high**. Murata does not publish a
dissipation constant for NCU18 in the datasheet at hand, so that range is
estimated, not verified.

## Discrete I/O

**Ask the board, not this table.** Command `0x6D channels` reports every pin
with its direction and whether a fixture may drive it, and
`Board/Src/board_io.c` is where that list lives. What follows is what it said
on 2026-08-24, kept for the notes attached to it — a pin added to the firmware
appears in `board_info` without anyone editing this page, and if the two ever
disagree the board is right.

| Pin | Direction | Role |
|---|---|---|
| PB2 | output, push-pull | AFE_ON. **Low at boot.** |
| PE15 | input | follows AFE_ON inversely |
| PB10, PB11 | AF7 | USART3 TX / RX, 115200 8N1, **polled: no NVIC, no ISR** |
| PA13-PA15, PB3, PB4 | AF | 5-pin JTAG debug port |

The first two rows are the digital **I/O** — what a fixture may read or set.
The last two are the bus and the debug port: `channels` reports them under a
separate kind, raw pin access refuses them in every mode, and they are not
channels. Driving the first pair severs the link the command arrived on; the
rest cost the ability to reflash.

USART3 has `HAL_UARTEx_DisableFifoMode()` applied, so there is **no RX FIFO** — a
single byte of overrun loses data.

## The link

One UART, two ways off the board. Modbus RTU rides either; so does the text
console. The protocol is the same on both — see
[PROTOCOL.md](PROTOCOL.md).

| Path | What the host opens | Used for |
|---|---|---|
| Debug probe | the probe's virtual COM port | bench work: this is what `coaxial`, the MCP server and `dbg.py` talk to |
| RS485 | a transceiver on the board, off to a field bus | a drive installed in a machine, where the probe is not there |

Nothing here is coaxial. The name is the mechanical arrangement — the PCB
behind the stator — and it says nothing about the cabling.

What the firmware does **not** do: the `.ioc` configures USART3 as a plain
asynchronous UART (`VM_ASYNC`) on PB10/PB11, and no driver-enable pin is
configured anywhere — the only GPIO in the file are PB2 out (AFE_ON) and PE15
in. Half-duplex RS485 direction control is therefore not the firmware's, as it
stands.

## What this document deliberately does not contain

No measured values, no limits, no expected levels.

This board is a **dumb slave**. It measures and reports raw ADC codes; it does
not know what "good" is and neither does this document. Numbers taken from one
board on a bench become a specification the moment somebody writes them down as
one, and then a later unit fails a test against a sample rather than against the
design.

Where those numbers belong:

| | |
|---|---|
| **Limits** | the test plan — a TestStand sequence, a pytest fixture, a YAML per variant. Somewhere a process engineer can read it, revise it under version control, and show it to an auditor. |
| **Truth** | calibrated instruments on the line. A DMM across the DC link, an electronic load on the phases, a known temperature at the thermistor. The board's own reading is uncalibrated by construction: its reference is an external rail it cannot measure, and its ADC is part of what is under test. |
| **Records** | the line's test database, per serial number, against the calibration certificates that were current that day. |

`host/examples/pytest_production_line.py` is a template showing that boundary,
with the limits fixture left deliberately empty.

The one exception is the board's **self test**, which returns pass or fail for
things provable from its own registers and flash — a locked PLL, a checksum, the
PCSEL state. Those need no external reference, so they need no external limit,
and the verdict is legitimately the firmware's. Everything else it reports as an
INFO value for the executive to judge. See PROTOCOL.md, command `0x6C`.

On the particular unit these documents were written against, Phase V reads
about 0.85 V away from Phase U and W. There genuinely is that much across the differential pair;
the board owner suspects a bad op-amp on this board and will check it. **It is a
component fault on one unit, not a property of the design** — so do not calibrate
around it, and do not chase it as an ADC or scaling problem.
