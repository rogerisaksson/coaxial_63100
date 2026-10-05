# Hardware

What a reading means. Authorities: `electronics/` (schematic, BOM,
pick-place) for what is fitted; `board/src/board_io.c` (pins, parts),
`board_adc.c` (channels), `board_cal.c` (scaling) for what the firmware
believes. Pins, channels and parts are answered by the bus (`0x6D`), not
tabled here. Nothing is measured against an instrument unless it says so.

## MCU, clocks, memory

- STM32H753VIT6, 25 MHz crystal, PLL1 M2 N76 P2: SYSCLK 475, HCLK 237.5,
  TIM1 237.5 MHz. ADC kernel 37.5 MHz (0x45 reports). SPI2 kernel 190 MHz
  (PLL1Q), SPI4 118.75 MHz (APB2); both report their bitrate (IMU op 3,
  angle op 6). CYCCNT wraps every 9.04 s.
- I-cache on; D-cache off (record read back through a pointer; sample path
  data in DTCM).

| Region | Use |
| --- | --- |
| ITCM 64 K | sample-path code (`.itcm`, copied by `Board_Early`), ~30 K |
| DTCM 128 K | .data, .bss, 1 K stack, log ring; top 32 B = handover slot |
| AXI SRAM 512 K | 448 K DAQ ring (`.buffers`, NOLOAD) |
| D2 SRAM 0x30000000, 288 K | application, linked to run here; header +0x400 |
| Flash 0x08000000, 128 K | bootloader |
| Flash 0x08020000, 1792 K | store: the application's sealed copy (BOOT.md) |
| Flash 0x081E0000, 128 K | calibration record, magic 'CX63', CAL_VERSION 15 |

## Conversions

- Phase: 2x 7 mOhm shunts = 3.5 mOhm, THS4551 gain 4.5455 -> 15.909 mV/A;
  100 A = 48 % of span. Schematic arithmetic (2026-08-26), not spanned.
  Noise floor 0.35-0.41 A rms. Group delay 60 ns (simulation).
- DC link: 49.9 k / 2.2 k = 78.15 V FS (invariant 11). Spanned vs DMM
  2026-08-30: -32 418 ppm on channel 5.
- NTC: Murata NCU18XH103D60RB, R25 10 k, B 3380 K, vs 10 k 0.1 %. 30 mK
  resolution.
- +5V sense 10 k / 10 k; Vgate 57 k / 10 k (ratio 6.70), traced 2026-08-27.
- Differential channels are offset binary, 32768 = 0 V.

## Reference and AFE_ON

VREFBUF off; U2 REF2033 makes `+3V3_ref` and `+1V65_bias` (they track).
PB2 AFE_ON powers AFE, reference, NTC divider, A1335, BNO085 and +5, the STO
chain's pilot detector (U16): off, the chain is down. The bench board is
unmodified (R93 on +5, not 3V3D): there AFE_ON removes the gate drivers'
supply and PE15 follows it inversely. Off:
mid-scale everywhere, NTC 25.00 C (invariant 9). The rail is reference
counted (`board_power.c`): host claim dropped after 10 s silence, others
hold 3 s leases.

## Gate stage

- TIM1 centre-aligned, 50 kHz, RCR 0 (update at both ends). CH5/TRGO2
  triggers injected ADC, lead 15 ticks. The drive, unskewed, runs RCR 1: the
  update at the overflow alone, its interrupt off, the triples written from
  ADC3's interrupt 15 ticks past the overflow. BKIN = PE15 active low, AOE off,
  OSSI/OSSR on. Gate pins VERY_HIGH speed.
- Dead time: record `deadtime_ns` 30 -> DTG 8 = 33.7 ns (floor 20 ns, DTG
  max 127 = 535 ns). `.ioc` DTG 19 holds until the record loads. Trimmed
  against the supply's OCP.
- Op 1 alone sets MOE, at zero duty. Op 2 [+ period count], op 8 Q16.16
  dither, op 10 alternate, op 6 break bypass (reset restores). The thermal
  envelope drops MOE by the break's path.

## STO chain

The master (the host's end of A1/B1, UART5's pair) drives a common-mode
pilot, 1.5 V at 5 kHz from its amplifier (`sto.asc`'s PAM8406). U16C squares
it at the zero cross (49.5 mV) and pumps Cinj through C101; U16B dumps Cinj
past TP67's 543 mV. U4 (TPS3840, VIT- 3.0 / VIT+ 3.1 V) on Cinj lets RESET go
216 us after VIT+. PA10 KEEPALIVE toggles at 200 kHz (a 100 kHz square wave)
-> R72 330 / C71 100 nF -> charge pump into Clevel, clamped by D10 at RESET's
0.8 Cinj; Q10A on Clevel releases U11 (FAULTOUT = PGD) at 1.55 V, trips at
1.52 V; FAULTOUT is PE15 (BKIN) and U9's enable for +15V7.
main() sleeps in WFI with AFE_ON low and MOE clear: the keepalive then pauses
up to a SysTick (1 ms), past the latch's hold. AFE_ON (the interlock's read) or
MOE keeps it pumping; a gap over ~120 us drops Clevel with PA10 held low.
Cinj (PC1) = recovered pilot, Clevel (PB1) = integrator, both straight to the
pin: Cinj's 3.70 V is over VDDA and reads full scale.
The model (`world/src/world_sto.c`, 2026-09-27): released 1.54 ms after the
pilot, Cinj 3.70 V, Clevel 2.86 V (keepalive every 5 us; 2.49 V every 50 us),
+15V7 14.9 V; the pilot lost trips it in 0.8 ms, +5 in 0.1, the keepalive in
0.13, PGD at once. `rig.pilot(volts, hz)` sets an emulated or simulated
master's. `GateStage.interlock()` wants Cinj >= 3.0 V, Clevel >= 2.0 V; the
unmodified board reads 0.77 / 0.06 V (2026-08-27), so its sessions arm with
`ignore_interlock=True, bypass_sto=True`. `tools/bench/sto_probe.py` reads it.

## SPI sensors

- BNO085 (SPI2, mode 3, 2.97 MHz, CS PB12 held across header and cargo).
  Advertisement 276 B (`IMU_BUF` 320). NRSTN/BOOTN active low. H_INTN read
  before every transfer; WAKE (PS0) required for writes. Reports: 0x01 accel
  Q8, 0x02 gyro Q9, 0x03 mag Q4, 0x05 rotation vector Q14.
- A1335 (SPI4, /64 = 1.86 MHz, CS PE4; DMA1 streams 0/1, buffers in AXI SRAM).
  20-bit packet, two frames per read, stepped from main(). ANG
  12 bits x 360/4096; TSEN 1/8 K; FIELD gauss. Register map from a reference
  implementation.

## Serial

USART3 PB10/PB11 -> ST-Link VCP, 115 200 (the recovery path). USART2, UART5
-> THVD1450 at the record's `link_baud`; RE tied low (each hears itself);
UART5 termination on PE14.

## Calibration record

`board_cal.c` holds the ids, units and schematic defaults; `0x6E` dev 3
reads and writes them. Ids 15-44 (motor, drive gains, dead-time table) are
placeholders until `tools/bench/commission.py` writes them. A stored record of
another version is refused, except the two before the firmware's (prefix
layouts), taken up with later fields at defaults.

## Thermal

`thermal/src/thermal.c`: 20 nodes, 30 edges, fitted to one camera campaign
(2026-08-28, 20 C room, 25 min per state; board is a dead patch of soldermask):

| State | board C | MCU | regulators | bridge | AFE |
| --- | --- | --- | --- | --- | --- |
| passive | 30.0 | +15.0 | +8.0 | +1.0 | +1.0 |
| AFE on | 31.1 | +14.2 | +8.1 | - | +5.9 |
| traffic | 31.4 | +13.6 | +7.6 | - | +5.9 |
| switching | 40.0 | +17.3 | +20.0 | +10.1 | 0.0 |

Measured: board-to-air 8.33 K/W, 49 J/K (tau 6.8 min). Everything else in
`thermal_defaults` (patch areas, per-leg paths, part capacities, motor
nodes, losses) is derived or estimated; `test_thermal_core` holds the
numbers to the C. Observer steps 100 ms, NTC sampled every 30 s.

## Other

- `host/coaxial/model/inverter.py` carries the power-stage constants traced from
  the LTspice submodule `electronic_simulations` (Q_RING assumed 1.0).
- `render/models/coaxial_63100.stl`: 100 mm disc, 10 mm bore.
- `electronic_simulations/3d_models/Coaxial 63100.glb` (2026-10-05; Draco,
  2 630 parts, read by their bounds): the laminate 100 mm round, 1.59 mm;
  parts 10.8 mm over it (one of 10.4 x 13.7 mm at r 27 mm, a header 9.3)
  and 2.3 under, 14.7 mm in all. Five 11.5 mm terminals through it at r
  40-43 mm, 7.1 mm up: the phases' three side by side, the supply's two a
  quarter turn on; two 4.6 x 6.8 mm parts beside each phase's, r 34-45 mm.
  The power on the rim, the MCU inside 70 mm; the parts' boxes 4 338 mm^2
  on top, 860 under - a 70 mm disc is 3 848 a side.
- `electronic_simulations/3d_models/U8IILite 85KV.glb` (2026-10-05; mm,
  16 parts): T-Motor's U8 II Lite, its can 87.1 x 26.95 mm, the stator
  80.5 x 9.6 mm, its windings 11.1 mm tall.
