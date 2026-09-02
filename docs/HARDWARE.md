# Hardware

What is on the board and what a reading means. The authority on what is
fitted is `electronics/` (the schematic and `Coaxial 63100 BOM.csv`);
the authority on what the firmware believes is `board/src/board_io.c`
(pins and parts), `board/src/board_adc.c` (the channel table) and
`board/src/board_cal.c` (every scaling number). Nothing here is
measured against an instrument unless it says so - invariant 7 and
invariant 10.

## MCU, clocks, memory

STM32H753VIT6 (U3), 475 MHz from a 25 MHz crystal (X1, ABM11W): PLL1
M2 N76 P2 gives SYSCLK 475 MHz, HCLK 237.5 MHz, and TIM1's kernel clock
237.5 MHz. The ADC kernel clock is PLL2 at 75 MHz through DIV2 =
37.5 MHz; command 0x45 reports it. SPI2's kernel clock is 190 MHz,
SPI4's 100 MHz. `DWT->CYCCNT` runs at SYSCLK and wraps every 9.04 s.
The instruction cache is on, the data cache off.

| Memory | Use |
|---|---|
| DTCM 128 KB | `.data`, `.bss`, the 1 KB stack, the 1024 x 16 B log ring |
| AXI SRAM 512 KB | the `.buffers` NOLOAD section - the 448 KB DAQ ring |
| Flash bank 2 sector 7, 0x081E0000 | the calibration record, magic 'CX63', CAL_VERSION 9, padded to a 32-byte flash word |

## ADC channels

Three converters, 16-bit. Every read path calls `HAL_ADC_ConfigChannel`
and clears `PCSEL` first (invariant 6). Differential channels come back
as offset binary, 32768 = 0 V. The table, `board_adc.c`:

| # | Signal | ADC | Channel | Pin | Mode | Unit |
|---|---|---|---|---|---|---|
| 0 | Phase U | ADC3 | IN1 | PC3_C / PC2_C | differential | A |
| 1 | Phase V | ADC1 | IN3 | PA6 / PA7 | differential | A |
| 2 | Phase W | ADC2 | IN4 | PC4 / PC5 | differential | A |
| 3 | Clevel | ADC2 | IN5 | PB1 | single | raw |
| 4 | NTC | ADC1 | IN9 | PB0 | single | C |
| 5 | DC bus | ADC3 | IN10 | PC0 | single | V |
| 6 | Cinj | ADC3 | IN11 | PC1 | single | raw |
| 7 | +5V | ADC1 | IN18 | PA4 | single | V |
| 8 | Vgate | ADC1 | IN19 | PA5 | single | V |
| 9 | MCU die | ADC3 | VSENSE | internal | single, 810.5 cycles | C |

Command 0x42 reports the rows with raw, microvolts and the scaled
value; `0x6D` kind 0 the same rows without readings. The two supply
senses were traced on the MCU sheet 2026-08-27: R113 is a 10 k array
whose four elements are GND, +5, +15V7 through R119 47 k, and GND, so
PA4 sits on 10 k / 10 k and PA5 on 57 k / 10 k (ratio 6.70).

## The reference and AFE_ON

VREFBUF is disabled and VREF+ left high-impedance: the analog front end
drives the ADC reference. Its source is U2, a REF2033, which produces
`+3V3_ref` and `+1V65_bias` from one die - reference and mid-point
track because they are the same part. The 3.3 V lives in the
calibration record (`vref_uv`), where a calibrated meter beats a
datasheet tolerance.

PB2 is AFE_ON. High, it powers the AFE, the reference, the NTC
divider, the A1335 and the BNO085; the same signal removes the gate
drivers' supply through the STO chain, and PE15 follows it inversely.
With AFE_ON low every channel reads exact mid-scale and the NTC exactly
25.00 C - mid-scale puts the divider at R25 by definition. Invariant 9:
AFE_ON decides what a reading means. The rail is reference counted
(`board_power.c`): users are the host, the thermal observer, the IMU,
the angle sensor and the DAQ; the host's claim is unleased and dropped
after 10 s of silence, the others hold 3 s leases.

The BNO085 is the worse case: unpowered it still drives MISO, still
resets and still advertises - a valid 276-byte advertisement reads
back - while acting on no write. `Board_ImuInit` refuses while PB2 is
low, and `power` is a column of the parts list for this reason.

## Phase current

Each phase carries RU1 || RU2, two WSHM28187L000FEA 7 mΩ shunts in
parallel = 3.5 mΩ, into a THS4551 (OP1) with Rf 1.5 k / Rg 330,
gain 4.5455 V/V: 15.909 mV/A. On the 3.3 V reference 100 A is 48 % of
the differential span. The phase gain was traced off the schematic
2026-08-26 and has not been spanned against an instrument; the
record's `chan[i].gain_ppm` is where a span goes. The measured noise
floor is 0.35 to 0.41 A rms per phase (FINDINGS); the sense chain's
group delay is 60 ns from the AFE simulation
(`coaxial/inverter.py`).

## DC link

R12 49.9 k over R11 2.2 k: 78.15 V full scale on a 63 V rating, 24 %
headroom - the over-rating transient is what should be recorded, not
clipped (invariant 11). This is the one channel spanned against an
instrument: 2026-08-30 the board read 31.04 V against 30.05 V on a
DMM, -32 418 ppm, saved to the record on channel 5.

## NTC

Murata NCU18XH103D60RB, R25 10 k, B25/50 3380 K, against R100
10 k 0.1 % (ERA-3AEB103V). The firmware converts with the B equation
from the record's `ntc_r25_ohm`, `ntc_beta_mk`, `ntc_rfixed_ohm` and
`ntc_t25_ck`. The NTC sits in the drivers' hot spot, resolves 30 mK
and is the thermal observer's reference; the A1335's TSEN measures its
own die, quantises at 0.125 K and is reset every time AFE_ON breaks -
measured 2026-08-28 it fell 1.88 K during a run that warmed the board.

## Digital pins

`s_digital` in `board_io.c`, what `0x6D` kinds 1 and 2 answer from:

| Pin | Signal | Direction |
|---|---|---|
| PB2 | AFE_ON | out |
| PE15 | nFAULT / TIM1_BKIN | in, reserved |
| PE14 | UART5_TERM, the 120 Ω termination | out |
| PA10 | KEEPALIVE, the STO pilot | out |
| PE8 / PE9 | TIM1 CH1N / CH1 = PWMUL / PWMUH | reserved |
| PE10 / PE11 | TIM1 CH2N / CH2 = PWMVL / PWMVH | reserved |
| PE12 / PE13 | TIM1 CH3N / CH3 = PWMWL / PWMWH | reserved |
| PB10 / PB11 | USART3 TX / RX, the console | reserved |
| PA13, PA14, PA15, PB3, PB4 | JTAG / SWD | reserved |
| PB12 .. PB15 | SPI2 NSS / SCK / MISO / MOSI, the BNO085 | reserved |
| PD8 | H_INTN, the BNO085's interrupt | reserved |
| PD9 | PS0 / WAKE | reserved |
| PD10 | NRSTN, the BNO085's reset | reserved |
| PD11 | BOOTN | reserved |
| PE2, PE4, PE5, PE6 | SPI4 SCK / NSS / MISO / MOSI, the A1335 | reserved |

Reserved pins are refused by the pin commands with the reason: driving
the console pins severs the link the request came on, the JTAG pins
cost the ability to reflash, the gate pins take a leg off the timer,
and PE15 disconnects the break from the timer silently until the next
reset.

## Parts

`s_parts` in `board_io.c`, what `0x6D` kind 4 answers. `state` is
probed on request, not stored.

| Name | What | Where | Power |
|---|---|---|---|
| STM32H753VIT6 | the MCU, 475 MHz | U3 | - |
| BNO085 | 9-axis IMU, SHTP | SPI2, U13 | AFE_ON |
| A1335 | magnetic angle sensor | SPI4, U14 | AFE_ON |
| AFE | phase chains + ADC ref | PB2 switches it | - |
| UART5 termination | 120 Ω across the pair | PE14 switches it | - |
| 2EDL8034 x3 | half bridge gate drivers | PE8..PE13, TIM1 | STO chain |
| IAUCN10S7N021 | bridge FETs, 63 V 100 A | HalfBridge x3 | STO chain |
| NTC | thermistor | ADC3 | AFE_ON |
| DC link divider | 49.9k/2.2k, 78.15 V FS | ADC | AFE_ON |
| USART3 | console or Modbus RTU | PB10/PB11 | - |

From the BOM, the parts behind them: U2 REF2033AIDDCR (reference),
U5 / U6 THVD1450DGKR (RS485), U7 LDI92-05EN, U8 / U9 MP4541GN-Z
(bucks), U10 LM66100, U11 NL7SZ97, U12 LM5069MMX-1 (hot swap),
U16 TLV3492, U1 LDI8119-3.3, U4 TPS3840PL30 (supervisor), OP1
THS4551IDGKR, OP2 ADA4891-4, Q1 / Q2 IAUCN10S7N021ATMA1 per leg,
2EDL8034F5BXUMA1 per leg, R113 / R101 / R102 EXB-18V103JX 10 k
arrays, R119 ERA-3AEB473V 47 k, K1 TLP175A, D22 ECMF02, U / V / W /
+ / - on AMT0440005TH0000G, RS485 in and out on JST BM02B, USB4115.
No coaxial cable and no coaxial connector: *coaxial* is where the
board sits behind the stator, not what it is wired with.

## The gate stage

TIM1 centre-aligned, ARR 2375 off 237.5 MHz = 50 kHz, RCR forced to 0
so the update fires at overflow and underflow both. CH5 / TRGO2
triggers the injected ADC group; `SYNC_TRIGGER_LEAD` is 15 ticks. PE15
is TIM1_BKIN, active low, pull-up, AOE off, OSSI and OSSR on:
`Board_PwmInit` starts with MOE clear and CCxE set, both FETs of every
leg held off in hardware, and clears BIF. Gate op 1 is the only thing
that sets MOE, always at zero duty. The gate pins are set to VERY_HIGH
speed: CubeMX's LOW made two of the three drivers run hot.

Dead time: the `.ioc` holds DTG 19 (80 ns) and only until the record
loads. The record's `deadtime_ns` is 30, which the timer rounds to
DTG 8 = 33.7 ns; the firmware's floor is 20 ns and DTG 127 = 535 ns is
the ceiling. The arithmetic 30 ns replaced: 59.4 ns of worst-corner
gate overlap plus the 2EDL8034's 6 ns TDMOFF, about 65 ns needed, and
80 ns fitted against that; the trim was made against the supply's
OCP. `deadtime_skew` shifts the two transitions of a leg by DTG counts
through a DTG rewrite each half-period and has not been on a scope.
The 2EDL8034 has no interlock of its own, so `rig.gates.arm()`
re-reads BDTR DTG and refuses a stage with no dead time.

The break bypass (op 6) clears BKE and a reset restores it; the gate
short probe reads the six pins with the stage disarmed and reports a
leg whose two gate pins sit on one node. A counted hold (op 2 with a
period count) is zeroed by the update ISR; the alternate (op 10) swaps
two compare triples every overflow; the dither (op 8) is a sigma-delta
on Q16.16 ticks. The drive owns the compares while it runs and commits
them at underflow. The thermal envelope drops MOE by the same path the
break uses.

## The STO chain

The drivers' supply is not the MCU's to switch. PA10 KEEPALIVE toggles
at 200 kHz into R72 330 Ω / C71 100 nF and a diode charge pump; the
chain releases the supply while the pump is fed and the pilot tone on
RS485 is present. Two ADC channels watch it: Cinj (PC1) is the
recovered pilot, Clevel (PB1) the integrator. `GateStage.interlock()`
asks for Cinj ≥ 3.0 V and Clevel ≥ 3.0 V; the unmodified bench board
reads 0.77 V and 0.06 V (2026-08-27), which is why bench sessions arm
with `ignore_interlock=True` and `bypass_sto=True`. The keepalive
latch holds a few hundred microseconds; gate op 0 reports the
keepalive count and the worst gap.

## Sensors on SPI

**BNO085 (U13), SPI2.** CS is PB12 as a GPIO held across header and
cargo - released between them the part restarts the message. Mode 3.
Prescaler 64 off the 190 MHz kernel = 2.97 MHz against the part's
3 MHz maximum; CubeMX's 32 gave 5.94 MBit/s and every read came back
FF FF FF FF. The advertisement is 276 bytes, so `IMU_BUF` is 320;
the chunk is 8 bytes; poll 1 kHz; wake wait 50 ms; reset held 1 ms
then 120 ms; quiet 60 ms. NRSTN and BOOTN are both active low and
CubeMX drove both low at boot - a part held in reset and strapped for
the bootloader. H_INTN is read before every transfer: without it the
advertisement turned up in one sample out of six. WAKE (PS0) is not
optional; a write with it left alone fails outright. SH-2 reports:
0x01 accelerometer Q8, 0x02 gyroscope Q9, 0x03 magnetometer Q4,
0x05 rotation vector Q14 (14 bytes), 0x08 game rotation vector
(12 bytes), 0xFB timebase (5 bytes).

**A1335 (U14), SPI4.** CS is PE4 as a GPIO. A 20-bit packet as four
5-bit words; at most 3 MHz, 1.56 MHz off the 100 MHz kernel. The
register map came from a reference implementation, not the datasheet
in this tree, so the polled register is settable (angle op 5). ANG's
low 12 bits x 360 / 4096 are degrees; TSEN is eighths of a kelvin;
FIELD is gauss, about 2 with no magnet and 300 to 1000 recommended.
Two frames per read: the first posts the address, the second clocks
the answer - asked TSEN, FIELD, TSEN in turn, one frame returned the
previous register every time. The R/W bit's direction was measured on
this board.

## Serial

USART3 on PB10 / PB11 to the debug probe's VCP at 115 200. USART2 and
UART5 each through a THVD1450 (U5, U6) at the record's `link_baud`;
RE is tied to GND so each hears itself, UART5's 120 Ω termination is
switched by PE14. CubeMX left the two RS485 UARTs at 9 216 000 and
nothing wrote the 115 200 everything reported, which is how the baud
joined the record at CAL_VERSION 9.

## Calibration record

`board_cal.c`. Compiled-in defaults are the schematic's arithmetic,
traced 2026-08-26; a rig overwrites them through `0x6E` device 3.
Parameter ids, units and defaults:

| id | Parameter | Default |
|---|---|---|
| 0 | vref_uv | 3 300 000 (U2 REF2033) |
| 1 | shunt_uohm | 3 500 (RU1 ǁ RU2) |
| 2 | amp_gain_ppm | 4 545 455 (THS4551, 1.5 k / 330) |
| 3 / 4 | bus_r_top_ohm / bus_r_bottom_ohm | 49 900 / 2 200 (R12 / R11) |
| 5 | ntc_r25_ohm | 10 000 |
| 6 | ntc_beta_mk | 3 380 000 |
| 7 | ntc_rfixed_ohm | 10 000 (R100) |
| 8 | ntc_t25_ck | 29 815 |
| 9 / 10 | r5_r_top_ohm / r5_r_bottom_ohm | 10 000 / 10 000 (R113) |
| 11 / 12 | vg_r_top_ohm / vg_r_bottom_ohm | 57 000 / 10 000 (R119 + R113) |
| 13 | deadtime_ns | 30 |
| 14 | deadtime_skew, DTG counts | 0 |
| 15 | motor_r_uohm | 50 000 |
| 16 / 17 | motor_ld_nh / motor_lq_nh | 20 000 / 25 000 |
| 18 | motor_lambda_uvs | 5 000 |
| 19 | motor_pole_pairs | 7 |
| 20 | drv_kp_mv_per_a | 100 |
| 21 | drv_ki_v_per_as | 250 |
| 22 / 23 | drv_l1_milli / drv_l2_milli | 100 / 100 000 |
| 24 | drv_inj_mv, 0 off | 0 |
| 25 | drv_inj_periods | 1 |
| 26 | drv_inj_phase_mrad | 0 |
| 27 | drv_eps_gain_ua_per_rad | 0 |
| 28 | drv_i_max_ma | 5 000 |
| 29 | drv_i_trip_ma | 100 000 (the rating) |
| 30 | drv_v_frac_ppm | 950 000 |
| 31 | drv_sign | 1 |
| 32 / 33 | drv_w_lo_mrad_s / drv_w_hi_mrad_s | 60 000 / 120 000 |
| 34 | drv_dt_step_ma | 1 000 |
| 35 .. 42 | drv_dt_mv, the dead-time table | 0 |
| 43 | drv_sigma_i_ua | 0 |
| 44 | drv_trigger_ticks, 0 none | 0 |
| 45 | link_baud | 115 200 |

Per channel, ten of them: `offset_raw` subtracted first, then
`gain_ppm`. The thermal envelope: ten SOA ceilings in centi-degrees
(driver U/V/W, phase U/V/W, mcu, regulators, afe at 125.00 - the FETs'
and the MCU's Tj max, the rest an estimate; board 105.00, an estimate
for the laminate) and a throttle at 85 %. The drive block (15 .. 44) is
placeholders in the same sense: the injection is off and the trip sits
at the rating until `tools/commission.py` measures and writes them.
CAL_VERSION history: 2 the supply senses, 4 the envelope, 5 the dead
time, 6 its skew, 7 per-leg nodes, 8 the drive, 9 the baud; a stored
record of another version is refused, not misread.

## Thermal network

`thermal/src/thermal.c`, ten nodes, fitted from a camera 2026-08-28
at a 20 C room. Board to ambient 8.33 K/W, board capacity 49 J/K
(tau 6.8 min). Per leg, driver and phase each 45.6 K/W to the board
(the camera saw one bridge zone, 15.2 K/W lumped; per leg is three
times that and no measurement says otherwise yet); mcu 22.5,
regulators 15.0, afe 41.5 K/W. Capacities: driver 0.35 / 3, phase
1.20 / 3, mcu 0.90, regulators 0.80, afe 0.30 J/K. The NTC sees the
drivers at 1.055 with a 6.00 K offset; die over node: mcu 27 K
(assumed), afe 0.5 K. Losses: Rds(on) 1.8 mΩ with alpha 7.8e-3 /K,
shunt 3.5 mΩ, hot-swap 5 mΩ (not measured), switching 1.20 W for
three legs at 50 % on 24.6 V, driver share 0.5, mcu 0.666 W, LDO
0.534 W, afe 0.13 W.

The camera states the fit came from (NTC, then the rises of the
bridge, the MCU, the regulators, the AFE):

| State | NTC C | bridge | MCU | regulators | AFE |
|---|---|---|---|---|---|
| passive | 30.0 | +15.0 | +8.0 | +1.0 | +1.0 |
| AFE on | 31.1 | +14.2 | +8.1 | - | +5.9 |
| traffic | 31.4 | +13.6 | +7.6 | - | +5.9 |
| switching | 40.0 | +17.3 | +20.0 | +10.1 | 0.0 |

NTC minus TSEN was -0.74 C idle and +10.94 C switching; the NTC
overstates the switching rise 2.48x. The observer steps every 100 ms,
samples the NTC every 30 s with a 500 ms settle (a sample is 0.42 s;
four samples 3 s apart spread 50 mK) and anchors at 0.05 Hz. The
camera saw one bridge zone, so it constrains the three legs together
and not one of them.

## Inverter constants

`host/coaxial/inverter.py`, the numbers the design arithmetic runs on:
FSW 50 kHz; T_DEAD 33.7 ns (DTG 8) and T_DEAD_SIM 65.4 ns, the
simulation's worst corner; T_MIN_PULSE 76 ns (18 ticks, TPW 40 ns +
DTG 8, 0.38 % duty); V_FRAC 0.95; RDS_ON 1.8 mΩ; Coss as CJO 15.6 nF,
M 0.45, VJ 0.7; L_LOOP 4 nH (0.25 nH/mm over the tight layout);
Q_RING 1.0 assumed; SHUNT 3.5 mΩ; AFE_V_PER_A 15.909 mV/A; AFE_DELAY
60 ns; AFE_A_PER_COUNT 3.2 mA at 16 bits; NOISE_A (0.35, 0.41) A rms
measured. The LTSpice models are the `electronic_simulations`
submodule.

## Geometry

`render/models/coaxial_63100.stl` is the CAD export the attitude view
draws from: a 100 mm disc with a 10 mm bore.
