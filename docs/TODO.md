# TODO

State of the QA and hardening pass started 2026-08-26.

| | Baseline, before | Now |
|---|---|---|
| `run_tests.ps1 -All` | 1540 passed, 0 failed, 9/9, 413 s | **1572 passed, 0 failed, 9/9, 414 s** |
| Debug build | 0 warnings | 0 warnings |
| FLASH | 92 684 B (4.42 %) | 97 708 B (4.66 %) |
| DTCMRAM | 13 488 B (10.29 %) | 13 616 B (10.39 %) |

32 checks added, no wall time. The 128 bytes of RAM are the calibration record.

## Done and tested

**Calibration lives on the board.** `0x6E` device 3: nine scalars and one
offset/gain pair per ADC channel, in flash, surviving a reset.

| What | Where | Checked by |
|---|---|---|
| The record, its defaults, CRC-16 and flash persistence | `Board/Src/board_cal.c` | `test_conformance.py`, 15 checks, and a live save → hardware reset → read-back |
| `zero` and `span`, which need a reading | `Board/Src/board_adc.c` | same, plus the refusals below |
| The wire | `Comms/Src/cmd_cal.c` | `test_conformance.py`, from a master sharing no code with the library |
| The host side | `host/coaxial/calibration.py` | structure suite |
| Phase current, DC link and NTC scaling as arithmetic | `host/coaxial/scaling.py` `ShuntParams` | `test_simulated.py` group `scaling`, 11 checks - DC link and NTC had **no test at all** before |

Verified on the board, 2026-08-26:

* record reads, edits are volatile, `defaults` restores, `load` discards
* save → `stored` flips → hardware reset → record intact, values intact
* `span` refused on the NTC (logarithmic) and on a channel with no unit
* a refused edit leaves the record byte-for-byte unchanged — **this caught a
  real bug**: the first version validated *after* assigning and rolled back by
  reloading flash, which does nothing on a board whose record has never been
  saved. It left `vref_uv` at zero. Fixed to validate first
* every phase now reports amperes off the board's own channel map, and
  Phase V reads −52 A with nothing connected — the known 0.85 V op-amp fault,
  now in the unit that makes it obvious

Traced values, all from `electronics/`, **none measured**:

| | |
|---|---|
| DC link | R12 49.9 kΩ / R11 2.2 kΩ → 78.15 V full scale |
| NTC | R100 10.0 kΩ / NCU18XH103**D60**RB → B = 3380 K, ratiometric |
| Phase | RU1‖RU2 3.5 mΩ (two 7 mΩ WSHM2818), THS4551 1.5k/330 = 4.5455 V/V → 15.909 mV/A, 207.4 A full scale |
| Reference | U2 REF2033 drives `+3V3_ref` **and** `+1V65_bias` |

Docs corrected: invariant 7 replaced (its premise is gone), the VREF line, the
`.ioc` peripheral list (15 IPs, still no timer), the suite sizes.

## Next

| | Why |
|---|---|
| **Span every channel against an instrument** | nothing on this board has been measured. Until then every ampere and every volt is arithmetic off a PDF |
| **What the ADA4891 quad does** | the shunt is confirmed at 7 mΩ ×2, which bounds the chain at 9.43 V/V for 100 A to be representable at all. The THS4551's 4.5455 fits; the ×18.5 and ×10 an Altium net dump appeared to put on the quad do not. Buffer, level shift, or the `PH_CURR` protection tap - unresolved, and it changes no reported ampere |
| MCP tool for the record | the firmware and the library have it; `coaxial_mcp` does not, so the local model cannot reach it |


## Open, from the −1 inventory

| | Why it matters |
|---|---|
| **No CI anywhere** | 151 commits in 5 days, the gate is `run_tests.ps1 -All` run by hand |
| Datasheets for 2 of 45 active parts | missing: 2EDL8034 gate driver, IAUCN10S7N021 FETs, THS4551, ADA4891, REF2033, TPS3840, LM5069, TLV3492, THVD1450 |
| No layout or gerbers in `electronics/` | schematic PDF and BOM only |

## Open, from reading the code

| | |
|---|---|
| [board_imu.c:174](../Board/Src/board_imu.c#L174), [dev_uart.c:320](../Comms/Src/dev_uart.c#L320) | deadline as `HAL_GetTick() + ms`, compared with `>` / `<=`. Three other waits in the same files use the wrap-safe `(now - start) < ms`. Breaks at 49.7 days |
| [board_io.c:27](../Board/Src/board_io.c#L27) | `s_digital` omits PA1/PA2/PA3 and PC8/PC12/PD2, so `testrig_port_write` masks USART3 and JTAG but not either RS485 port |
| `BOARD_BURST_MAX_US` | 5 s against a 9.04 s CYCCNT wrap at 475 MHz. Correct today, undocumented coupling |
| No watchdog; fault handlers are bare `while(1)` | U4 TPS3840 supervises the rail in hardware; nothing supervises the firmware |
| Calibration has no wear levelling | one sector, erased whole on every save. Fine for a bench, not for a rig that saves per unit under test |
