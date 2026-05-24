# pr-description

Generate a pull request description suited for STM32 firmware development using the STM32CubeMX + STM32 VS Code Extension toolchain (arm-none-eabi-gcc, ST HAL/LL drivers, OpenOCD/ST-Link).

Analyze the git diff and commit history since branching from main, then produce a PR description covering the sections below. Omit any section that is clearly not applicable to the change.

---

## Title
One line, under 70 characters. Prefix with the affected module or peripheral in brackets, e.g. `[FDCAN] Fix DLC validation on extended frames` or `[CubeMX] Regen after clock tree update`.

## Summary
2–4 bullet points: what changed, why, and any constraints that shaped the approach (silicon errata, HAL version, timing budgets, toolchain limits, etc.).

## CubeMX & Code Generation
- Note if the `.ioc` file was modified and briefly describe what changed (clock tree, peripheral config, pin assignment, middleware, FreeRTOS heap/stack settings).
- Confirm that generated files (`Core/Src/`, `Core/Inc/`, `Drivers/`, `Middlewares/`) were regenerated cleanly and that no hand-edits inside `/* USER CODE BEGIN */` / `/* USER CODE END */` blocks were lost.
- State the CubeMX version used if it changed.
- Flag any generated code that was intentionally NOT regenerated and why.

## MISRA C Compliance
- Applicable edition: MISRA C:2012 (default for STM32 HAL-based projects unless stated otherwise).
- List any new deviations introduced in hand-written code, with rule number, a one-line rationale, and whether a formal deviation record is needed.
- Note that ST HAL/LL drivers carry their own pre-approved deviation list — do not re-raise those unless the project extends or overrides HAL code.
- Confirm no previously documented deviations were removed or widened without justification.
- Static analysis result: Cppcheck, PC-lint Plus, Polyspace, or similar (state tool and version).

## Memory & Resource Impact
- Flash delta (`.text` + `.rodata`), readable from the `.map` file or `arm-none-eabi-size` output.
- RAM delta (`.bss` + `.data`), and heap usage if `pvPortMalloc` / `malloc` was introduced (flag if project policy bans dynamic allocation).
- FreeRTOS task stack changes: task name, old/new stack size in words, and `uxTaskGetStackHighWaterMark` result if available.
- DMA buffer alignment requirements added (note if `__attribute__((aligned(32)))` or MPU region was needed for cache coherency on Cortex-M7 targets).

## Hardware & Peripheral Impact
- STM32 peripherals added, reconfigured, or removed (TIM, USART/UART/LPUART, SPI, I2C, FDCAN/CAN, USB, ADC, DAC, DMA, BDMA, MDMA, RNG, CRC, etc.).
- Clock tree changes: source, PLL config, peripheral bus frequencies (APB1/APB2/APB3/AHB).
- NVIC changes: IRQ enable/disable, priority group, preemption and sub-priorities. Flag any priority inversion risk with FreeRTOS (`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`).
- DMA stream/channel/request reassignments.
- Pin mux changes (GPIO alternate function, speed, pull, open-drain).
- Timing-sensitive paths: ISR execution budget, bit-bang sequences, watchdog window (`IWDG`/`WWDG` reload).
- Silicon errata workarounds applied or removed — cite the errata sheet revision.
- Board or PCB revision constraints (if change is revision-specific, say so).

## Safety & Reliability
- Functional Safety relevance: IEC 61508 SIL or ISO 26262 ASIL level of the changed component, if applicable.
- Error handling: `HAL_StatusTypeDef` / `HAL_ErrorCallback` paths added, changed, or removed.
- Watchdog coverage: does the change affect `HAL_IWDG_Refresh` / `HAL_WWDG_Refresh` call sites?
- Safe-state / fault reaction logic affected.
- Calling context assumptions: task vs. ISR, whether `__disable_irq()` / FreeRTOS critical section is required, reentrance.

## Build Verification
- Toolchain: `arm-none-eabi-gcc` version.
- Build configuration tested: Debug / Release / (custom).
- Compiler flags relevant to the change (optimization level, `-fstack-usage`, LTO, etc.).
- Zero new warnings at `-Wall -Wextra -Wpedantic` (or list and justify any suppressed).

## Test Plan
Checklist of verifications performed or required before merge:
- [ ] CubeMX regeneration checked — no hand-edits lost.
- [ ] Clean build, zero new warnings (Debug and Release configs).
- [ ] Static analysis clean (or deviations documented above).
- [ ] Flashed and booted on target via ST-Link (STM32 VS Code Extension / OpenOCD).
- [ ] Affected peripheral verified on hardware — state board revision and MCU part number/stepping.
- [ ] FreeRTOS task high-water marks checked (no stack overflow).
- [ ] Regression tested on connected subsystems / communication buses.
- [ ] Timing verified with oscilloscope / logic analyser (if relevant).
- [ ] Reviewed against MISRA C:2012 checklist.
- [ ] Code review by a second engineer familiar with the subsystem.

## Reviewer Notes
Links to relevant ST documents (reference manual section, application note, errata sheet), known limitations, follow-up tickets, or any tricky sections to focus on during review.
