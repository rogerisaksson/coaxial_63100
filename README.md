# Coaxial 63100

Instrumentation firmware and Python host library for a stator-mounted ("coaxial") BLDC inverter. Rated 63 V / 100 A (peak SOA survival, not continuous). Driven by an STM32H753VIT6 at 475 MHz.

This is currently a telemetry pipeline. Motor control (PWM, commutation, current loops) is unconfigured.

## Getting Started

Follow these steps to set up the environment, build the firmware, and launch the host telemetry and LLM pipeline.

### 1. Environment Setup

The provisioning script handles dependencies via Winget, Python, and ST's bundle manager:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check     # Check status without changes
powershell -ExecutionPolicy Bypass -File .\setup.ps1           # Interactive install
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Yes      # Unattended install
```

Load the per-shell toolchain and command aliases (does not modify the system PATH):

```powershell
. .\env.ps1
```

### 2. Build & Flash

```powershell
cbuild              # Compile firmware (zero warnings expected)
cflash              # Flash via SWD and start the core
```

### 3. Interact with the Board

```powershell
board all           # Dump all ADC channels, NTC temperature, and link stats
```

### 4. Launch the AI-Assisted Telemetry Prompt

```powershell
board_prompt        # Starts the local model daemon, checks board health, and opens the REPL
```

---

## Environment & Toolchain

* **`setup.ps1`**: Automates dependency installation (Winget, Python, Ollama, ST toolchains). Bypasses ST's login wall by hijacking the VS Code bundle manager.
* **`env.ps1`**: Scopes paths strictly per-shell to avoid registry rot. Injects core aliases (`cbuild`, `cflash`, `board`, `board_prompt`, `dbg`, `cubemx`).

## Build & Deploy

* **`cbuild`**: Compiles the firmware. Zero warnings is a strict requirement, not an aspiration.
* **`cflash`**: Flashes via SWD. Must terminate with `--start`. Asserting `-hardRst` on this probe halts the core and silently kills serial comms.

## Telemetry & The AFE Trap

* **Link**: USART3 multiplexes a text console and Modbus RTU. Reachable via debug VCP or RS485.
* **The AFE Trap**: The ADC voltage reference is powered by the AFE switch. If the AFE is off, channels read exact mid-scale, generating a phantom 25.00 °C on the NTC. A plausible number, but not a measurement.
* **Dumb Slave**: The board reports raw codes. It knows nothing of calibration or limits. Pass/fail thresholding is strictly the jurisdiction of the host test executive.

## LLM Orchestrator

* **`board_prompt`**: Initializes the local model daemon, preloads weights, and binds hardware tools.
* **VRAM Strictness**: Dynamically selects the largest model that fits *entirely* in VRAM, reserving overhead for the OS. Spilling to CPU RAM incurs a 5x speed penalty and is treated as a fallback, never a target.
* **Data Sovereignty**: Cloud tags are explicitly blocked to prevent unreleased hardware telemetry from leaking to external servers.
* **Execution, Not Judgement**: The LLM may script builds, flash hardware, and pull telemetry, but final test verdicts are strictly calculated by Python (`plan.Limit`).

## Validation & Licensing

* **Verification**: `.un_tests.ps1` is the interface. It runs a change-sized subset by default (~25 % of every check, chosen by the local model from the diff); `-AutomaticMedium`, `-AutomaticHigh` and `-All` widen it, `-Only NAMES` narrows it to named tests.
* **License**: CC BY-NC-SA 4.0. Open for hobbyists. Commercial deployment requires a paid license. Contact [erogisa@gmail.com](mailto:erogisa@gmail.com) for enterprise terms.
