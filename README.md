# coaxial_63100

Control firmware and a Python host library for a **coaxial BLDC inverter** — a
three-phase drive whose PCB sits coaxially behind the rotor of an outrunner. The
name is the rating: **63 V and 100 A**, the current being instantaneous within
the FETs' safe operating area. STM32H753VIT6 at 475 MHz, one UART carrying
either a text console or binary Modbus RTU.

What is in this repository today is **instrumentation, not a motor controller**.
No timer is configured: there is no PWM, no commutation and no current loop. It
reads the three differential phase channels, the DC link, an NTC and the board's
own registers, and exposes all of it over the link. Nothing here has been
exercised near 63 V or 100 A.

## Getting started

### 1. What the machine needs

Windows and winget. Everything else `setup.ps1` installs:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # changes nothing
powershell -ExecutionPolicy Bypass -File .\setup.ps1          # asks, then installs
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Yes -AllowScripts   # the lot, unattended
```

Run without switches and the first question is which kind of run this is:

```
  Unattended, or one question per step?
    y  install everything that is missing without asking again
    n  ask before each install  (default)
  unattended? [y/N]
```

`y` there is the same as passing `-Yes`, and it is asked once at the top rather
than discovered a dozen prompts in. The default is `n`. `-Check` never asks —
there is nothing to consent to when nothing installs — and a run with no console
to ask on (piped, scheduled) takes silence as `n` and puts the rest on the todo
list rather than dying half-installed.

| What | From |
|---|---|
| Python, git, VS Code | winget |
| pyserial, PyYAML, mcp, anyio, pytest | `host/requirements.txt`, via pip |
| arm-none-eabi-gcc, gdb, cmake, ninja | STM32 bundles, via `cube.exe` |
| STM32_Programmer_CLI | same |
| STM32CubeMX | same — the `stm32cubemx-application` bundle, or st.com's installer if you would rather |
| ST-Link gdbserver, server and USB driver | same |
| cube-cmake | the STM32 VS Code extension |
| VS Code extensions: the STM32 pack, cpptools, python, ollama | `code --install-extension`, mirroring [.vscode/extensions.json](.vscode/extensions.json) |
| ollama, and a model this machine can actually run | winget or `ollama.com/install.ps1`, then `ollama pull` |
| STM32Cube FW_H7 | st.com — the one thing still behind a login, and only CubeMX needs it |

None of the ST tools land on the system PATH: they install as "bundles" under
`%LOCALAPPDATA%\stm32cube\bundles`, which is why plain `cmake` and
`STM32_Programmer_CLI` are nowhere to be found until `env.ps1` has run.

The ST half used to be the manual half — st.com is behind a login and a
click-through licence, so the honest instruction was "open VS Code once and let
its bundle manager download the toolchain". It is not any more. The STM32 VS
Code extension ships `cube.exe`, and `cube bundle install --yes NAME` pulls the
same bundles from developer.st.com with no account and no browser. That is what
the script drives, so a fresh machine needs no human in the middle of the
toolchain install.

Three things still cannot be helped. A winget install only reaches the PATH of
shells opened *after* it, so on a bare machine the first run installs Python and
VS Code and asks to be run once more — the second run finishes. Installing the
ST-Link USB driver needs administrator rights, so it comes with an elevation
prompt of its own. And the STM32Cube FW_H7 package is still behind an st.com
login.

That last one matters less than it sounds. **The build does not need it** —
`Drivers/` is in this repository, so gcc has its HAL and CMSIS either way. What
needs it is CubeMX: opening the `.ioc` against a repository without
`STM32Cube_FW_H7_V1.13.0` in it means CubeMX offers to fetch its own, and
regenerating against a different version is a different `Core/` than the one in
git. The script reads the required version out of the `.ioc`, looks in CubeMX's
repository, and if it is absent offers three ways in:

```powershell
# 1. you already have it - a share, a stick, another bench. No browser, no account.
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -FirmwarePackage D:\STM32Cube_FW_H7_V1.13.0.zip

# 2. CubeMX's own package manager: cubemx, then Help > Manage embedded software packages
# 3. the download page, opened for you when asked
```

What it will not do is scrape the download out of st.com. The URLs that used to
serve these unauthenticated answer 404 now, and the GitHub mirror keeps its
drivers as submodules — a zipball of the tag has none of the sources in it.

| Switch | Why |
|---|---|
| `-Check` | report only: what is present, what is absent, what each absent thing costs |
| `-Yes` | answer the opening question from the command line: install everything, ask nothing |
| `-SkipOllama` | a machine that only builds and flashes |
| `-SkipCubeMX` | skip STM32CubeMX — 308 MB down, 835 MB on disk, and only the `.ioc` needs it |
| `-SkipDriver` | leave the ST-Link USB driver alone (no elevation prompt) |
| `-FirmwarePackage PATH` | install a `STM32Cube_FW_H7_*.zip`, or an unpacked copy, into the CubeMX repository |
| `-SkipFirmware` | do not look for FW_H7 at all — a machine that only builds and flashes |
| `-CubeMXInstaller PATH` | run an STM32CubeMX installer downloaded from st.com, instead of taking the 308 MB bundle |
| `-Repository PATH` | where CubeMX keeps its packages, if yours is not the default |
| `-Model TAG` | overrule the automatic choice with a tag of your own |
| `-Reserve N` (board_prompt) | VRAM in GB to keep for the desktop; raise it if the screens stutter. `COAXIAL_VRAM_RESERVE_GB` sets it per machine |
| `-Prefer speed\|capability` | `speed` fits the card whole; `capability` allows a bigger model to spill onto the CPU |
| `-WingetToolchain` | cmake, ninja and Arm's gcc from winget instead of the ST bundles |
| `-AllowScripts` | set the CurrentUser execution policy so `. .\env.ps1` works in a plain shell |

Re-running is free: every step checks before it installs, and a finished machine
prints `nothing outstanding` and changes nothing.

### 2. Every shell

```powershell
. .\env.ps1
```

This puts the newest of each bundle on PATH **for that shell only** — nothing is
written to the system PATH or the registry, so a stale entry cannot outlive the
window it was made in. It also defines the six commands the project is driven
with:

```
board_prompt  the model, the board and a prompt in one window
dbg      one question to the local model            (host/dbg.py)
board    the plain CLI, no model                    (python -m coaxial)
cubemx   open coaxial_63100.ioc in STM32CubeMX
cbuild   build the firmware, zero warnings expected
cflash   flash over SWD and start the core
```

### 3. Build and flash

```powershell
cbuild                      # cube-cmake --build --preset Debug
cflash                      # SWD, then --start
```

Zero warnings is the standard, not an aspiration. Flashing is SWD on purpose:
any connect that asserts NRST on this probe fails with `Unable to get core ID`,
and a programmer invocation must end in `--start` rather than `-hardRst` or the
core is left halted with no clue as to why.

### 4. Talk to the board

```powershell
board all                   # every channel, the NTC, the link stats, the version
```

Before believing anything analog, know this: **the AFE switch powers the ADC
reference**, not just the signal path. With it off every channel reads exact
mid-scale and the NTC reports exactly 25.00 °C — a plausible number that is not
a measurement.

The board is a dumb slave. It reports raw codes and sensed values; there are no
limits and no expected values anywhere in the firmware or in these tests,
because pass/fail against real thresholds belongs to a test executive on the
line, beside the calibrated instruments.

### 5. The model in the loop

```powershell
board_prompt                # daemon started, model preloaded, board checked, prompt open
board_prompt -Ask "read the NTC and give me the temperature"
board_prompt -Plain         # a bare ollama chat: no tools, no board
```

`board_prompt.ps1` is preflight plus `host/dbg.py --repl`: the local model with
the board's tools, `/py` against a live session and `/sh` for a build, both of
which cost no tokens, and a token meter on every turn. It measures the machine,
picks the model that fits it, **pulls that model if it is not here yet**, loads
it before the prompt opens and says which COM port answered — so "the model is
not installed" is never the reason a question goes unanswered.

The model can read this repository's documents while it works — `docs()` for the
index, `docs(find='25.00')` to search, `docs(doc='HARDWARE', section=...)` for
the text. That is not decoration: the documents are what stop a reading being
misinterpreted, and a model that cannot reach them answers from memory instead.
[docs/MODELS.md](docs/MODELS.md) is the chapter about the model itself.

Structured output is not done with Ollama's json mode here, and that is a
decision rather than an omission. `format='json'` constrains the *content* of a
reply — the one part of it this bench does not parse. Every number that reaches
a verdict arrives as an argument to the `report` tool, against a JSON Schema the
daemon enforces, and `plan.Limit` judges it in Python. A model told to answer in
JSON tends to describe a tool call in its content instead of making one, so json
mode would compete with that rather than help it. It is available for callers
outside the runner — `dbg.py --format json`, usually with `-t none` — and off
everywhere else.

**The tag is chosen from the machine, not from whoever wrote this file.** A
bench PC is whatever was on the shelf, and one hardcoded model means a laptop
crawls or a workstation idles. `setup.ps1` measures cores, RAM and the size of
the graphics card and pulls the largest tools-capable model that fits the card
*whole*, keeping a quarter of the VRAM back so the desktop still has somewhere
to live. Ask it yourself:

```powershell
cd host
python -m coaxial_ollama.capability                    # what this machine gets, and why
python -m coaxial_ollama.capability --prefer capability
dbg -m auto "what is the board temperature?"           # same choice, from the prompt
```

Three things there were measured on this bench rather than assumed, and two of
them contradict the usual advice — RTX 4080 SUPER 16 GB, Threadripper 3970X,
`gemma4:12b` Q4_K_M at 48 layers:

| | VRAM | tok/s |
|---|---|---|
| all 48 layers on the GPU | 7.8 GB | **64.3** |
| 24 layers (the "hybrid") | 4.3 GB | 12.7 |
| CPU only | 0 GB | 6.7 |

So a split model costs about **five times** the speed for half the VRAM back,
which makes hybrid the fallback when nothing fits — not a target. `num_gpu`
needs no Modelfile either; it is an ordinary entry in `options` on a normal
call, which beats a Modelfile because a Modelfile is a second tag to keep in
step. And raising `num_thread` on 64 threads bought nothing and cost a little
(6.4 → 5.7 tok/s at 64), because decode is bandwidth-bound, not core-bound.

Only tools-capable tags are candidates. Everything here reaches the board
through tool calls, and a tag without them describes a measurement instead of
taking one.

The model is **local and that is enforced, not assumed**. Ollama will happily
proxy a `:cloud` tag to somebody else's GPU, which on a bench means register
dumps and pin names leaving the building; a cloud tag or a non-loopback host
raises instead of quietly working, and `--allow-remote` is how you mean it on
purpose.

### 6. Prove the install

```powershell
cd host
python tests/test_ollama.py         # 187 checks, no board and no ollama needed
python tests/test_conformance.py    # 40 Modbus conformance checks, needs the board
python tests/test_mcp.py            # 36 MCP server checks
```

### Where to read next

| Read this | When |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | before touching any source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | before changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | before interpreting any measurement |
| [docs/MODELS.md](docs/MODELS.md) | before changing anything about the local model |
| [docs/FINDINGS.md](docs/FINDINGS.md) | **before investigating anything** — it records what has already been ruled out |

---

## License

The **coaxial_63100** project is licensed under the **Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International (CC BY-NC-SA 4.0)** license.

### Terms for Hobbyists and Individuals
* 🟢 **Free for personal use:** You are free to run, copy, and modify the code for personal or hobby projects.
* 🔴 **No commercial use:** You may **not** use this code for commercial purposes, within a company, or for financial gain.
* 🔄 **ShareAlike:** If you modify the code, you must distribute your contributions under this exact same license.
* 👤 **Attribution:** You must give appropriate credit and link back to this repository.

Read the full legal text here: [CC BY-NC-SA 4.0 License Terms](https://creativecommons.org)

---

## Commercial Use & Licensing

Companies and commercial entities wishing to use **coaxial_63100** in their business, production, or commercial products must acquire a separate commercial license.

To discuss pricing and commercial licensing options, please contact me:
* 📧 **Email:** [erogisa@gmail.com](mailto:erogisa@gmail.com)
* 💬 **GitHub:** Open an [Issue](https://github.com/rogerisaksson/coaxial_63100/issues) or contact me via my GitHub profile.
