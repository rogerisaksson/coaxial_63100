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

Windows, Python 3.9+, and the ST toolchain. None of the ST tools are on the
system PATH — arm-gcc, cmake, ninja and STM32_Programmer_CLI are downloaded as
"bundles" by the STM32 VS Code extension, which is why the setup script leans on
that extension rather than on st.com: the direct downloads are behind a login
and a click-through licence, and a script cannot agree to one for you.

```powershell
powershell -ExecutionPolicy Bypass -File .\setup.ps1 -Check   # changes nothing
powershell -ExecutionPolicy Bypass -File .\setup.ps1          # installs what is missing
```

`-Check` prints what is present, what is absent and what each absent thing will
cost to fix. Run it first. Useful switches:

| Switch | Why |
|---|---|
| `-Yes` | do not ask before each install |
| `-SkipOllama` | a machine that only builds and flashes |
| `-Model TAG` | a different Ollama tag; the default is `gemma4:12b` |
| `-WingetToolchain` | cmake, ninja and Arm's gcc from winget instead of the VS Code bundles |
| `-AllowScripts` | set the CurrentUser execution policy so `. .\env.ps1` works in a plain shell |

The one step a script cannot finish is the bundle download: the extension fetches
those itself, on the first build, from inside VS Code. `setup.ps1` says so when
they are absent rather than leaving you to find out at flashing time.

### 2. Every shell

```powershell
. .\env.ps1
```

This puts the newest of each bundle on PATH **for that shell only** — nothing is
written to the system PATH or the registry, so a stale entry cannot outlive the
window it was made in. It also defines the five commands the project is driven
with:

```
bench    the model, the board and a prompt in one window
dbg      one question to the local model            (host/dbg.py)
board    the plain CLI, no model                    (python -m coaxial)
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
bench                       # daemon started, model preloaded, board checked, prompt open
bench -Ask "read the NTC and give me the temperature"
bench -Plain                # a bare ollama chat: no tools, no board
```

`bench.ps1` is preflight plus `host/dbg.py --repl`: the local model with the
board's tools, `/py` against a live session and `/sh` for a build, both of which
cost no tokens, and a token meter on every turn.

The model is **local and that is enforced, not assumed**. Ollama will happily
proxy a `:cloud` tag to somebody else's GPU, which on a bench means register
dumps and pin names leaving the building; a cloud tag or a non-loopback host
raises instead of quietly working, and `--allow-remote` is how you mean it on
purpose.

### 6. Prove the install

```powershell
cd host
python tests/test_ollama.py         # 93 checks, no board and no ollama needed
python tests/test_conformance.py    # 40 Modbus conformance checks, needs the board
python tests/test_mcp.py            # 31 MCP server checks
```

### Where to read next

| Read this | When |
|---|---|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | before touching any source layout |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | before changing anything on the wire |
| [docs/HARDWARE.md](docs/HARDWARE.md) | before interpreting any measurement |
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
