#!/usr/bin/env python3
"""Build the firmware and flash it over SWD - the one path the local model
is allowed to take from source to silicon.

This exists so `run_command` never needs `cube-cmake` or
`STM32_Programmer_CLI` on its own allowlist. Either of those, called
directly, lets the model choose its own arguments - a different preset, a
different `-d`, a mode that asserts reset. This script takes none of that
from the caller: the preset name and the flash incantation are fixed here,
the same ones `cbuild`/`cflash` in env.ps1 use, and the only knobs exposed
are which of the two steps to run. `run_command`'s own allowlist already
gates on `python`, and every write still passes through the toolbox's
`--confirm` gate before this file is ever reached - see
`coaxial_ollama/sandbox.py` and `tools.py`.

Nothing here is new toolchain logic. It is env.ps1's bundle search
(`Get-NewestBundleBin`) and its `cbuild`/`cflash` functions, translated to
Python so a program that did not source that script can still find
arm-none-eabi-gcc, cmake, ninja and the programmer under
%LOCALAPPDATA%\\stm32cube\\bundles and the VS Code extension folder.

Exit code is 0 only if every requested step succeeded.
"""
import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

BUNDLE_ROOT = Path(os.environ.get('LOCALAPPDATA', '')) / 'stm32cube' / 'bundles'
VSCODE_EXT = Path(os.environ.get('USERPROFILE', '')) / '.vscode' / 'extensions'

WARNING_RE = re.compile(r'\bwarning\b', re.IGNORECASE)


def _newest(dir_path):
    """Newest version subdirectory of a bundle, '+st.N' suffix ignored."""
    if not dir_path.is_dir():
        return None
    def key(p):
        stem = p.name.split('+')[0]
        try:
            return tuple(int(x) for x in stem.split('.'))
        except ValueError:
            return (0,)
    candidates = [p for p in dir_path.iterdir() if p.is_dir()]
    return max(candidates, key=key, default=None)


def _bundle_bin(name):
    """One bundle's bin dir, or its version dir if it has no 'bin'."""
    newest = _newest(BUNDLE_ROOT / name)
    if newest is None:
        return None
    bin_dir = newest / 'bin'
    return bin_dir if bin_dir.is_dir() else newest


def _newest_ext(pattern):
    if not VSCODE_EXT.is_dir():
        return None
    matches = sorted(VSCODE_EXT.glob(pattern), reverse=True)
    return matches[0] if matches else None


def toolchain_path():
    """PATH entries env.ps1 adds before calling cube-cmake or the programmer.

    cube-cmake is not a standalone build tool: it shells out to `cube` (the
    bundle manager) and to the gcc/cmake/ninja bundles themselves, none of
    which are on PATH by default - that is the whole reason env.ps1 exists.
    Skip this and cube-cmake fails with 'cube' command is not available in
    current context, which says nothing about the firmware.
    """
    dirs = []
    for bundle in ('gnu-tools-for-stm32', 'cmake', 'ninja', 'programmer', 'gnu-gdb-for-stm32'):
        bin_dir = _bundle_bin(bundle)
        if bin_dir is not None:
            dirs.append(str(bin_dir))

    core = _newest_ext('stmicroelectronics.stm32cube-ide-core-*')
    if core is not None:
        cube_bin = core / 'resources' / 'binaries' / 'win32' / 'x86_64'
        if cube_bin.is_dir():
            dirs.append(str(cube_bin))

    build_ext = _newest_ext('stmicroelectronics.stm32cube-ide-build-cmake-*')
    if build_ext is not None:
        cube_cmake_dir = build_ext / 'resources' / 'cube-cmake' / 'win32' / 'x86_64'
        if cube_cmake_dir.is_dir():
            dirs.append(str(cube_cmake_dir))

    return os.pathsep.join(dirs + [os.environ.get('PATH', '')])


def find_programmer(path):
    """STM32_Programmer_CLI.exe, searched on the augmented PATH."""
    from shutil import which
    return which('STM32_Programmer_CLI', path=path)


def find_cube_cmake(path):
    """cube-cmake.exe, searched on the augmented PATH."""
    from shutil import which
    return which('cube-cmake', path=path)


#: Helpers cube-cmake starts and never stops. Measured: four of them, 121 MB,
#: still up from builds hours apart, and the VS Code extension keeps its own
#: alive on top of that.
CUBE_HELPERS = ('cube.exe', 'cube-cmsis-scanner.exe')


def cube_helpers():
    """PIDs of the helpers running now. Empty when tasklist is not there."""
    try:
        listed = subprocess.run(
            ['tasklist', '/fo', 'csv', '/nh'], capture_output=True, text=True,
            encoding='utf-8', errors='replace', timeout=20).stdout or ''
    except (OSError, subprocess.SubprocessError):
        return set()

    found = set()
    for line in listed.splitlines():
        parts = [f.strip('"') for f in line.split('","')]
        if len(parts) > 1 and parts[0].lower() in CUBE_HELPERS:
            try:
                found.add(int(parts[1]))
            except ValueError:
                pass
    return found


def reap(before):
    """Stop the helpers this run started, and only those.

    The extension's own `cube` respawns within a second of being killed and
    is not this script's to end - so what was already running is left alone.
    """
    for pid in cube_helpers() - before:
        try:
            subprocess.run(['taskkill', '/F', '/PID', str(pid)],
                           capture_output=True, timeout=20)
        except (OSError, subprocess.SubprocessError):
            pass


def run(argv, cwd, path):
    env = dict(os.environ, PATH=path)
    started = time.monotonic()
    before = cube_helpers()
    try:
        done = subprocess.run(argv, cwd=cwd, env=env, capture_output=True,
                              text=True, encoding='utf-8',
                              errors='replace', timeout=600)
    except subprocess.TimeoutExpired as exc:
        reap(before)
        return 1, (exc.stdout or '') + (exc.stderr or ''), time.monotonic() - started
    reap(before)
    return done.returncode, (done.stdout or '') + (done.stderr or ''), time.monotonic() - started


#: What the linker script gives each region, so the print says how much of it
#: is spent rather than a byte count nobody can size up.
REGIONS = {'FLASH': 2 * 1024 * 1024, 'DTCMRAM': 128 * 1024}


def footprint(elf, path):
    """(flash, dtcmram) bytes from the ELF's own section table.

    Flash holds every loaded section including `.data`'s initialisers;
    DTCMRAM holds `.data`, `.bss` and the heap/stack reservation. Written
    here because a number in a document is one nobody re-measures - TODO
    carried 134 748 B for as long as it took to grow by ten kilobytes.
    """
    size = shutil.which('arm-none-eabi-size', path=path)
    if size is None or not Path(elf).exists():
        return None

    done = subprocess.run([size, '-A', str(elf)], capture_output=True,
                          text=True, errors='replace')
    flash = ram = 0
    for line in done.stdout.splitlines():
        part = line.split()
        if len(part) != 3 or not part[1].isdigit():
            continue
        name, count, addr = part[0], int(part[1]), int(part[2])
        if 0x08000000 <= addr < 0x08200000:
            flash += count
        elif 0x20000000 <= addr < 0x20020000:
            ram += count
            if name == '.data':
                flash += count       # its initialiser is stored in flash
    return flash, ram


def build(preset, path):
    cube_cmake = find_cube_cmake(path)
    if cube_cmake is None:
        print('BUILD  FAIL  cube-cmake not found (VS Code extension '
              'stmicroelectronics.stm32cube-ide-build-cmake not installed - see setup.ps1)')
        return False
    code, output, elapsed = run([cube_cmake, '--build', '--preset', preset],
                                cwd=str(ROOT), path=path)
    warnings = len(WARNING_RE.findall(output))
    if code != 0:
        print('BUILD  FAIL  exit=%d  %.1fs' % (code, elapsed))
        print('\n'.join(output.splitlines()[-60:]))
        return False
    used = footprint(ROOT / 'build' / preset / 'coaxial_63100.elf',
                     path)
    room = ('  flash %d B (%.0f%%)  dtcmram %d B (%.0f%%)'
            % (used[0], 100.0 * used[0] / REGIONS['FLASH'],
               used[1], 100.0 * used[1] / REGIONS['DTCMRAM'])) if used else ''
    print('BUILD  ok  %.1fs  %d warning%s%s'
          % (elapsed, warnings, '' if warnings == 1 else 's', room))
    if warnings:
        for line in output.splitlines():
            if WARNING_RE.search(line):
                print('  ' + line.strip())
    return True


def flash(elf, path):
    elf = Path(elf)
    if not elf.exists():
        print('FLASH  FAIL  %s does not exist - build first' % elf)
        return False
    programmer = find_programmer(path)
    if programmer is None:
        print('FLASH  FAIL  STM32_Programmer_CLI not found (see setup.ps1)')
        return False
    # SWD, not JTAG: any connect on this probe that asserts NRST fails with
    # "Unable to get core ID". --start, not -hardRst, or the core is left
    # halted. Both measured on this bench - see the top-level CLAUDE.md.
    argv = [programmer, '-c', 'port=SWD', 'mode=UR', '-d', str(elf), '-v', '--start']
    code, output, elapsed = run(argv, cwd=str(ROOT), path=path)
    if code != 0:
        print('FLASH  FAIL  exit=%d  %.1fs' % (code, elapsed))
        print('\n'.join(output.splitlines()[-40:]))
        return False
    print('FLASH  ok  %.1fs  %s' % (elapsed, elf.name))
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--preset', default='Debug', choices=['Debug', 'Release'])
    parser.add_argument('--elf', default=None,
                        help='defaults to build/<preset>/coaxial_63100.elf')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--flash-only', action='store_true',
                        help='skip the build, flash whatever is already there')
    args = parser.parse_args(argv)

    elf = Path(args.elf) if args.elf else ROOT / 'build' / args.preset / 'coaxial_63100.elf'
    path = toolchain_path()

    ok = True
    if not args.flash_only:
        ok = build(args.preset, path)
    if ok and not args.build_only:
        ok = flash(elf, path)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
