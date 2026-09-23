#!/usr/bin/env python3
"""Build the firmware and flash it over SWD: the local model's one path from source to silicon.

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

from find_board import _text          # noqa: E402 - tools/ is the script dir
from shutil import which
from contextlib import suppress

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


#: The STM32 VS Code extensions that carry binaries of their own, and
#: where under each: `cube` in the core, `cube-cmake` in the build one.
EXTENSION_BINS = (
    ('stmicroelectronics.stm32cube-ide-core-*',
     ('resources', 'binaries', 'win32', 'x86_64')),
    ('stmicroelectronics.stm32cube-ide-build-cmake-*',
     ('resources', 'cube-cmake', 'win32', 'x86_64')),
)


def _extension_bins():
    """The extensions' bin directories, those that are there."""
    for pattern, parts in EXTENSION_BINS:
        ext = _newest_ext(pattern)
        folder = ext.joinpath(*parts) if ext is not None else None
        if folder is not None and folder.is_dir():
            yield str(folder)


def toolchain_path():
    """PATH entries env.ps1 adds before calling cube-cmake or the
    programmer.
    """
    dirs = []
    for bundle in ('gnu-tools-for-stm32', 'cmake', 'ninja', 'programmer', 'gnu-gdb-for-stm32'):
        bin_dir = _bundle_bin(bundle)
        if bin_dir is not None:
            dirs.append(str(bin_dir))

    dirs.extend(_extension_bins())
    return os.pathsep.join(dirs + [os.environ.get('PATH', '')])


def find_programmer(path):
    """STM32_Programmer_CLI.exe, searched on the augmented PATH."""
    return which('STM32_Programmer_CLI', path=path)


def find_cube_cmake(path):
    """cube-cmake.exe, searched on the augmented PATH."""
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
            with suppress(ValueError):
                found.add(int(parts[1]))
    return found


def reap(before):
    """Stop the helpers this run started, and only those."""
    for pid in cube_helpers() - before:
        with suppress(OSError, subprocess.SubprocessError):
            subprocess.run(['taskkill', '/F', '/PID', str(pid)],
                           capture_output=True, timeout=20)


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
        return 1, _text(exc.stdout) + _text(exc.stderr), time.monotonic() - started
    reap(before)
    return done.returncode, (done.stdout or '') + (done.stderr or ''), time.monotonic() - started


#: The two images and what their linker scripts give each region, so the
#: print says how much of it is spent rather than a byte count nobody can
#: size up. The bootloader is sector 0; the application's flash begins
#: behind it and ends before the record's sector (docs/BOOT.md). Code that
#: runs from ITCM is stored in flash and counted there.
IMAGES = {
    'coaxial_63100.elf': {'FLASH': (0x08020000, 1792 * 1024),
                          'DTCMRAM': (0x20000000, 128 * 1024),
                          'ITCMRAM': (0x00000000, 64 * 1024)},
    'coaxial_63100_boot.elf': {'FLASH': (0x08000000, 128 * 1024),
                               'DTCMRAM': (0x20000000, 128 * 1024),
                               'ITCMRAM': (0x00000000, 64 * 1024)},
}
APP, BOOT = tuple(IMAGES)
LABELS = {APP: 'app', BOOT: 'boot'}


def _region_of(regions, addr):
    """The linker region an address falls in, or None."""
    return next((name for name, (base, size) in regions.items()
                 if base <= addr < base + size), None)


def footprint(elf, path):
    """(flash, dtcmram) bytes from the ELF's own section table."""
    size = shutil.which('arm-none-eabi-size', path=path)
    if size is None or not Path(elf).exists():
        return None

    regions = IMAGES[Path(elf).name]
    done = subprocess.run([size, '-A', str(elf)], capture_output=True,
                          text=True, errors='replace')
    flash = ram = 0
    for line in done.stdout.splitlines():
        part = line.split()
        if len(part) != 3 or not part[1].isdigit():
            continue
        name, count, addr = part[0], int(part[1]), int(part[2])
        region = _region_of(regions, addr)
        if region == 'FLASH':
            flash += count
        elif region == 'DTCMRAM':
            ram += count
            flash += count if name == '.data' else 0   # its initialiser is in flash
        elif region == 'ITCMRAM' and name in ('.itcm', '.text'):
            flash += count      # code copied out of flash by the startup
    return flash, ram


def room(elf, path):
    """One image's footprint against its regions, or nothing to say."""
    used = footprint(elf, path)
    if used is None:
        return ''
    regions = IMAGES[Path(elf).name]
    return ('  %s flash %d B (%.0f%%)  dtcmram %d B (%.0f%%)'
            % (LABELS[Path(elf).name],
               used[0], 100.0 * used[0] / regions['FLASH'][1],
               used[1], 100.0 * used[1] / regions['DTCMRAM'][1]))


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
    print('BUILD  ok  %.1fs  %d warning%s%s'
          % (elapsed, warnings, '' if warnings == 1 else 's',
             ''.join(room(ROOT / 'build' / preset / name, path) for name in IMAGES)))
    for line in filter(WARNING_RE.search, output.splitlines()):
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
    # "Unable to get core ID".
    argv = [programmer, '-c', 'port=SWD', 'mode=UR', '-d', str(elf), '-v', '--start']
    code, output, elapsed = run(argv, cwd=str(ROOT), path=path)
    if code != 0:
        print('FLASH  FAIL  exit=%d  %.1fs' % (code, elapsed))
        print('\n'.join(output.splitlines()[-40:]))
        return False
    print('FLASH  ok  %.1fs  %s' % (elapsed, elf.name))
    if elf.name == APP:
        # --start runs the image now; a reset goes through sector 0, the
        # bootloader's, and finds the application only if one is there.
        print('       the image sits at 0x%08X behind the bootloader sector - '
              'a reset reaches it through the bootloader (--boot flashes it)'
              % IMAGES[APP]['FLASH'][0])
    return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--preset', default='Debug', choices=['Debug', 'Release'])
    parser.add_argument('--elf', default=None,
                        help='defaults to build/<preset>/coaxial_63100.elf')
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--flash-only', action='store_true',
                        help='skip the build, flash whatever is already there')
    parser.add_argument('--boot', action='store_true',
                        help='flash the bootloader first - sector 0, the image '
                             'a reset goes through (docs/BOOT.md)')
    args = parser.parse_args(argv)

    elf = Path(args.elf) if args.elf else ROOT / 'build' / args.preset / APP
    path = toolchain_path()

    ok = True
    if not args.flash_only:
        ok = build(args.preset, path)
    if ok and not args.build_only and args.boot:
        ok = flash(ROOT / 'build' / args.preset / BOOT, path)
    if ok and not args.build_only:
        ok = flash(elf, path)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
