"""The images their sources' or built: a pull leaves them behind, and the terminal opens on none.

    python tools/target/current.py          # said, and built where behind

The user, 2026-10-10: a pull on the other machine, a terminal opened, no old binaries run. Each
tree's own ninja dry run says what it would build (0.04 s); Debug is the emulator's
(tools/emu/emulator.py ELF), Release the bench's (`Coaxial63100.open()` loads the newest).
"""
import os
import re
import shutil
import subprocess
import sys

from tools.target.build_and_flash import APP, ROOT, build, toolchain_path

#: Built in this order where either is behind: Release last, the newest, the host's own build.
PRESETS = ('Debug', 'Release')


def pending(said):
    """What ninja's dry run `said` it would build: '' for nothing, else its steps and the first."""
    steps = re.findall(r'^\[\d+/(\d+)\] (.*)$', said, re.M)
    return '%s steps, the first %s' % (steps[0][0], steps[0][1]) if steps else ''


def behind(preset, path):
    """What `preset`'s tree would build on `path`'s ninja - 'not configured' without its tree -,
    '' where nothing, None where no ninja is found."""
    tree = ROOT / 'build' / preset
    if not (tree / 'build.ninja').exists():
        return 'not configured'
    ninja = shutil.which('ninja', path=path)
    if ninja is None:
        return None
    said = subprocess.run([ninja, '-C', str(tree), '-n'], capture_output=True, text=True,
                          encoding='utf-8', errors='replace').stdout
    return pending(said)


def ensure():
    """Every tree built where behind its sources, said; True where all are current."""
    path = toolchain_path()
    late = {preset: behind(preset, path) for preset in PRESETS}
    if not any(late.values()):
        return True
    for preset, why in late.items():
        if why:
            print('IMAGES  %s behind its sources (%s): building' % (preset, why))
    built = all([build(preset, path) for preset in PRESETS])
    # Release current and Debug relinked, Debug's would be the newest open() loads.
    debug, release = (ROOT / 'build' / preset / APP for preset in PRESETS)
    if built and release.exists() and debug.exists() \
            and release.stat().st_mtime < debug.stat().st_mtime:
        os.utime(release)
    if not built:
        print('IMAGES  the old ones stand: the emulator and the bench would run them')
    return built


if __name__ == '__main__':
    sys.exit(0 if ensure() else 1)
