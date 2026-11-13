#!/usr/bin/env python3
"""One version of each distribution wherever host/ is installed: requirements.txt resolved as one lock.

    python -X utf8 tools/dev/pinned.py            # host/constraints.txt laid anew, every target resolved
    python -X utf8 tools/dev/pinned.py --check    # whether it pins every requirement at a version it allows

requirements.txt writes what host/ needs and why, each at a floor. Left at that, every place it was installed held
whatever was newest the day it was: numpy 2.4.6 on the desk, 2.5.4 in the Linux containers and in CI's environments,
those cached a runner image each on different days. The lock (host/constraints.txt) is one resolution of it and of
the package's own build (pyproject's build-system) for each place: this host's Python, and each Python of host.yml's
matrix in CI's runner image (tools/dev/linux.py) - each by its own pip, nothing installed (`--dry-run --report`). A
distribution every target holds is locked at the one version they agree on, the lock refused where two differ; one a
platform holds alone (pywin32, pexpect) as that platform resolved it - a constraint installs nothing where nothing
asks for it. Setup (setup/python.ps1), CI's inputs action and the runner's image install through it (pip -c).
"""
import argparse
import os
import subprocess
import sys
import tomllib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.dev import depends
from tools.dev.suites import ROOT

#: The requirements, the package whose build installs beside them, and the lock laid of both.
REQUIREMENTS, PYPROJECT, LOCK = ROOT / 'requirements.txt', ROOT / 'pyproject.toml', ROOT / 'constraints.txt'
#: A target's own resolution, run by its Python on requirements and specs as arguments: what pip would install on an
#: empty environment, `name==version` a line - its report read as UTF-8 whatever the host's console takes.
RESOLVE = ('import json, subprocess, sys\n'
           'out = subprocess.run([sys.executable, "-X", "utf8", "-m", "pip", "install", "--dry-run", "--quiet",\n'
           '                      "--ignore-installed", "--disable-pip-version-check", "--report", "-", "-r",\n'
           '                      *sys.argv[1:]], capture_output=True, encoding="utf-8")\n'
           'if out.returncode:\n'
           '    sys.exit(out.stderr[-600:])\n'
           'for item in json.loads(out.stdout)["install"]:\n'
           '    print("%s==%s" % (item["metadata"]["name"], item["metadata"]["version"]))\n')


def built():
    """What the package's own build asks (pyproject's build-system), as pip is told it."""
    with open(PYPROJECT, 'rb') as f:
        return list(tomllib.load(f)['build-system']['requires'])


def read(text):
    """{distribution: version} of a resolution's lines, each name as an index compares it."""
    return {depends._name(name): version
            for name, _, version in (line.strip().partition('==') for line in text.splitlines()) if version}


def here(build):
    """This host's resolution, by its own Python."""
    done = subprocess.run([sys.executable, '-X', 'utf8', '-c', RESOLVE, str(REQUIREMENTS), *build],
                          capture_output=True, encoding='utf-8', check=False)
    if done.returncode:
        raise SystemExit('pinned: this host resolved nothing: %s' % done.stderr[-600:])
    return read(done.stdout)


def linux(build):
    """{'linux 3.12': its resolution, ...}: each Python of host.yml's matrix in CI's runner image (tools/dev/linux.py),
    the image built as the gate's is, of the tree's requirements and pyproject."""
    from tools.dev import linux as runner
    info, client, found = runner.workflow(), runner.engine(), {}
    for minor in info['pythons']:
        probe = client.containers.run(runner.built(info, minor), ['python', '-X', 'utf8', '-c', RESOLVE,
                                                                  '/tmp/inputs/requirements.txt', *build], detach=True)
        try:
            code = probe.wait()['StatusCode']
            said = probe.logs(stdout=True, stderr=bool(code)).decode('utf-8', 'replace')
        finally:
            probe.remove(force=True)
        if code:
            raise SystemExit('pinned: Python %s in the runner resolved nothing: %s' % (minor, said[-600:]))
        found['linux %s' % minor] = read(said)
    return found


def merged(targets):
    """{distribution: version}, each at the one version every target holding it resolved; the lock refused, the
    disagreement said, where two differ."""
    seen: dict = {}
    for target, got in targets.items():
        for dist, version in got.items():
            seen.setdefault(dist, {})[target] = version
    differ = {dist: held for dist, held in seen.items() if len(set(held.values())) > 1}
    if differ:
        raise SystemExit('pinned: the targets differ - ' + '; '.join(
            '%s %s' % (dist, ', '.join('%s %s' % pair for pair in held.items())) for dist, held in sorted(differ.items())))
    return {dist: next(iter(held.values())) for dist, held in sorted(seen.items())}


def laid(pins, targets):
    """The lock's text: what it is and how it was laid, then a distribution a line."""
    return ''.join(['# One version of each distribution host/ installs, not by hand: tools/dev/pinned.py resolved\n',
                    '# requirements.txt and the build on %s. Installed through it: pip -c.\n' % ', '.join(targets)]
                   + ['%s==%s\n' % pair for pair in sorted(pins.items())])


def check(requirements=REQUIREMENTS, lock=LOCK, build=None):
    """The requirements and build specs the lock leaves out or pins at a version they do not allow: `name spec` each."""
    pins = {depends._name(dist): spec.lstrip('=') for dist, _extras, spec in depends.asked(lock)}
    asked = depends.asked(requirements) + [(m.group(1), (), m.group(3).strip())
                                           for m in map(depends.ASKED.match, built() if build is None else build) if m]
    return ['%s%s' % (dist, spec) for dist, _extras, spec in asked
            if depends._name(dist) not in pins or not depends.met(pins[depends._name(dist)], spec)]


def main(argv=None):
    """The lock laid anew from every target's resolution; --check: 1 where it misses or outgrows a requirement."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--check', action='store_true', help='whether the lock pins every requirement at a version it allows')
    args = parser.parse_args(argv)
    if args.check:
        left = check()
        print('pinned: ' + ('%d not pinned as asked: %s' % (len(left), ', '.join(left)) if left else 'every requirement pinned'))
        return 1 if left else 0
    build = built()
    targets = {'%s %d.%d' % (sys.platform, *sys.version_info[:2]): here(build), **linux(build)}
    pins = merged(targets)
    with open(LOCK, 'w', encoding='utf-8', newline='\n') as f:
        f.write(laid(pins, targets))
    print('pinned: %d distributions, one version each on %s (numpy %s) - %s'
          % (len(pins), ', '.join(targets), pins.get('numpy'), LOCK))
    return 0


if __name__ == '__main__':
    sys.exit(main())
