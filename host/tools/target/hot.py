"""Where the image the board runs jumps, calls and waits, by interrupt: ranked.

    python tools/target/hot.py                      # every handler's reach, Release
    python tools/target/hot.py --irq ADC3_IRQHandler --top 40

Each function of the ELF's disassembly counted - its instructions, conditional branches (b<cc>,
cbz, cbnz, it), calls (bl, blx), divides and roots, memory accesses - and each handler's reach
followed through its calls: what an interrupt can run, statically, and of it what runs outside
ITCM (fetched through the cache from D2 SRAM or flash, a miss a stall). The user, 2026-10-10:
the hot paths' jumps, interrupts and stalls ranked, each cut or named unavoidable.
"""
import argparse
import re
import shutil
import subprocess
import sys
from typing import Any

from tools.target.build_and_flash import APP, ROOT, toolchain_path

#: ITCM's span: zero-wait fetch, the sample path's (STM32H753xx_FLASH.ld's .itcm).
ITCM = (0x00000000, 0x00010000)

FUNCTION = re.compile(r'^([0-9a-f]{8}) <([^>]+)>:$')
INSTRUCTION = re.compile(r'^\s*([0-9a-f]+):\s+(\S+)\s*(.*)$')
BRANCH = re.compile(r'^(b(?:eq|ne|cs|hs|cc|lo|mi|pl|vs|vc|hi|ls|ge|lt|gt|le)|cbn?z|it[te]{0,3})'
                    r'(?:\.[nw])?$')
CALL = re.compile(r'^blx?(?:\.[nw])?$')
#: An unconditional branch: into another function a tail call.
JUMP = re.compile(r'^b(?:\.[nw])?$')
#: A long branch's veneer: the linker's, to a function out of a bl's reach.
VENEER = re.compile(r'^__(.+)_veneer$')
DIVIDE = re.compile(r'^(?:[su]div|vdiv|vsqrt)')
MEMORY = re.compile(r'^(?:v?ldr|v?str|ldm|stm|v?push|v?pop|ldrex|strex)')
TARGET = re.compile(r'<([^>+]+)>')

#: What a function's row holds, in order.
FIELDS = ('instructions', 'branches', 'calls', 'divides', 'memory')


def functions(elf, path):
    """{name: {'at': address, FIELDS..., 'callees': set}} off `elf`'s disassembly."""
    objdump = shutil.which('arm-none-eabi-objdump', path=path)
    if objdump is None:
        raise SystemExit('arm-none-eabi-objdump not found (setup.ps1)')
    said = subprocess.run([objdump, '-d', '--no-show-raw-insn', str(elf)], capture_output=True,
                          text=True, encoding='utf-8', errors='replace').stdout
    out: dict[str, dict[str, Any]] = {}
    row: dict[str, Any] | None = None
    for line in said.splitlines():
        found = FUNCTION.match(line)
        if found:
            name = found.group(2)
            row = out[name] = {'at': int(found.group(1), 16), 'callees': set(),
                                         **{field: 0 for field in FIELDS}}
            continue
        found = INSTRUCTION.match(line)
        if row is None or not found:
            continue
        op, args = found.group(2), found.group(3)
        row['instructions'] += 1
        row['branches'] += bool(BRANCH.match(op))
        row['divides'] += bool(DIVIDE.match(op))
        row['memory'] += bool(MEMORY.match(op))
        target = TARGET.search(args)
        if CALL.match(op):
            row['calls'] += 1
        if target and (CALL.match(op) or JUMP.match(op)) and target.group(1) != name:
            row['callees'].add(target.group(1))
    for name, row in out.items():
        veneer = VENEER.match(name)
        if veneer:
            row['callees'].add(veneer.group(1))
    return out


def reach(table, root):
    """Every function `root` can run through its calls, itself first."""
    seen, stack = [], [root]
    while stack:
        name = stack.pop()
        if name in seen or name not in table:
            continue
        seen.append(name)
        stack.extend(sorted(table[name]['callees'], reverse=True))
    return seen


def itcm(row):
    """Whether a function is fetched from ITCM."""
    return ITCM[0] <= row['at'] < ITCM[1]


def report(table, handler, top):
    """A handler's reach: its sums, then its functions by conditional branches."""
    names = reach(table, handler)
    rows = [table[n] for n in names]
    outside = [n for n in names if not itcm(table[n])]
    sums = {f: sum(r[f] for r in rows) for f in FIELDS}
    print('%s: %d functions, %d instructions, %d conditional branches, %d calls, %d divides, '
          '%d memory; outside ITCM %d (%s)' % (
              handler, len(names), sums['instructions'], sums['branches'], sums['calls'],
              sums['divides'], sums['memory'], len(outside), ', '.join(outside[:8])
              + (' ..' if len(outside) > 8 else '')))
    for name in sorted(names, key=lambda n: -table[n]['branches'])[:top]:
        r = table[name]
        print('  %-40s %5d ins %4d br %3d calls %2d div %4d mem  %s' % (
            name[:40], r['instructions'], r['branches'], r['calls'], r['divides'], r['memory'],
            'itcm' if itcm(r) else 'OUT'))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--preset', default='Release', choices=['Debug', 'Release'])
    parser.add_argument('--irq', action='append', help='a handler; every one if none')
    parser.add_argument('--top', type=int, default=12)
    args = parser.parse_args(argv)
    table = functions(ROOT / 'build' / args.preset / APP, toolchain_path())
    handlers = args.irq or sorted(n for n in table
                                  if n.endswith('_IRQHandler') or n == 'SysTick_Handler')
    for handler in handlers:
        report(table, handler, args.top)
    return 0


if __name__ == '__main__':
    sys.exit(main())
