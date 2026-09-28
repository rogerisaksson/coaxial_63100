#!/usr/bin/env python3
"""A suite past the token budget split by subject: its tests into files, its helpers where used.

    python tools/dev/split_suite.py plan.json       # {"suite", "kit", "kit_brief", "parts"}

`parts` maps each new file to its brief and its tests, in order; a part may name the suite's
own file to keep it. A helper one part uses - itself or through another helper - moves into
that part; the rest, and anything run at import, go to the kit. Each file imports what it
uses and runs its tests through tools.dev.focus.chosen; a part's `main` is the plain one - a
Report, the ROSTER, the tally - unless the plan gives its own (tools.dev.token_budget).
"""
import ast
import json
import os
import sys

TESTS = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__)))), 'tests')

MAIN = '''

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
'''


def _roster(names):
    lines, line = [], 'ROSTER = ('
    for k, name in enumerate(names):
        piece = name + (',' if k + 1 < len(names) else ',)' if len(names) == 1 else ')')
        if len(line) + 1 + len(piece) > 96:
            lines.append(line.rstrip())
            line = '          ' + piece
        else:
            line += ('' if line.endswith('(') else ' ') + piece
    return '\n'.join(lines + [line])


def _used(node):
    """The bare names a node reads."""
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


def _bound(node):
    """The names a top-level statement binds."""
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return {node.name}
    if isinstance(node, (ast.Import, ast.ImportFrom)):
        return {(a.asname or a.name).split('.')[0] for a in node.names}
    if isinstance(node, (ast.Assign, ast.AnnAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        return {n.id for t in targets for n in ast.walk(t) if isinstance(n, ast.Name)}
    return set()


def _source(lines, node, previous_end):
    """A statement's text with the comments and blank lines just above it."""
    start = node.lineno - 1
    if getattr(node, 'decorator_list', None):
        start = min(d.lineno for d in node.decorator_list) - 1
    top = start
    while top > previous_end and (not lines[top - 1].strip()
                                  or lines[top - 1].lstrip().startswith('#')):
        top -= 1
    while top < start and not lines[top].strip():
        top += 1
    return '\n'.join(lines[top:node.end_lineno])


def _parse(path):
    """The suite's tests {name: (node, text)}, its imports and its helpers [(node, text)]."""
    with open(path, encoding='utf-8') as f:
        text = f.read()
    tree, lines = ast.parse(text), text.splitlines()
    docstring = ast.get_docstring(tree) is not None
    previous = tree.body[0].end_lineno if docstring else 0
    tests, imports, helpers = {}, [], []
    for node in tree.body[1:] if docstring else tree.body:
        src = _source(lines, node, previous)
        previous = node.end_lineno
        if isinstance(node, ast.FunctionDef) and node.name.startswith('test_'):
            tests[node.name] = (node, src)
        elif isinstance(node, (ast.Import, ast.ImportFrom)):
            imports.append((node, src))
        elif not (isinstance(node, ast.If) or _bound(node) & {'main', 'ROSTER'}):
            helpers.append((node, src))
    return tests, imports, helpers


def split(plan):
    """{file: text} for the kit and the parts out of `plan`."""
    tests, imports, helpers = _parse(os.path.join(TESTS, plan['suite']))
    placed = [t for part in plan['parts'].values() for t in part['tests']]
    assert sorted(placed) == sorted(tests), (set(tests) - set(placed), set(placed) - set(tests))
    by_name = {name: k for k, (node, _s) in enumerate(helpers) for name in _bound(node)}

    def needs(uses):
        """The helpers `uses` reaches, through each other."""
        got, todo = set(), [by_name[n] for n in uses if n in by_name]
        while todo:
            k = todo.pop()
            if k not in got:
                got.add(k)
                todo += [by_name[n] for n in _used(helpers[k][0]) if n in by_name]
        return got
    reach = {name: needs(set().union(*[_used(tests[t][0]) for t in part['tests']]))
             for name, part in plan['parts'].items()}
    owners = {k: [p for p in reach if k in reach[p]] for k in range(len(helpers))}
    kit = [k for k in range(len(helpers)) if len(owners[k]) != 1 or not _bound(helpers[k][0])]
    kit_names = set().union(*[_bound(helpers[k][0]) for k in kit]) if kit else set()

    def imports_for(uses):
        return [s for n, s in imports if _bound(n) & uses]
    kit_uses = set().union(*[_used(helpers[k][0]) for k in kit]) if kit else set()
    out = {plan['kit'] + '.py': '"""%s"""\n%s\n\n\n%s\n' % (
        plan['kit_brief'], '\n'.join(imports_for(kit_uses)),
        '\n\n\n'.join(helpers[k][1] for k in kit))}
    for name, part in plan['parts'].items():
        own = [helpers[k] for k in sorted(reach[name]) if k not in kit]
        body = own + [tests[t] for t in part['tests']]
        uses = set().union(*[_used(n) for n, _s in body]) | {'sys'}
        strays = uses & set(tests) - set(part['tests'])
        assert not strays, (name, strays)
        head = imports_for(uses)
        if not any('sys' in _bound(n) for n, s in imports if s in head):
            head = ['import sys'] + head
        head += ['', 'from tools.dev.focus import chosen',
                 'from %s import %s' % (plan['kit'], ', '.join(sorted((uses | {'Report'})
                                                                      & kit_names)))]
        out[name] = '"""%s"""\n%s\n\n\n%s\n\n\n%s%s' % (
            part['brief'], '\n'.join(head), '\n\n\n'.join(s for _n, s in body),
            _roster(part['tests']), part.get('main', MAIN))
    return out


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    with open(argv[0], encoding='utf-8') as f:
        plan = json.load(f)
    made = split(plan)
    if plan['suite'] not in made:
        os.remove(os.path.join(TESTS, plan['suite']))
        print('removed', plan['suite'])
    for name, text in made.items():
        with open(os.path.join(TESTS, name), 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
        print('%-32s %5.1f k tokens' % (name, len(text) / 4e3))
    return 0


if __name__ == '__main__':
    sys.exit(main())
