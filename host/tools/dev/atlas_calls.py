"""What each function and class names of the code it imports: the atlas's ties from symbol to symbol.

A module's rows (atlas_tree._file) keep, a def or a class a row, the names its body uses that the module's
imports or its own defs bind (`named`) - read once a file, kept in the atlas's cache -; the build joins
them (`join`): an imported name to the symbol its module defines there, or to the module it is; a module's
own name to its own symbol; `self.x` in a method to its class's `x`; a class to its methods; a symbol to
its module's own code, what importing it runs - a script's guard not, run as a script alone (SCRIPT). `reach`
follows them from where a page starts: what it can touch, not all its imports could - a module imported for
one function does not bring its every user along, nor what it runs as a script.
"""
import ast

SCRIPT = '__main__'


def _script(statement):
    """Whether `statement` is a script's guard, `if __name__ == '__main__':`: what running the module runs."""
    test = statement.test if isinstance(statement, ast.If) else None
    if not (isinstance(test, ast.Compare) and len(test.ops) == 1 and isinstance(test.ops[0], ast.Eq)):
        return False
    sides = {('name', n.id) if isinstance(n, ast.Name) else ('text', n.value) if isinstance(n, ast.Constant)
             else None for n in (test.left, test.comparators[0])}
    return sides == {('name', '__name__'), ('text', SCRIPT)}


def named(tree):
    """[(symbol, names)] of a module's syntax `tree`: '' its own code at the top - what importing it runs -,
    SCRIPT what its guard runs as a script, a def or class its name, a method 'Class.method' - private ones
    too, a public one's way on -; the names each uses that an import of the module or a def of its own binds:
    'x', 'x.y' an attribute of one, '.y' a method's `self.y`."""
    bound = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            bound.update((a.asname or a.name).partition('.')[0] for a in node.names)
    defs = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
    bound.update(s.name for s in tree.body if isinstance(s, defs))
    top, script = [], []
    for s in tree.body:   # a guard's body run as a script, its else on import
        if _script(s):
            script, top = script + s.body, top + s.orelse
        elif not isinstance(s, defs):
            top.append(s)
    rows = [('', top)] + ([(SCRIPT, script)] if script else [])
    for s in tree.body:
        if isinstance(s, ast.ClassDef):   # its own statements; its methods rows of their own
            rows.append((s.name, [m for m in s.body if not isinstance(m, defs)] + s.bases + s.decorator_list))
            rows += [(s.name + '.' + m.name, [m]) for m in s.body if isinstance(m, defs)]
        elif isinstance(s, defs):
            rows.append((s.name, [s]))
    out = []
    for name, body in rows:
        nodes = [n for top in body for n in ast.walk(top)]
        own = {n.id for n in nodes if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Store)} | \
            {a.arg for n in nodes if isinstance(n, ast.arguments) for a in n.posonlyargs + n.args + n.kwonlyargs
             + [n.vararg, n.kwarg] if a} if name not in ('', SCRIPT) else set()   # a def's own hide the module's
        own -= {(a.asname or a.name).partition('.')[0] for n in nodes if isinstance(n, (ast.Import, ast.ImportFrom))
                for a in n.names}
        said = set()
        for node in nodes:
            if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
                base = node.value.id
                said.update(['%s.%s' % (base, node.attr)] if base in bound and base not in own
                            else ['.' + node.attr] if base == 'self' else [])
            elif isinstance(node, ast.Name) and node.id in bound and node.id not in own:
                said.add(node.id)
        out.append((name, sorted(said)))
    return out


def join(atlas, files, imports, calls):
    """`atlas.calls` {key: [keys]} - a key 'path' a module's own code, 'path::name' its symbol -: each file's
    rows (`files` {path: rows}, `imports` and `calls` their indices) joined, the names each symbol uses
    taken to what they name (`named`)."""
    known = {path[5:-3].replace('/', '.').replace('.__init__', ''): path for path, told in files.items()
             if told[calls] is not None and path.startswith('host/')}
    defined = {path: {name for name, _said in files[path][calls]} for path in known.values()}

    def symbol(module, name):   # a module's symbol `name`, else the module of that name
        path = known.get(module)
        if path is not None and name in defined[path]:
            return path + '::' + name
        return known.get(module + '.' + name)
    for dotted, path in known.items():
        package = dotted if path.endswith('__init__.py') else dotted.rpartition('.')[0]
        alias = {}
        for module, names, level in files[path][imports]:
            if module is None and level == 0:
                alias.update((a or n.partition('.')[0], (n if a else n.partition('.')[0], None)) for n, a in names)
                continue
            base = '.'.join(package.split('.')[:len(package.split('.')) - level + 1] + ([module] if module else [])) \
                if level else module
            alias.update((a or n, (base, n)) for n, a in names)
        for name, said in files[path][calls]:
            key, owner = path + ('::' + name if name else ''), name.partition('.')[0]
            out = ([path] if name else []) + [path + '::' + m for m in defined[path] if '.' not in name and m.startswith(name + '.')]
            for word in said:
                out.append(_named(word, path, owner, alias, known, defined, symbol))
            atlas.calls[key] = sorted({n for n in out if n is not None and n != key})


def _named(word, path, owner, alias, known, defined, symbol):
    """The key a name `word` (`named`) of the module at `path` names - in a method of class `owner` -, by the
    module's imports (`alias` {name: (module, its name there or None)}) and its own defs; None for none."""
    head, _, tail = word.partition('.')
    if not head:   # self.tail
        return path + '::' + owner + '.' + tail if owner + '.' + tail in defined[path] else None
    if head not in alias:   # its own def, or that def's attribute
        return path + '::' + head + '.' + tail if tail and head + '.' + tail in defined[path] else path + '::' + head
    module, what = alias[head]
    if not tail:
        return symbol(module, what) if what else known.get(module)
    inner = module + '.' + what if what else module
    if inner in known:   # an attribute of a module
        return symbol(inner, tail)
    return symbol(module, what + '.' + tail) or symbol(module, what) if what else None


def reach(atlas, starts):
    """Every key (`join`) that what `starts` [key] call can come to, them in: their names followed, each
    symbol's module's own code with it."""
    seen, todo = set(starts), list(starts)
    for key in todo:
        for nxt in atlas.calls.get(key, ()):
            if nxt not in seen:
                seen.add(nxt)
                todo.append(nxt)
    return seen
