"""How many checks each suite and each test group last reported."""
import io
import json
import os

PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.counts.json')


def load():
    try:
        with io.open(PATH, encoding='utf-8') as handle:
            got = json.load(handle)
    except Exception:                                         # noqa: BLE001
        return {}
    return got if isinstance(got, dict) else {}


def record(section, sizes):
    """Merge sizes into one section and return the whole file."""
    # A SUITE THAT RAN NOTHING MEASURED NOTHING.
    sizes = {name: n for name, n in sizes.items() if n}
    got = load()
    have = got.get(section)
    got[section] = dict(have if isinstance(have, dict) else {}, **sizes)
    try:
        with io.open(PATH, 'w', encoding='utf-8') as handle:
            json.dump(got, handle, indent=1, sort_keys=True)
    except Exception:                                         # noqa: BLE001
        pass
    return got


def missing(section, names):
    """(skipped_checks, how_many_have_never_been_measured)."""
    known = load().get(section) or {}
    return (sum(known.get(n, 0) for n in names),
            sum(1 for n in names if n not in known))
