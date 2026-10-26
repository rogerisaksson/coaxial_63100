"""What she can be told, scanned from the sources themselves - no register written by hand.

    rows = says.catalog()          # [{op, kind, command, said, ..}], JSON-able, for the local LLM or MCP
    python -m machine.says         # the catalog served as JSON

A row's kind and its source: 'errand' walks to a thing (`route.docks`, what affords it
`furniture.GOES`); 'stunt' plays a suite patch where she stands (`maneuver.PLAYED`, the furniture
her eyes see its seat); 'gait' a stream of her texts; 'behavior' any other text of her gym
(`sprung.entries`, its phases); 'take' a mocap take said at its beats (`machine.takes`); 'word'
the loop's own command vocabulary (`sprung.HEARD`); 'program' the page's own (`tour.HouseTour`).
A row's `said` is its source's own words: a text's comment above its (defbehavior ..), an
affordance's docstring, a stunt's patch. A row's `command` what hears it: a gait her keys drive
its twist (`se2.NOMINAL_TWIST`), the rest a say. A say out of reach is refused by name at the say,
the refusal text the answer.
"""
import functools
import glob
import json
import os
import re


@functools.lru_cache(maxsize=1)
def _comments():
    """{name: the ;; comment over its (defbehavior name ..) or (defpatch name ..)} of her texts and
    the suite's (machine/*.lisp, machine/ortho_dsp/*.lisp)."""
    out, here = {}, os.path.dirname(__file__)
    for path in sorted(glob.glob(os.path.join(here, '*.lisp')) + glob.glob(os.path.join(here, 'ortho_dsp', '*.lisp'))):
        lines = open(path, encoding='utf8').read().splitlines()
        for k, line in enumerate(lines):
            m = re.match(r'\((?:defbehavior|defpatch)\s+([\w-]+)', line)
            if m is None:
                continue
            j, said = k - 1, []
            while j >= 0 and lines[j].startswith(';;'):
                said.insert(0, lines[j][2:].strip())
                j -= 1
            if said:
                out.setdefault(m.group(1), ' '.join(said))
    return out


def catalog(texts=None, suite=None):
    """The rows of everything she can be told, scanned live from her texts, the suite, the docks,
    the takes and the loop's words; `texts` and `suite` hers where given, else their files'."""
    import numpy as np
    from machine import furniture, maneuver, route, se2, sprung, takes, tour
    rows, said = [], _comments()
    docks = route.docks()
    for sc, spec in docks.items():
        affords, _itself, _faces = furniture.GOES.get(sc, furniture._NONE)
        doc = (affords.__doc__ or '').strip().split('\n')[0]
        rows.append({'op': sc, 'kind': 'errand', 'command': {'say': sc}, 'optional': {'at': 'the (x, z) of the thing meant'},
                     'affords': affords.__name__, 'heading_rad': spec[0], 'source': 'route.docks',
                     'said': 'walks to the nearest thing among hers that affords it - %s' % doc})
    for name, (patch, under, stages) in maneuver.PLAYED.items():
        if name in docks:
            continue
        rows.append({'op': name, 'kind': 'stunt', 'command': {'say': name}, 'patch': patch,
                     'rises_from': under, 'phases': sorted(set(stages.values())), 'source': 'maneuver.PLAYED',
                     'said': said.get(patch, name.replace('_', ' '))})
    told = {r['op'] for r in rows}
    for label, _wire, names in sprung.entries(texts=texts, suite=suite):
        if label in told:
            continue
        told.add(label)
        gait = len(names) == 1 and names[0] == label
        twist = se2.NOMINAL_TWIST.get(str(label))
        rows.append({'op': label, 'kind': 'gait' if gait else 'behavior',
                     'command': {'say': label} if twist is None else {'twist': twist},
                     **({} if gait else {'phases': list(names)}), 'source': 'sprung.entries',
                     'said': said.get(str(label), str(label).replace('_', ' '))})
    for name in sorted(set(k[:-2] for k in np.load(takes.FILE).files if k.endswith('_t'))):
        rows.append({'op': name, 'kind': 'take', 'command': {'take': name}, 'source': 'machine.takes',
                     'said': 'the mocap take %s' % name.replace('_', ' ')})
    rows.append({'op': 'tour', 'kind': 'program', 'command': {'tour': True}, 'source': 'tour.HouseTour',
                 'said': (tour.__doc__ or 'the house tour').strip().split('\n')[0]})
    rows.append({'op': 'heard', 'kind': 'word', 'command': {w: '...' for w in sprung.HEARD}, 'source': 'sprung.HEARD',
                 'said': 'the loop\'s command words'})
    return rows


def main():
    print(json.dumps(catalog(), indent=1))


if __name__ == '__main__':
    main()
