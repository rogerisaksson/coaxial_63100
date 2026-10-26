"""What she can be told, scanned from the sources themselves - no register written by hand.

    rows = says.catalog()          # [{op, kind, command, ..}], JSON-able, for the local LLM or MCP
    python -m machine.says         # the catalog served as JSON

A row's kind and its source: 'errand' walks to a thing (`route.docks`, what affords it
`furniture.GOES`); 'stunt' plays a suite patch where she stands (`maneuver.PLAYED`, the furniture
her eyes see its seat); 'gait' a stream of her texts; 'behavior' any other text of her gym
(`sprung.entries`, its phases); 'take' a mocap take said at its beats (`machine.takes`); 'word'
the loop's own command vocabulary (`sprung.HEARD`). A say out of reach is refused by name at the
say, the refusal text the answer.
"""
import json


def catalog(texts=None, suite=None):
    """The rows of everything she can be told, scanned live from her texts, the suite, the docks,
    the takes and the loop's words; `texts` and `suite` hers where given, else their files'."""
    import numpy as np
    from machine import furniture, maneuver, route, sprung, takes
    rows = []
    docks = route.docks()
    for sc, spec in docks.items():
        affords, _itself, _faces = furniture.GOES.get(sc, furniture._NONE)
        rows.append({'op': sc, 'kind': 'errand', 'command': {'say': sc, 'at': '(x, z), optional'},
                     'affords': getattr(affords, '__name__', sc), 'heading_rad': spec[0], 'source': 'route.docks'})
    for name, (patch, under, stages) in maneuver.PLAYED.items():
        if name in docks:
            continue
        rows.append({'op': name, 'kind': 'stunt', 'command': {'say': name}, 'patch': patch,
                     'rises_from': under, 'phases': sorted(set(stages.values())), 'source': 'maneuver.PLAYED'})
    said = {r['op'] for r in rows}
    for label, _wire, names in sprung.entries(texts=texts, suite=suite):
        if label in said:
            continue
        said.add(label)
        gait = len(names) == 1 and names[0] == label
        rows.append({'op': label, 'kind': 'gait' if gait else 'behavior',
                     'command': {'say': label}, **({} if gait else {'phases': list(names)}), 'source': 'sprung.entries'})
    for name in sorted(set(k[:-2] for k in np.load(takes.FILE).files if k.endswith('_t'))):
        rows.append({'op': name, 'kind': 'take', 'command': {'take': name}, 'source': 'machine.takes'})
    rows.append({'op': 'heard', 'kind': 'word', 'command': {w: '...' for w in sprung.HEARD}, 'source': 'sprung.HEARD'})
    return rows


def main():
    print(json.dumps(catalog(), indent=1))


if __name__ == '__main__':
    main()
