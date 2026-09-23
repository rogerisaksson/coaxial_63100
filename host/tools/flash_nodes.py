#!/usr/bin/env python3
"""The master at the bench: every blank node on a segment takes its image
and its record from the store (docs/BOOT.md).

    python tools/flash_nodes.py --store DIR --simulated --bus LL
    python tools/flash_nodes.py --store DIR --port COM4 --bus LL
    python tools/flash_nodes.py --store DIR --place <uid> LL 2 coaxial_63100

The store is a directory: nodes.json (uid -> bus, position, type),
images/<type>.bin, records/<bus>/<position>.record. Unit id is position
down the limb; the last position on the segment closes the termination.
`--place` writes one row of the table and stops. The stand-in's bus is
four blank nodes with uids 10.., 11.., 12.., 13..; place them first.
"""
import argparse
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.boot import BOOT_BAUD, Master                  # noqa: E402

TYPES = {'coaxial_63100': 1, 'coaxial_63020': 2}


class Store:
    """The directory, read as the master's three maps for one bus."""

    def __init__(self, root):
        self.root = Path(root)
        self.nodes = self.root / 'nodes.json'

    def table(self, bus):
        rows = json.loads(self.nodes.read_text()) if self.nodes.exists() else {}
        on_bus = {uid: row for uid, row in rows.items() if row['bus'] == bus}
        last = max((row['position'] for row in on_bus.values()), default=0)
        return {uid: {'unit': row['position'], 'position': row['position'],
                      'type': TYPES[row['type']], 'terminate': row['position'] == last}
                for uid, row in on_bus.items()}

    def images(self, table):
        names = {v: k for k, v in TYPES.items()}
        return {t: (self.root / 'images' / (names[t] + '.bin')).read_bytes()
                for t in {row['type'] for row in table.values()}}

    def records(self, bus, table):
        found = {}
        for row in table.values():
            path = self.root / 'records' / bus / ('%d.record' % row['position'])
            if path.exists():
                found[row['unit']] = path.read_bytes()
        return found

    def place(self, uid, bus, position, type_):
        rows = json.loads(self.nodes.read_text()) if self.nodes.exists() else {}
        rows[uid] = {'bus': bus, 'position': int(position), 'type': type_}
        self.root.mkdir(parents=True, exist_ok=True)
        self.nodes.write_text(json.dumps(rows, indent=1, sort_keys=True) + '\n')


def segment_of(args):
    if args.simulated:
        from coaxial.simulated.boot import SimulatedBoot, SimulatedSegment
        return SimulatedSegment(SimulatedBoot(uid=bytes([0x10 + k] + list(range(11))))
                                for k in range(4))
    from coaxial.boot import TransportSegment
    from coaxial.transport import Transport
    return TransportSegment(Transport(args.port, BOOT_BAUD))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--store', required=True)
    parser.add_argument('--bus', default='LL')
    parser.add_argument('--port')
    parser.add_argument('--simulated', action='store_true')
    parser.add_argument('--session', type=int, default=1)
    parser.add_argument('--place', nargs=4, metavar=('UID', 'BUS', 'POSITION', 'TYPE'))
    args = parser.parse_args(argv)
    store = Store(args.store)
    if args.place:
        store.place(*args.place)
        print('placed %s at %s %s as %s' % tuple(args.place))
        return 0
    table = store.table(args.bus)
    master = Master(segment_of(args), table, store.images(table),
                    store.records(args.bus, table), session=args.session)
    states = master.run()
    for unit, state in sorted(states.items()):
        print('unit %d position %d: %s, %s, uid %s' % (
            unit, state['position'], state['state'],
            'valid' if state['valid'] else 'INVALID', state['uid']))
    for uid in master.unknown:
        print('uid %s answered and is not in the table - place it' % uid)
    return 1 if master.unknown else 0


if __name__ == '__main__':
    sys.exit(main())
