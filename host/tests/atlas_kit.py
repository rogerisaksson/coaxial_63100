"""What the atlas views' suites share: the cells where one lane only passes across another."""


def _crossed(lanes):
    """The cells where one of `lanes` (atlas/mirror.py's) only passes across another (schematic.py breaks it)."""
    from terminal.views.atlas import schematic
    straight, per = {schematic.LEFT | schematic.RIGHT, schematic.UP | schematic.DOWN}, {}
    for k, (_a, _b, corners, *_inks) in enumerate(lanes):
        cells = schematic.walked(corners)
        for (pc, pr), (qc, qr) in zip(cells, cells[1:]):
            way = schematic.LEFT | schematic.RIGHT if qr == pr else schematic.UP | schematic.DOWN
            for cell in ((pc, pr), (qc, qr)):
                per.setdefault(cell, {}).setdefault(k, set()).add(way)
    return sum({way for ways in got.values() for way in ways} == straight and all(len(w) == 1 for w in got.values())
               for got in per.values())
