"""Centered Braille progress screen shown before the menu while 3D and host C11 cores compile."""
import math
import threading
import time

from rich import box
from rich.align import Align
from rich.console import Group
from rich.layout import Layout
from rich.panel import Panel
from rich.text import Text

from coaxial.draw.cross_section import BRAILLE, BRAILLE_BITS, DOTS_X, LED_PITCH
from terminal.compiling import SPIN, SPIN_HZ
from terminal.ui.chrome import Chrome
from terminal.ui.console import Keys
from terminal.ui.screen import FPS_CAP, paced
from terminal.ui.stage import band_of, curtain, stage
from terminal.ui.taskbar import footer

#: Asymptotic creep parameters and dot rows lit in the upper and lower Braille bar rows.
CREEP_TAU_S, CREEP_CAP = 1.8, 0.92
WARMUP_ROWS = ((1, 2, 3), (0, 1, 2))


def braille_bar(share: float, cells: int) -> Text:
    """Two-row segmented Braille progress bar `cells` wide for `share` in `[0, 1]`."""
    cells = max(8, int(cells))
    wide = cells * DOTS_X
    clamped = max(0.0, min(1.0, float(share)))
    filled = int(clamped * wide + 0.5)
    tip_col = (min(filled, wide - 1) // DOTS_X) if 0 < filled < wide else -1
    out = Text(no_wrap=True)
    for r_idx, ys in enumerate(WARMUP_ROWS):
        if r_idx:
            out.append('\n')
        for col in range(cells):
            x0 = col * DOTS_X
            bits = 0
            if x0 < filled:
                for dx in range(DOTS_X):
                    x = x0 + dx
                    if x < filled and (clamped >= 1.0 or x == filled - 1
                                       or x % LED_PITCH != LED_PITCH - 1):
                        for y in ys:
                            bits |= BRAILLE_BITS[dx][y]
                if col == tip_col and not bits:
                    for y in ys:
                        bits |= BRAILLE_BITS[0][y]
                style = 'name' if col == tip_col else 'value'
            else:
                for y in ys:
                    bits |= BRAILLE_BITS[0][y]
                style = 'frame.hud'
            out.append(chr(BRAILLE + bits), style=style)
    return out


def remaining_text(share: float) -> str:
    """Percentage left to compile as uppercase text, `100% REMAINING` down to `0% REMAINING`."""
    clamped = max(0.0, min(1.0, float(share)))
    return '%d%% REMAINING' % max(0, min(100, int(round(100.0 * (1.0 - clamped)))))


def compose(port: str, share: float, label: str, detail: str, elapsed: float, size=None) -> Layout:
    """Full-screen layout centering the Braille progress panel inside the CRT Chrome HUD."""
    width = size.width if size else 120
    bar_cells = min(46, max(24, width // 3))
    pct = remaining_text(share)
    spin = SPIN[int(max(0.0, elapsed) * SPIN_HZ) % len(SPIN)]
    left_room = max(8, bar_cells - len(pct) - 3)
    head = Text.assemble((spin, 'value'), ' ', (label[:left_room].ljust(left_room), 'name'),
                         ' ', (pct, 'value'))
    head.no_wrap = True
    clk = '%d:%02d' % (int(elapsed) // 60, int(elapsed) % 60)
    det_room = max(8, bar_cells - len(clk) - 1)
    foot = Text.assemble((detail[:det_room].ljust(det_room), 'label'), ' ', (clk, 'label'))
    foot.no_wrap = True
    body = Group(head, Text(''), braille_bar(share, bar_cells), Text(''), foot)
    card = Panel(body, title=Text(' COMPILING 3D & HOST CORES ', style='name'),
                 title_align='left', box=box.ROUNDED, border_style='frame.hud',
                 padding=(1, 2), width=bar_cells + 6)
    centred = Chrome(Align(card, align='center', vertical='middle'),
                     'COAXIAL 63100', lock=True, tags=True)
    bar = footer((('Q', 'EXIT'),))
    whole = Layout()
    whole.split_column(
        Layout(band_of('COAXIAL 63100', 'PORT: %s' % port, Text('WARMING CORES', style='bar.dim')),
               size=1),
        Layout(centred, name='body'),
        Layout(bar, size=bar.row_count),
    )
    return whole


def cold_tasks(port: str, simulated: bool, state: dict, include_preload: bool = True) -> list:
    """Return `[(key, label, runner)]` for cold 3D/board/host C11 cores and STL decimates."""
    from coaxial.draw import orientation
    from coaxial.graphics import preload as pickled
    from terminal import loader
    from tools.cores import identify, native, prefetch, raster
    from tools.target import current

    tasks = []
    if not current.is_current():
        tasks.append(('images', 'STM32 FIRMWARE IMAGES',
                      lambda step: (step(0.1, 'NINJA DEBUG & RELEASE'),
                                    current.ensure(quiet=True))))
    if not raster.current():
        tasks.append(('raster', '3D ENGINE & BRAILLE C11',
                      lambda step: (step(0.15, 'MESH, TRIANGLES & BRAILLE'),
                                    raster.ensure())))
    if not simulated and str(port).startswith('native:') and not native.current():
        tasks.append(('board', 'STM32H753 BOARD & WORLD C11',
                      lambda step: (step(0.1, 'FIRMWARE & WORLD SIL'),
                                    native.ensure())))
    if not prefetch.current() or not identify.current():
        def _warm_host_cores(step):
            if not prefetch.current():
                step(0.1, 'L1/L2/L3 CACHE LOADER C11')
                prefetch.ensure()
            if not identify.current():
                step(0.55, 'SYSID & KINEMATICS C11')
                identify.ensure()
        tasks.append(('cores', 'CACHE & KINEMATICS C11', _warm_host_cores))
    if include_preload and not pickled.whole(pickled.load(orientation.MODEL)):
        def _warm_mesh(step):
            seen = [0]

            def on_step(line: str):
                seen[0] += 1
                step(min(0.95, seen[0] / 10.0), line.upper())

            loader.preload(state, interactive=False, on_step=on_step)
        tasks.append(('mesh', '3D MODEL DECIMATES & SHADOWS', _warm_mesh))
    return tasks


def run_in_curtain(live, keys, page, port: str, tasks: list) -> bool:
    """Run `tasks` concurrently inside an active `curtain(page)`, returning `False` on quit."""
    if not tasks:
        return True
    lock = threading.Lock()
    slots = [[0.0, False, label, 'COMPILING'] for _key, label, _fn in tasks]
    began = time.monotonic()

    def worker(idx: int, fn):
        def step(sub_share: float, detail: str = ''):
            with lock:
                slots[idx][0] = max(slots[idx][0], min(0.95, float(sub_share)))
                if detail:
                    slots[idx][3] = detail

        try:
            fn(step)
        finally:
            with lock:
                slots[idx][0] = 1.0
                slots[idx][1] = True

    for idx, (_key, _label, fn) in enumerate(tasks):
        threading.Thread(target=worker, args=(idx, fn), name='warmup-%d' % idx,
                         daemon=True).start()

    shown = 0.0
    while True:
        now = time.monotonic()
        elapsed = now - began
        creep = CREEP_CAP * (1.0 - math.exp(-elapsed / CREEP_TAU_S))
        with lock:
            snap = [list(s) for s in slots]
        done_n = sum(1 for s in snap if s[1])
        all_done = done_n == len(snap)
        eff = [1.0 if s[1] else max(s[0], creep) for s in snap]
        shown = 1.0 if all_done else max(shown, min(0.99, sum(eff) / len(snap)))
        active = next((s for s in snap if not s[1]), snap[-1])
        step_no = len(snap) if all_done else min(len(snap), done_n + 1)
        detail = 'STEP %d OF %d  %s' % (step_no, len(snap),
                                        'READY' if all_done else active[3])
        live.update(compose(port, shown, active[2], detail, elapsed, page.size),
                    refresh=True)
        if all_done:
            time.sleep(0.06)
            return True
        leave, _moved, typed = paced(keys, max(0.0, (1.0 / FPS_CAP) - (time.monotonic() - now)))
        if leave == 'quit' or any(k in ('q', 'Q') for k in typed):
            return False


def ensure(port: str, tasks: list) -> bool:
    """Open a temporary `curtain` to run `tasks` before a direct view; `False` if quit."""
    if not tasks:
        return True
    page = stage()
    if not page.is_terminal:
        for _key, _label, fn in tasks:
            fn(lambda *_a: None)
        return True
    with curtain(page) as live, Keys(True, quits=frozenset(), menus=frozenset()) as keys:
        return run_in_curtain(live, keys, page, port, tasks)
