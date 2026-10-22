"""Where BOARD ATTITUDE is drawn: a process of its own, this one's crew meanwhile, the hand-over.

The drawing's process (`coaxial.draw.easel`) where it comes up on a card; until it is up, this
process's own crew on a card the front page left warm, the exposure and the three frames' vote
carried across; for good where that process cannot draw, `bands`' crew - the card's, else
processes -, and the photographic renderer's `workshop`.
"""
from coaxial.draw import farm, orientation
from coaxial.graphics import card, gpu, shading
from terminal.ui.screen import say


def workshop(args):
    """A pool of drawing processes, or None if this run is too short for
    one.
    """
    if args.frames and args.frames <= 4:
        return None
    if not args.photo:
        # The toon mesh draws in 12 ms single-process, measured, against 108
        # for the photographic one: a pool would cost more in spawn time than
        # it saves, and render() only draws the toon package when no shop is
        # passed.
        return None

    try:
        return _announced(farm.Farm(orientation.MODEL_MESH))
    except (OSError, ValueError) as exc:
        say('warn', 'drawing', 'one process only: %s' % exc)
        return None


def _announced(pool):
    name = getattr(pool, 'name', None)
    say('ok', 'drawing', ('the GPU, %s' % name) if name else
        '%d processes, one band of the picture each' % pool.workers)
    return pool


def bands(args, step):
    """The staged engine's crew for the vector drawing, or None for a run
    too short to pay for spawning it.
    """
    from coaxial.graphics import creases, crew, gpu, shading, solids, stereotype, wireframe
    levels = len(solids.LODS)

    def landed(done, _total, divisions):
        step(0.15 + 0.45 * done / levels, 'DECIMATING GRID %d' % divisions)

    lods = wireframe._lods(landed)
    step(0.62, 'SHADOW CASTERS')
    shading._shadowmap((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    # The outline's exact index and its loops: 0.6 s once, here behind the
    # strip rather than as a hitch on the first frame.
    step(0.66, 'OUTLINE EDGES')
    creases._outline_source()
    stereotype._stereotypes()
    if args.photo or (args.frames and args.frames <= 4):
        return None
    step(0.70, 'THE GPU, OR %d PROCESSES' % crew.MAX_WORKERS)
    try:
        return _announced(gpu.crew_for(lods, art=shading._face()))
    except (OSError, ValueError) as exc:
        say('warn', 'drawing', 'one process only: %s' % exc)
        return None


def apart(args):
    """Whether the toon drawing runs in a process of its own (`easel`): not the photographic
    one, which bands its own, nor a run too short to pay for starting it."""
    return not args.photo and not (args.frames and args.frames <= 4)


def warm_here(size, zoom, held):
    """This process's crew on the front page's warm card, a frame drawn with it at the page's size
    while the board opens - its waits on the wire leave the interpreter free; the crew in
    `held['crew']`."""
    crew = gpu.card_crew((), shading._face())
    orientation.render((0.0, 0.0, 0.0, 1.0), width=size[0], height=size[1], zoom=zoom,
                       toon=True, wire=True, colour=True, crew=crew, persist={}, scroll=0.0,
                       ahead=True)
    held['crew'] = crew


def handed(drawing, persist):
    """Whether the drawing's process draws this frame: once it is up, the exposure and the three
    frames' vote carried to it the first time, the hand-over unseen; after a fault in it, never -
    this process draws on."""
    try:
        up = drawing['ahead'] is not None and drawing['ahead'].up()
    except RuntimeError:
        drawing['ahead'].close()
        drawing['ahead'] = None
        return False
    if up and not drawing['handed']:
        drawing['handed'] = True
        drawing['carried'] = {k: persist[k] for k in ('exposure', 'steady') if k in persist}
    return up


def where(args, step, drawing, pool, warmed):
    """(drawing, pool): the drawing's process once it is up on a card, and while it starts - this
    process's card warm, as the front page's stand leaves it - a crew of this process's own on it,
    handed over as the process comes up (`handed`); else closed - no card answers there, or its
    start failed - and this process draws on `bands`' crew, said why."""
    if drawing is None:
        return None, pool
    try:
        here = card.held() is not None and not drawing.up()
        name, why = (None if here else drawing.name), 'no card answers'
    except RuntimeError as exc:
        here, name, why = False, None, str(exc)
    warming, held = warmed
    if here and warming is not None:
        warming.join()
        say('ok', 'drawing', 'here on the card while its own process starts')
        return drawing, held.get('crew') or gpu.card_crew((), shading._face())
    if name is not None:
        say('ok', 'drawing', 'a process of its own, on %s' % name)
        return drawing, pool
    say('warn', 'drawing', 'in this process: %s' % why)
    drawing.close()
    return None, bands(args, step)
