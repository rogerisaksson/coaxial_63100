"""The board's attitude on an easel of its own: drawn in the process `terminal.ui.ahead` starts.

Its crew and its frames' state - the held face, the exposure, the three frames' vote - are kept
there from one ask to the next; the page asks a pose and shows the one asked a frame before,
its instruments on one core while the drawing takes another.
"""
from coaxial.draw import orientation

#: The process's crew - the card's, None without one - and the frames' state, by `warm`.
HELD: dict = {}

#: The small frame `warm` draws, cells across and down: the frame path's C types parsed and its
#: shader compiled while the page opens its board, not in the page's first frame (0.36 s).
WARM_FRAME = (16, 8)


def warm():
    """Everything a first frame waits on, built before the drawing says it is up: the levels of
    detail, the shadow casters, the outline and its stereotypes, the card's crew - a level
    uploaded when first drawn - and a small frame drawn on it. The process is a daemon and has
    no children: the levels no preload holds are decimated here, one after another, where
    `wireframe._lods` hands two or more to a pool - the bench's cold preload stopped the page's
    start on it - and the bundle is written for the next start where there is room."""
    from coaxial.graphics import creases, gpu, preload, shading, solids, stereotype, wireframe
    path = orientation.MODEL
    held = preload.load(path)
    if held is not None:
        wireframe._adopt(path, held)
    for divisions in preload.levels():
        solids._decimated(path, divisions)
    shading._shadowmap((1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0))
    creases._outline_source()
    stereotype._stereotypes()
    if not preload.whole(held) and preload.refusal() is None:
        preload.build(path)
    crew = gpu.card_crew((), shading._face())
    HELD.update(crew=crew, persist={})
    if crew is not None:
        orientation.render((0.0, 0.0, 0.0, 1.0), *WARM_FRAME, toon=True, wire=True,
                           colour=True, crew=crew, persist={}, scroll=0.0)


def render(q, width, height, zoom=1.0, colour=False, frame_on=True, scroll=None, carried=None):
    """`orientation.render`'s toon drawing of `q` with the held crew, its frames' state carried
    from one ask to the next - and from the page's own, `carried`, when it hands over."""
    if not HELD:
        warm()
    HELD['persist'].update(carried or {})
    return orientation.render(q, width=width, height=height, zoom=zoom, toon=True, wire=True,
                              colour=colour, frame_on=frame_on, crew=HELD['crew'],
                              persist=HELD['persist'], scroll=scroll)
