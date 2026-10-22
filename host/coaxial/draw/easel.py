"""The board's attitude on an easel of its own: drawn in the process `terminal.ui.ahead` starts.

Its crew and its frames' state - the held face, the exposure, the three frames' vote - are kept
there from one ask to the next; the page asks a pose and shows the one asked a frame before,
its instruments on one core while the drawing takes another.
"""
from coaxial.draw import orientation

#: The process's crew, the frames' state and whether the crew paints a pose behind, by `warm`.
HELD: dict = {}


#: The small frame `warm` draws, cells across and down: the frame path's C types parsed and its
#: shader compiled while the page opens its board, not in the page's first frame (0.36 s).
WARM_FRAME = (16, 8)


def warm():
    """Everything a first frame waits on, built before the drawing says it is up: the levels of
    detail, the shadow casters, the outline and its stereotypes, the crew - the card's, a level
    uploaded when first drawn, else processes, which paint a pose behind to raster beside the
    painting - and a small frame drawn on it."""
    from coaxial.graphics import creases, crew as crewmod, gpu, shading, stereotype, wireframe
    lods = wireframe._lods()
    identity = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    shading._shadowmap(identity)
    creases._outline_source()
    stereotype._stereotypes()
    crew = gpu.card_crew((), shading._face()) or crewmod.Crew(lods, shading._face())
    HELD.update(crew=crew, persist={}, behind=not isinstance(crew, gpu.GpuCrew))
    orientation.render((0.0, 0.0, 0.0, 1.0), *WARM_FRAME, toon=True, wire=True, colour=True,
                       crew=crew, persist={}, scroll=0.0, ahead=HELD['behind'])


def render(q, width, height, zoom=1.0, colour=False, frame_on=True, scroll=None):
    """`orientation.render`'s toon drawing of `q` with the held crew, its frames' state carried."""
    if not HELD:
        warm()
    return orientation.render(q, width=width, height=height, zoom=zoom, toon=True, wire=True,
                              colour=colour, frame_on=frame_on, crew=HELD['crew'],
                              persist=HELD['persist'], scroll=scroll, ahead=HELD['behind'])
