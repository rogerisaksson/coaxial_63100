"""The notebooks' figures, in one shape.

Every plot the examples draw goes through `figure`, so a reader sees
the same page whatever the subject: nine inches wide, three a panel,
a light grid, laid out tight. Nine sizes and no shared style was the
count before this, measured 2026-09-21 over twenty-two notebooks.

matplotlib is NOT a dependency of this library and is imported inside
`figure` rather than at the top, the way the acquisition front door
imports pandas: a bench that only reads a thermistor should not have
to install it, and the refusal says what to do.
"""
from .errors import RigError

#: The page: inches wide, and inches a panel row.
WIDTH = 9.0
ROW = 3.0
#: The grid's ink, faint enough to read under a trace.
GRID_ALPHA = 0.3


def figure(rows=1, cols=1, title=None, sharex=False):
    """`(fig, panels)` in the house shape: `rows` by `cols` panels, the
    panels a flat list top-left first whatever the shape asked for, each
    with the grid on. `title` sits over the page."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        raise RigError('figure() needs matplotlib, which this library does '
                       'not require - `pip install matplotlib`, or take the '
                       'numbers from the record and draw them your own way'
                       ) from None
    fig, axes = plt.subplots(rows, cols, sharex=sharex, squeeze=False,
                             figsize=(WIDTH, ROW * rows + (0.5 if title else 0.0)))
    panels = [ax for row in axes for ax in row]
    for ax in panels:
        ax.grid(True, alpha=GRID_ALPHA)
    if title:
        fig.suptitle(title)
    fig.tight_layout()
    return fig, panels


def show(fig):
    """Draw `fig` in the notebook and let it go, so a loop that redraws
    holds one figure at a time."""
    import matplotlib.pyplot as plt   # the same optional dependency as above
    plt.show()
    plt.close(fig)
