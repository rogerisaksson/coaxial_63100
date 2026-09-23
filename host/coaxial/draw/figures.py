"""The notebooks' figures, in one shape."""
from coaxial.errors import RigError

#: The page: inches wide, and inches a panel row.
WIDTH = 9.0
ROW = 3.0
#: The grid's ink, faint enough to read under a trace.
GRID_ALPHA = 0.3


def figure(rows=1, cols=1, title=None, sharex=False):
    """`(fig, panels)` in the house shape: `rows` by `cols` panels, the
    panels a flat list top-left first whatever the shape asked for, each
    with the grid on.
    """
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
