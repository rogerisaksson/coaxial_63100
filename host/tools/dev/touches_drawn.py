"""What a change to the robot as drawn reaches: its body's, its likeness's and its guards' modules."""
from tools.dev.suites import GYNOID, RENDER, STRUCTURE, VIEWS

_BARE = ('test_views_bared.py', 'test_views_seat.py', 'test_views_marks.py', 'test_views_likeness.py',
         'test_views_clothes.py', 'test_views_lettering.py')
_DRESSED = ('test_views_humanoid.py', 'test_views_likeness.py', 'test_views_clothes.py')

#: The robot's drawing's rows of the path map (`touches.TOUCHES`), a module each: touches.py stood 6 characters
#: under its budget.
DRAWN = (
    ('host/coaxial/graphics/gynoid.py',        (RENDER, *VIEWS, GYNOID)),
    ('host/coaxial/graphics/shapes.py',        (RENDER, *VIEWS, GYNOID)),
    ('host/coaxial/graphics/lit.py',           (RENDER, *VIEWS, GYNOID)),
    ('host/coaxial/graphics/callouts.py',      (RENDER, *VIEWS, GYNOID)),
    ('host/coaxial/graphics/panels.py',        (RENDER, *VIEWS, GYNOID)),
    ('host/coaxial/graphics/drape.py',         (STRUCTURE, *_DRESSED, 'test_views_seat.py')),
    ('host/coaxial/graphics/seat.py',          (STRUCTURE, *_BARE)),
    ('host/coaxial/graphics/likeness.py',      (STRUCTURE, *_DRESSED)),
    ('host/coaxial/graphics/garment.py',       (STRUCTURE, 'test_views_likeness.py', 'test_views_clothes.py')),
    ('host/coaxial/graphics/fall.py',          (STRUCTURE, 'test_views_likeness.py', 'test_views_clothes.py')),
    ('host/coaxial/graphics/wavefront.py',     (STRUCTURE, 'test_views_likeness.py')),
    ('host/coaxial/graphics/under.py',         (STRUCTURE, *_DRESSED)),
    ('host/coaxial/graphics/marks.py',         (STRUCTURE, 'test_views_lettering.py')),
    ('host/coaxial/graphics/lettering.py',     (STRUCTURE, 'test_views_lettering.py')),
    ('host/coaxial/graphics/plated.py',        (STRUCTURE, *_BARE)),
    ('host/coaxial/graphics/guards.py',        (STRUCTURE, *_BARE)),
    ('host/coaxial/graphics/livery.py',        (STRUCTURE, *_BARE)),
    ('host/coaxial/graphics/form.py',          (STRUCTURE, 'test_views_marks.py', 'test_views_bared.py')),
    ('host/tools/sim/alike.py',                (STRUCTURE, 'test_views_bared.py')),
    ('host/tools/sim/shown.py',                (STRUCTURE, 'test_views_clothes.py')),
    ('host/tools/sim/sewn.py',                 (STRUCTURE, 'test_views_likeness.py', 'test_views_clothes.py', 'test_views_marks.py')),
)
