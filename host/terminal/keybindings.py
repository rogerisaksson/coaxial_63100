"""The HUMANOID page's key bar, its help, and what each scene asks of her."""

#: Each scene's objective (`machine.floor.SCENARIOS`).
SCENARIO_DESC = {
    'flat': 'level ground // stand and go',
    'lodge': 'the lodge // its deck, stair and steps',
    'chair': 'chair // F sit, Space hop',
    'bed': 'soft bed // F lies and rises',
    'mattress': 'floor mattress // F lies and rises',
    'slope': 'ramp // walk up 10 deg',
    'slope_down': 'ramp // walk down 10 deg',
    'cross_slope': 'camber // walk across 10 deg',
    'stairs': 'stairs // walk up',
    'stairs_down': 'stairs // walk down',
    'ice': 'ice // walk on, slick',
    'snow': 'snow // walk on, yielding',
    'asphalt': 'asphalt // walk on, rough',
    'slip': 'slick patch // walk over',
    'obstacle': 'wall and tree // walk into',
    'wall': 'wall // walk into, stand',
    'tree': 'tree // walk into, stand',
    'sill': 'door sill // trip, walk on',
    'hole': '10 cm hole // step in, walk on',
    'rug': 'loose rug // walk over',
    'edge_behind': "step's edge behind // back off it",
}

#: The bar under the page.
BAR = (('WASD', 'MOVE'), ('SHIFT', 'SPRINT'), ('Q E', 'TURN'), ('SPC', 'HOP'), ('F', 'INTERACT'),
       ('C', 'CROUCH'), ('H', 'TOUR'), ('/', 'CHAT'), ('X', 'SHOULDER'), ('[ ]', 'SCENE'), ('TAB', 'KEYS'), ('ESC', 'MENU'))


def groups():
    """The TAB help, its groups of (key, what)."""
    return (
        ('HER', (
            ('W A S D', 'walk ahead, sidestep left, walk back, sidestep right'),
            ('SHIFT', 'sprint'),
            ('Q E', 'turn left, right in place (or on an arc with W/S)'),
            ('SPACE', 'hop - onto a chair ahead, down from one'),
            ('F', 'sit, lie or rise: what is near or bears her'),
            ('C', 'crouch, and up'),
            ('1-0 / >', 'the gym\'s entries / its other text'),
        )),
        ('VIEW', (
            ('L-DRAG / ARROWS', 'look round her; behind her again as she goes'),
            ('X', 'the other shoulder'),
            ('Z', "her horizon: each sole's landing room, her capture point"),
            ('R-DRAG / WHEEL', 'pan / zoom'),
            ('V / O', 'behind her / orbit'),
            ('M', 'dressed, shell, mechanism, drives'),
            ('L / T', 'callouts / torque, power, temp'),
        )),
        ('LAB', (
            ('[ ] / ENTER', 'scene before, after / again'),
            ('P held', 'charge a push along the view'),
            ('SHIFT+P / SHIFT+N', '80 N / 200 N along the view'),
            ('H / /', 'the tour of the scenes / ask LLM prompt'),
            ('BACKSPACE', 'laid anew'),
            ('R', 'record / save'),
            ('TAB ? / ESC', 'these keys / menu'),
        )),
    )
