"""Centralized keybindings and input definitions for terminal views and locomotion control.

Consolidates mode toggles, WASD/TPW navigation, stunt hotkeys, and help bars across
humanoid views without per-view hardcoding.
"""

#: Keys that toggle Third Person Control (TPW / WASD mode).
#: Includes Nordic section symbol '§', US tilde '~' and backtick '`',
#: and function keys F1-F3 (and their terminal escape sequences).
TOGGLE_TPW_KEYS = (
    '§', '½', '~', '`',
    'f1', 'F1', 'f2', 'F2', 'f3', 'F3',
    '\x1bOP', '\x1bOQ', '\x1bOR',
    '\x1b[11~', '\x1b[12~', '\x1b[13~',
)

#: Standard stunts mapped to direct keypresses.
STUNT_KEYS = {
    'weight_shift': 'h',
    'bow': ';',
    'ballistic_walk': 'b',
    'sit_chair': 's',
    'rise_chair': 'u',
    'lie_bed': 'e',
    'rise_bed': 'g',
    'lie_floor': 'f',
    'rise_floor': 'k',
    'hop_down_chair': 'd',
    'hop_up_chair': 'j',
    'stairs_walk': 'x',
    'stairs_down': 'n',
    'ice_walk': 'i',
    'slope_walk': 'm',
    'parry_shove': 'y',
    'controlled_fall': 'z',
    'circle_walk': 'q',
}

#: Canonical directional compound chords for TPW combat steering.
CHORD_PAIRS = (
    ('wa', 'turn_left'), ('wd', 'turn_right'),
    ('sd', 'turn_back_right'), ('sa', 'turn_back_left'),
    ('wq', 'turn_left'), ('we', 'turn_right'),
    ('se', 'turn_back_right'), ('sq', 'turn_back_left'),
    ('ae', 'strafe_left'), ('aq', 'strafe_left'),
    ('dq', 'strafe_right'), ('de', 'strafe_right'),
)
CHORDS = {pair: mv for tag, mv in CHORD_PAIRS for pair in ((tag[0], tag[1]), (tag[1], tag[0]))}

#: Scenario deck metadata: {scenario_name: (default_move, tactical_objective)}
SCENARIO_INFO = {
    'flat': ('#201', 'level ground // nominal stance'),
    'glass_room': ('circle_walk', 'proving ground // multi-prop course'),
    'chair': ('sit_chair', 'chair // sit [s], rise [u], hop [j/d]'),
    'bed': ('lie_bed', 'soft bed // recline [e] & rise [g]'),
    'mattress': ('lie_floor', 'floor mattress // lie [f] & rise [k]'),
    'slope': ('slope_walk', 'inclined ramp // uphill pitch adaptation'),
    'slope_down': ('slope_walk', 'declined ramp // downhill grade braking'),
    'cross_slope': ('slope_walk', 'camber ramp // ankle roll compensation'),
    'stairs': ('stairs_walk', '5-step riser // stair ascent [x]'),
    'stairs_down': ('stairs_down', '5-step descent // controlled step-down [n]'),
    'ice': ('ice_walk', 'low-mu ice // flat-foot slick shuffle [i]'),
    'snow': ('#204', 'yielding snow // high-clearance tread'),
    'asphalt': ('#204', 'rough asphalt // firm outdoor stride'),
    'slip': ('ice_walk', 'slick patch // low-friction recovery [i]'),
    'obstacle': ('#204', 'urban obstacles // wall & tree clearance'),
    'wall': ('#204', 'masonry wall // compliant contact arrest'),
    'tree': ('#204', 'tree trunk // compliant contact arrest'),
    'sill': ('#204', '35mm door sill // trip & stumble reflex'),
    'hole': ('#204', '10cm floor hole // drop & step recovery'),
    'rug': ('#204', 'sliding rug // shear slip & balance'),
}

SCENARIO_MOVES = {k: v[0] for k, v in SCENARIO_INFO.items()}
SCENARIO_DESC = {k: v[1] for k, v in SCENARIO_INFO.items()}

#: Standard footer bars for the terminal stage.
BAR_NORMAL = (
    ('§/~', 'TPW'), ('TAB', 'KEYS'), ('ARROWS', 'WALK'), ('< ENT', 'SCENARIO'),
    ('STUNTS', 's e x m..'), ('SPC', 'PUSH'), ('Q', 'CIRCLE'), ('ESC', 'MENU')
)

BAR_TPW = (
    ('§/~', 'NORMAL'), ('TAB', 'KEYS'), ('WASD+QE', 'MOVE'), ('Q/E', 'YAW'), ('SHIFT', 'SPRINT'),
    ('SPC', 'JUMP'), ('F', 'INTERACT'), ('MOUSE', 'AIM/LOOK'), ('ESC', 'MENU')
)


def bar_for(wasd_mode):
    """The bottom bar: adapts when Third Person Control (TPW) is active."""
    return BAR_TPW if wasd_mode else BAR_NORMAL


def is_toggle_tpw(key):
    """Whether `key` toggles Third Person Control."""
    return key in TOGGLE_TPW_KEYS


_TPW_BASE = {
    'w': 'TPW: advance forward in body frame (Shift+W to sprint)',
    's': 'TPW: reverse walk in body frame (Shift+S fast backpedal)',
    'a': 'TPW: pure lateral strafe left (Shift+A sprint strafe)',
    'd': 'TPW: pure lateral strafe right (Shift+D sprint strafe)',
    'q': 'TPW: pure angular velocity left (+wz: spin in place, W+Q arc, D+Q circle-strafe)',
    'e': 'TPW: pure angular velocity right (-wz: spin in place, W+E arc, A+E circle-strafe)',
    ' ': 'TPW: jump in stride / hop',
    'f': 'TPW: interact (sit / lie / rise)',
    'shift': 'TPW: sprint / jog',
    'sprint': 'TPW: sprint / jog',
    'wa': 'TPW: curvilinear arc left (WA, vx > 0, wz > 0, vy = 0)',
    'wq': 'TPW: curvilinear arc left (WQ, vx > 0, wz > 0, vy = 0)',
    'wd': 'TPW: curvilinear arc right (WD, vx > 0, wz < 0, vy = 0)',
    'we': 'TPW: curvilinear arc right (WE, vx > 0, wz < 0, vy = 0)',
    'sa': 'TPW: reverse curvilinear arc left (SA, vx < 0, wz > 0, vy = 0)',
    'sq': 'TPW: reverse curvilinear arc left (SQ, vx < 0, wz > 0, vy = 0)',
    'sd': 'TPW: reverse curvilinear arc right (SD, vx < 0, wz < 0, vy = 0)',
    'se': 'TPW: reverse curvilinear arc right (SE, vx < 0, wz < 0, vy = 0)',
    'ae': 'TPW: circle-strafe left (A+E, vy > 0, wz < 0)',
    'aq': 'TPW: circle-strafe left (A+Q, vy > 0, wz > 0)',
    'dq': 'TPW: circle-strafe right (D+Q, vy < 0, wz > 0)',
    'de': 'TPW: circle-strafe right (D+E, vy < 0, wz < 0)',
}
_TPW_DESCRIPTIONS = {v_k: desc for k, desc in _TPW_BASE.items() for v_k in (k, k.upper(), k.capitalize())}

_COMMON_DESCRIPTIONS = {
    'up': 'arrow up: walk forward (#204)',
    'down': 'arrow down: step backward / brake (#205)',
    'left': 'arrow left: sway / strafe left',
    'right': 'arrow right: sway / strafe right',
    '\r': 'enter: run scenario action',
    '\n': 'enter: run scenario action',
    '4': 'say: 3 (#204 walk) [gentle walk]',
    '<': 'cycle scenario',
    ',': 'cycle scenario',
    ']': 'cycle scenario',
    '[': 'previous scenario',
    '>': 'switch gym deck (gynoid_mesomorph_medium / examples)',
    'H': 'toggle automated house tour / autopilot',
    'N': 'severe shove (parry / controlled breakfall)',
    ' ': 'charge push',
}


def describe_action(key, wasd_mode=False):
    """Human-readable explanation of what `key` performs in the active mode."""
    if is_toggle_tpw(key):
        return 'toggle Third Person Control (§ / ~ / F2)'
    if wasd_mode and key in _TPW_DESCRIPTIONS:
        return _TPW_DESCRIPTIONS[key]
    if key in ('a', 'A'):
        return 'a: inactive (mode switch - restart disabled; use § / ~)'
    if key in _COMMON_DESCRIPTIONS:
        return _COMMON_DESCRIPTIONS[key]
    if len(key) == 1 and key in '123567890':
        return 'say: digit %s' % key
    if key in ('w', 'W'):
        return 'walk from rest'
    rev_stunts = {v: k for k, v in STUNT_KEYS.items()}
    if key.lower() in rev_stunts:
        return 'stunt: %s' % rev_stunts[key.lower()]
    return key


def groups():
    """Grouped key help definitions for the TAB overlay."""
    return (
        ('THIRD PERSON CONTROL (SE(2) BODY FRAME)', (
            ('§ / ~ / F2', 'toggle Third Person Control ON / OFF'),
            ('WASD+QE', 'walk (W/S), strafe (A/D), pure yaw spin in place (Q/E)'),
            ('WA/WD / WQ/WE / SA/SD / SQ/SE', 'curvilinear arc turn forward or backward'),
            ('A+E / D+Q', 'circle-strafe around a point ahead (lateral + yaw)'),
            ('SHIFT + WASD+QE', 'run / sprint in any direction or yaw rate'),
            ('SPACE', 'jump in stride / hop (or hop down from chair)'),
            ('F', 'universal interact (sit / lie / rise)'),
            ('X / N', 'stairs ascent / stairs descent'),
            ('P / SHIFT+N', 'moderate shove (parry) / severe shove (breakfall)'),
            ('MOUSE L-DRAG', 'free-orbiting camera (decoupled from body heading)'),
            ('MOUSE WHEEL', 'zoom in / zoom out'),
        )),
        ('MOVE (ARROWS)', (
            ('UP', 'walk forward (#204 gentle cruise)'),
            ('DOWN', 'step backward / brake (#205)'),
            ('LEFT RIGHT', 'sway / strafe left / right'),
            ('1-5', '#201 stand, #202 sway, #203 step, #204 walk, #205 back'),
            ('A', 'disabled in normal mode (switch with § / ~)'),
        )),
        ('SCENARIO DECK', (
            ('< , ] / [', 'next / previous scenario + auto-stage'),
            ('ENTER', 'execute scenario (or toggle rise on chair/bed/mattress)'),
            ('H / >', 'toggle house tour autopilot / switch gym deck'),
            ('20 scenarios', 'flat, glass_room, chair, bed, mattress, slope/down/cross, stairs/down, ice, snow, asphalt, slip, obstacle, wall, tree, sill, hole, rug'),
        )),
        ('STUNTS', (
            ('s / u', 'sit chair / rise chair (in normal mode)'),
            ('j / d', 'hop up chair / hop down chair'),
            ('e / g', 'lie bed / rise bed'),
            ('f / k', 'lie floor / rise floor'),
            ('x / n', 'stairs walk up / stairs down'),
            ('m / i', 'slope walk / ice walk (slick)'),
            ('y / z', 'parry shove / controlled breakfall'),
            ('b / q', 'ballistic walk / circle walk'),
        )),
        ('PUSH / STRIKE', (
            ('SPACE', 'hold to charge push along view vector'),
            ('p / P', 'moderate shove lateral / fore-aft (parry)'),
            ('SHIFT+N', 'severe shove (attempt parry / breakfall)'),
        )),
        ('OPTIC & VIEW', (
            ('L-DRAG', 'rotate / orbit camera (mouselook)'),
            ('R-DRAG', 'pan viewport'),
            ('WHEEL', 'zoom in / out'),
            ('V / O', 'home camera view / toggle auto-orbit'),
            ('C', 'cycle shell: dressed, shell, mechanism, actuators'),
            ('L / T', 'joint callouts / torque, power & temp'),
        )),
        ('CONSOLE', (
            ('R', 'record / save telemetry CSV'),
            ('TAB ?', 'toggle key matrix overlay'),
            ('Q / ESC', 'exit / return to menu'),
        )),
    )

