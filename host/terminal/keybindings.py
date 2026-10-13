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

#: Directional compound chords for TPW combat steering.
CHORDS = {
    ('w', 'a'): 'turn_left',
    ('a', 'w'): 'turn_left',
    ('w', 'd'): 'turn_right',
    ('d', 'w'): 'turn_right',
    ('s', 'a'): 'turn_back_left',
    ('a', 's'): 'turn_back_left',
    ('s', 'd'): 'turn_back_right',
    ('d', 's'): 'turn_back_right',
}

#: Standard footer bars for the terminal stage.
BAR_NORMAL = (
    ('§/~', 'TPW'), ('TAB', 'KEYS'), ('ARROWS', 'WALK'), ('< ENT', 'SCENARIO'),
    ('STUNTS', 's e x m..'), ('SPC', 'PUSH'), ('A', 'RESET: OFF'), ('Q', 'EXIT')
)

BAR_TPW = (
    ('§/~', 'NORMAL'), ('TAB', 'KEYS'), ('WASD', 'MOVE'), ('SHIFT', 'SPRINT'),
    ('F', 'INTERACT'), ('MOUSE', 'AIM/LOOK'), ('SPC', 'STRIKE'), ('Q', 'EXIT')
)


def bar_for(wasd_mode):
    """The bottom bar: adapts when Third Person Control (TPW) is active."""
    return BAR_TPW if wasd_mode else BAR_NORMAL


def is_toggle_tpw(key):
    """Whether `key` toggles Third Person Control."""
    return key in TOGGLE_TPW_KEYS


def describe_action(key, wasd_mode=False):
    """Human-readable explanation of what `key` performs in the active mode."""
    if is_toggle_tpw(key):
        return 'toggle Third Person Control (§ / ~ / F2)'
    if wasd_mode:
        if key in ('w', 'W'):
            return 'TPW: advance forward along aim crosshair'
        if key in ('s', 'S'):
            return 'TPW: tactical backpedal / retreat'
        if key in ('a', 'A'):
            return 'TPW: strafe left'
        if key in ('d', 'D'):
            return 'TPW: strafe right'
        if key in ('f', 'F'):
            return 'TPW: interact (sit / lie / rise)'
        if key in ('shift', 'Shift', 'sprint'):
            return 'TPW: sprint / jog'
        if key in ('wa', 'WA'):
            return 'TPW: turn left (WA, vänstersväng)'
        if key in ('wd', 'WD'):
            return 'TPW: turn right (WD, högersväng)'
        if key in ('sa', 'SA'):
            return 'TPW: turn back left (SA, bakåt-vänstersväng)'
        if key in ('sd', 'SD'):
            return 'TPW: turn back right (SD, SD-högersväng)'
    if key in ('a', 'A'):
        return 'a: inactive (mode switch - restart disabled; use § / ~)'
    if key == 'up':
        return 'arrow up: walk forward (#204)'
    if key == 'down':
        return 'arrow down: step backward / brake (#205)'
    if key == 'left':
        return 'arrow left: sway / strafe left'
    if key == 'right':
        return 'arrow right: sway / strafe right'
    if key in ('\r', '\n'):
        return 'enter: run scenario action'
    if key == '4':
        return 'say: 3 (#204 walk) [gentle walk]'
    if key in '123567890':
        return 'say: digit %s' % key
    if key in ('w', 'W'):
        return 'walk from rest'
    if key in ('<', ','):
        return 'cycle scenario'
    if key in ('f', 'F'):
        return 'stunt: lie_floor'
    if key in ('d', 'D'):
        return 'stunt: hop_down_chair'
    if key in ('s', 'S'):
        return 'stunt: sit_chair'
    if key == ' ':
        return 'charge push'
    rev_stunts = {v: k for k, v in STUNT_KEYS.items()}
    if key.lower() in rev_stunts:
        return 'stunt: %s' % rev_stunts[key.lower()]
    return key


def groups():
    """Grouped key help definitions for the TAB overlay."""
    return (
        ('THIRD PERSON CONTROL (TPW)', (
            ('§ / ~ / F2', 'toggle Third Person Control ON / OFF'),
            ('W', 'advance forward along aim crosshair'),
            ('S', 'tactical backpedal / retreat'),
            ('A / D', 'tactical strafe left / right'),
            ('WA / WD', 'compound forward turns'),
            ('SHIFT', 'sprint / jog'),
            ('F', 'universal interact (sit / lie / rise)'),
            ('SPACE', 'hold to charge kinetic push / strike'),
            ('MOUSE L-DRAG', '3D mouselook & dynamic steering'),
            ('MOUSE WHEEL', 'zoom in (aim) / zoom out'),
        )),
        ('MOVE (ARROWS)', (
            ('UP', 'walk forward (#204 gentle cruise)'),
            ('DOWN', 'step backward / brake (#205)'),
            ('LEFT RIGHT', 'sway / strafe left / right'),
            ('1-5', '#201 stand, #202 sway, #203 step, #204 walk, #205 back'),
            ('A', 'disabled in normal mode (switch with § / ~)'),
        )),
        ('SCENARIO', (
            ('< ,', 'cycle scenario + auto-move'),
            ('ENTER', 'run / replay active scenario'),
            ('H', 'toggle automated house tour / autopilot'),
            ('scenarios', 'flat, chair, bed, slope, stairs, ice, snow, obstacle, sill, hole, rug, glass_room'),
        )),
        ('STUNTS', (
            ('s / u', 'sit chair / rise chair (in normal mode)'),
            ('j / d', 'hop up chair / hop down chair'),
            ('e / g', 'lie bed / rise bed'),
            ('f / k', 'lie floor / rise floor'),
            ('x / n', 'stairs walk up / stairs down'),
            ('m / i', 'slope walk / ice walk (slick)'),
            ('y / z', 'parry shove / controlled breakfall'),
            ('b', 'ballistic walk'),
        )),
        ('PUSH / STRIKE', (
            ('SPACE', 'hold to charge push along view vector'),
            ('P N', 'lateral push / nudge'),
            ('shift', 'sprint in TPW or longitudinal push'),
        )),
        ('VIEW (MOUSE)', (
            ('L-DRAG', 'rotate / orbit camera (mouselook)'),
            ('R-DRAG', 'pan viewport'),
            ('WHEEL', 'zoom in / out'),
            ('V', 'home camera view'),
            ('O', 'toggle auto-orbit'),
            ('C', 'cycle skin: dressed, shell, mechanism, actuators'),
            ('L / T', 'joint callouts / torque & power'),
        )),
        ('PAGE', (
            ('R', 'record / save CSV'),
            ('TAB ?', 'these keys'),
            ('Q / ESC', 'exit'),
        )),
    )
