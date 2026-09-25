"""Machine types: a body, and the routines a program calls by name.

A body is subsystems in bus order - bus 1 the first - each one kind of actuator, named
outward along its bus: which board is which is measured (`machine.machine.fit`). A routine
is lines of the program grammar with {param} in them, `{-p}` its negative and `{p*k}`
scaled; a line calls it: `0 run=walk times=4 stride=25`. No labels inside: a routine
repeats by `times`. A line that goes somewhere waits to arrive, its seconds the timeout;
one that lasts - a gait, a hover, a manoeuvre - says wait=time. Add a type: TYPES['exo'] = Type([Subsystem('legs', 'joint', (..))], {..}).
"""
from collections import namedtuple

#: Lines with {param}s, and every param's default.
Routine = namedtuple('Routine', 'text defaults')

#: One bus of a body: its name, the kind its actuators are, their names outward.
Subsystem = namedtuple('Subsystem', 'name kind actuators')

#: `body` [Subsystem], bus 1 first; `routines` {name: Routine}; `failsafe` a program safe
#: from anywhere, what `Live` plays on silence, a trip or a stop.
Type = namedtuple('Type', 'body routines failsafe')


def _limb(name, side, *joints):
    return Subsystem(name, 'joint', tuple(side + j for j in joints))


LEGS_ZERO = 'left_hip=0 right_hip=0 left_knee=0 right_knee=0 left_ankle=0 right_ankle=0'
ROTORS = 'rotor_fl={0} rotor_fr={0} rotor_rl={0} rotor_rr={0}'

TYPES = {
    'humanoid': Type([
        _limb('axis', '', 'pelvis', 'waist', 'neck', 'head'),
        _limb('left_arm', 'left_', 'shoulder', 'elbow', 'wrist', 'gripper'),
        _limb('left_leg', 'left_', 'hip', 'knee', 'ankle', 'foot'),
        _limb('right_arm', 'right_', 'shoulder', 'elbow', 'wrist', 'gripper'),
        _limb('right_leg', 'right_', 'hip', 'knee', 'ankle', 'foot')], {
        'stand': Routine('{seconds} ' + LEGS_ZERO, {'seconds': 1.0}),
        'squat': Routine(
            '{seconds} left_hip={-hip} right_hip={-hip} left_knee={knee} right_knee={knee} '
            'left_ankle={-ankle} right_ankle={-ankle}\n{seconds} ' + LEGS_ZERO,
            {'seconds': 1.2, 'hip': 30, 'knee': 60, 'ankle': 30}),
        'walk': Routine(
            '{period} wait=time left_hip={-stride} left_knee={lift} right_hip={stride} '
            'right_knee=0\n'
            '{period} wait=time left_knee=0\n'
            '{period} wait=time right_hip={-stride} right_knee={lift} left_hip={stride} '
            'left_knee=0\n'
            '{period} wait=time right_knee=0',
            {'stride': 20, 'lift': 30, 'period': 0.4}),
        'wave': Routine('{seconds} wait=time right_shoulder={raise} right_elbow=45\n'
                        '{seconds} wait=time right_shoulder={raise} right_elbow=-20',
                        {'seconds': 0.6, 'raise': 80}),
        'look': Routine('{seconds} head={yaw} neck={pitch}',
                        {'seconds': 0.8, 'yaw': 0, 'pitch': 0}),
        'rest': Routine('{seconds} right_shoulder=0 right_elbow=0 left_shoulder=0 '
                        'left_elbow=0 head=0 neck=0', {'seconds': 1.0}),
    }, '0 run=stand'),
    'gynoid': Type([
        _limb('axis', '', 'spine', 'spine_roll', 'waist', 'neck', 'head'),
        _limb('left_arm', 'left_', 'shoulder', 'elbow', 'wrist', 'gripper'),
        _limb('left_leg', 'left_', 'hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll', 'foot'),
        _limb('right_arm', 'right_', 'shoulder', 'elbow', 'wrist', 'gripper'),
        _limb('right_leg', 'right_', 'hip_yaw', 'hip_roll', 'hip', 'knee', 'ankle', 'ankle_roll',
              'foot')], {
        'look': Routine('{seconds} head={yaw} neck={pitch}',
                        {'seconds': 0.8, 'yaw': 0, 'pitch': 0}),
        'rest': Routine('{seconds} right_shoulder=0 right_elbow=0 left_shoulder=0 '
                        'left_elbow=0 head=0 neck=0', {'seconds': 1.0}),
    }, '0 run=rest'),
    'quad': Type([Subsystem('rotors', 'rotor', ('rotor_fl', 'rotor_fr', 'rotor_rl', 'rotor_rr'))], {
        'take_off': Routine('{seconds} ' + ROTORS.format('{rpm*0.5}') + '\n'
                            '{seconds} ' + ROTORS.format('{rpm}'),
                            {'seconds': 1.0, 'rpm': 3500}),
        'hover': Routine('{seconds} wait=time ' + ROTORS.format('{rpm}'),
                         {'seconds': 2.0, 'rpm': 3000}),
        'yaw': Routine('{seconds} wait=time rotor_fl={high} rotor_rr={high} rotor_fr={low} '
                       'rotor_rl={low}',
                       {'seconds': 1.0, 'high': 3200, 'low': 2800}),
        'pitch': Routine('{seconds} wait=time rotor_fl={front} rotor_fr={front} '
                         'rotor_rl={rear} rotor_rr={rear}', {'seconds': 1.0, 'front': 2800, 'rear': 3200}),
        'land': Routine('{seconds} ' + ROTORS.format('{rpm}') + '\n{seconds} ' +
                        ROTORS.format('0'), {'seconds': 1.5, 'rpm': 1500}),
    }, '0 run=land'),
    'fixed_wing': Type([Subsystem('propulsion', 'rotor', ('throttle',)),
                        Subsystem('surfaces', 'surface',
                                  ('aileron_l', 'aileron_r', 'elevator', 'rudder'))], {
        'take_off': Routine('{seconds} throttle={rpm} elevator=0\n'
                            '{seconds} wait=time throttle={rpm} elevator={-pitch}',
                            {'seconds': 1.5, 'rpm': 5000, 'pitch': 10}),
        'cruise': Routine('{seconds} wait=time throttle={rpm} elevator=0 aileron_l=0 '
                          'aileron_r=0 rudder=0',
                          {'seconds': 2.0, 'rpm': 4000}),
        'bank': Routine('{seconds} wait=time aileron_l={deg} aileron_r={-deg} '
                        'rudder={deg*0.3}\n'
                        '{seconds} aileron_l=0 aileron_r=0 rudder=0',
                        {'seconds': 1.5, 'deg': 15}),
        'land': Routine('{seconds} wait=time throttle={rpm} elevator={pitch}\n'
                        '{seconds} throttle=0',
                        {'seconds': 2.0, 'rpm': 2000, 'pitch': 5}),
    }, '0 run=land'),
    'ebike': Type([Subsystem('drive', 'torque', ('assist',))], {
        'assist': Routine('{seconds} wait=time assist={amps}', {'seconds': 5.0, 'amps': 2.5}),
        'coast': Routine('{seconds} assist=0', {'seconds': 1.0}),
    }, '0 run=coast'),
}
