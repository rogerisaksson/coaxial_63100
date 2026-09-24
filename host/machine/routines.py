"""Machine types: the actuators a type wants, and the routines a program calls by name.

An actuator is wanted as (name, kind) and lands on the next free node that offers the kind;
(None, kind) is every node that offers it, named where it sits. A routine is lines of the
program grammar with {param} in them, `{-p}` its negative and `{p*k}` scaled; a line calls
it: `0 run=walk times=4 stride=25`. No labels inside: a routine repeats by `times`.
Add a type: TYPES['exoskeleton'] = Type([('hip', 'joint'), ..], {...}).
"""
from collections import namedtuple

#: Lines with {param}s, and every param's default.
Routine = namedtuple('Routine', 'text defaults')

#: `actuators` [(name, kind)]; `routines` {name: Routine}.
Type = namedtuple('Type', 'actuators routines')


LEGS_ZERO = 'left_hip=0 right_hip=0 left_knee=0 right_knee=0 left_ankle=0 right_ankle=0'
ROTORS = 'rotor_fl={0} rotor_fr={0} rotor_rl={0} rotor_rr={0}'

TYPES = {
    'humanoid': Type([(None, 'joint')], {
        'stand': Routine('{seconds} ' + LEGS_ZERO, {'seconds': 1.0}),
        'squat': Routine(
            '{seconds} left_hip={-hip} right_hip={-hip} left_knee={knee} right_knee={knee} '
            'left_ankle={-ankle} right_ankle={-ankle}\n{seconds} ' + LEGS_ZERO,
            {'seconds': 1.2, 'hip': 30, 'knee': 60, 'ankle': 30}),
        'walk': Routine(
            '{period} left_hip={-stride} left_knee={lift} right_hip={stride} right_knee=0\n'
            '{period} left_knee=0\n'
            '{period} right_hip={-stride} right_knee={lift} left_hip={stride} left_knee=0\n'
            '{period} right_knee=0',
            {'stride': 20, 'lift': 30, 'period': 0.4}),
        'wave': Routine('{seconds} right_shoulder={raise} right_elbow=45\n'
                        '{seconds} right_shoulder={raise} right_elbow=-20',
                        {'seconds': 0.6, 'raise': 80}),
        'look': Routine('{seconds} head={yaw} neck={pitch}',
                        {'seconds': 0.8, 'yaw': 0, 'pitch': 0}),
        'rest': Routine('{seconds} right_shoulder=0 right_elbow=0 left_shoulder=0 '
                        'left_elbow=0 head=0 neck=0', {'seconds': 1.0}),
    }),
    'quad': Type([(r, 'rotor') for r in ('rotor_fl', 'rotor_fr', 'rotor_rl', 'rotor_rr')], {
        'take_off': Routine('{seconds} ' + ROTORS.format('{rpm*0.5}') + '\n'
                            '{seconds} ' + ROTORS.format('{rpm}'),
                            {'seconds': 1.0, 'rpm': 3500}),
        'hover': Routine('{seconds} ' + ROTORS.format('{rpm}'), {'seconds': 2.0, 'rpm': 3000}),
        'yaw': Routine('{seconds} rotor_fl={high} rotor_rr={high} rotor_fr={low} rotor_rl={low}',
                       {'seconds': 1.0, 'high': 3200, 'low': 2800}),
        'pitch': Routine('{seconds} rotor_fl={front} rotor_fr={front} rotor_rl={rear} '
                         'rotor_rr={rear}', {'seconds': 1.0, 'front': 2800, 'rear': 3200}),
        'land': Routine('{seconds} ' + ROTORS.format('{rpm}') + '\n{seconds} ' +
                        ROTORS.format('0'), {'seconds': 1.5, 'rpm': 1500}),
    }),
    'fixed_wing': Type([('throttle', 'rotor')] + [(s, 'surface') for s in (
        'aileron_l', 'aileron_r', 'elevator', 'rudder')], {
        'take_off': Routine('{seconds} throttle={rpm} elevator=0\n'
                            '{seconds} throttle={rpm} elevator={-pitch}',
                            {'seconds': 1.5, 'rpm': 5000, 'pitch': 10}),
        'cruise': Routine('{seconds} throttle={rpm} elevator=0 aileron_l=0 aileron_r=0 rudder=0',
                          {'seconds': 2.0, 'rpm': 4000}),
        'bank': Routine('{seconds} aileron_l={deg} aileron_r={-deg} rudder={deg*0.3}\n'
                        '{seconds} aileron_l=0 aileron_r=0 rudder=0',
                        {'seconds': 1.5, 'deg': 15}),
        'land': Routine('{seconds} throttle={rpm} elevator={pitch}\n{seconds} throttle=0',
                        {'seconds': 2.0, 'rpm': 2000, 'pitch': 5}),
    }),
    'ebike': Type([('assist', 'torque')], {
        'assist': Routine('{seconds} assist={amps}', {'seconds': 5.0, 'amps': 2.5}),
        'coast': Routine('{seconds} assist=0', {'seconds': 1.0}),
    }),
}
