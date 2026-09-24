"""The machines: every board a node, a type over them, programs a model writes and streams."""
from .parts import code, md, section

TITLE = 'The machines'
SUMMARY = 'Every board a node; a humanoid, a quad, a fixed wing, an e-bike over them; programs a model writes, streamed live behind a failsafe.'

SECTIONS = [
    section(
        'The nodes',
        md('`Nodes.discover` loads each installed board family - here `coaxial.node` - which finds '
           'its boards on every bus; then the robot\'s other boards, a battery pack and a head '
           'camera. Each is a node named where it sits, asked what it offers first.'),
        code('''from machine.nodes import Nodes

nodes = Nodes.discover(port=PORT, simulated=SIMULATED)
kinds = sorted({n.type for n in nodes})
print('%d nodes: %s' % (len(nodes), ', '.join('%d %s' % (len(nodes.of_type(k)), k) for k in kinds)))
print(Nodes([nodes['battery'], nodes['head_camera']]).card())
offered = len(nodes.capabilities())
print(offered, 'channels in all')'''),
    ),
    section(
        'A humanoid, and what a model is told',
        md('`Machine(nodes, type=\'humanoid\')`: a joint on every node that offers one, the pack '
           'and the camera read beside them; `Machine.discover(\'humanoid\', port=PORT)` is both '
           'steps. `prompt()` is the grammar, the joints, what can be read, the routines.'),
        code('''from machine import Machine

humanoid = Machine(nodes, type='humanoid')
told = humanoid.prompt()
print(told)
print('%d characters, about %d tokens' % (len(told), len(told) / 4))'''),
    ),
    section(
        'A program, as a model answers',
        md('Routines by name, and a search: the head turns 10 deg at a time until the camera '
           'sees the target, then 2 deg at a time until it is centred - a closed loop, as data.'),
        code('''PROGRAM = """
# squat twice, wave three times, then find the target and centre it
0 run=squat times=2
0 run=wave times=3
0 run=rest
0.3 label=scan head+=10 head_camera.vision.target.seen.H=1 then=aim
0 goto=scan times=6
0.2 label=aim head+=2 head_camera.vision.target.x.L=0.05 then=found
0 goto=aim times=20
0 label=found
0 run=stand
"""
out = humanoid.run(PROGRAM)
print(out.summary('head.deg', 'head_camera.vision.target.x', 'battery.pack.volts'))'''),
        code('''from coaxial.draw.figures import figure, show

t = [r['t'] for r in out.rows]
fig, (legs, head) = figure(rows=2, sharex=True)
for joint in ('left_hip', 'left_knee', 'right_shoulder'):
    legs.plot(t, [r[joint + '.deg'] for r in out.rows], label=joint)
legs.set_ylabel('deg')
legs.legend(loc='upper right')
head.plot(t, [r['head.deg'] for r in out.rows], label='head, deg')
head.plot(t, [30 * r['head_camera.vision.target.x'] for r in out.rows], label='target off centre, deg')
head.set_xlabel('s')
head.legend(loc='upper left')
show(fig)'''),
    ),
    section(
        'When the model is wrong',
        md('Checked before anything arms, one line back: the routine meant, a parameter it does '
           'not take, a target out of range, a value that is not a number.'),
        code('''from machine.errors import MachineError

refusals = []
for wrong in ('0 run=wlak', '0 run=walk strides=3', '0 run=squat knee=120', '0.5 head=4O'):
    try:
        humanoid.run(wrong)
    except MachineError as exc:
        refusals.append(str(exc).splitlines()[-1])
        print('%-22s -> %s' % (wrong, refusals[-1]))'''),
    ),
    section(
        'Other machines, the same nodes',
        md('A type is data: the actuators it wants by kind, and its routines. The quad\'s rotors '
           'are speed loops (rpm), the fixed wing\'s surfaces narrow joints, the e-bike\'s '
           'assist a current.'),
        code('''RUNS = {'quad': '0 run=take_off\\n0 run=hover seconds=1\\n0 run=yaw\\n0 run=land',
        'fixed_wing': '0 run=take_off\\n0 run=bank deg=-15\\n0 run=land',
        'ebike': '0 run=assist amps=3 seconds=2\\n0 run=coast'}
others = {}
for kind, program in RUNS.items():
    machine = Machine(nodes, type=kind)
    print(machine.prompt().splitlines()[-1])
    others[kind] = machine.run(program)
    print('  %s' % others[kind].summary().splitlines()[0])'''),
    ),
    section(
        'Live: a stream, a buffer, a failsafe',
        md('`Live` plays chunks as they arrive, from a buffer of at most `horizon` seconds; '
           'silence past `timeout` plays the failsafe. Here the model streams, stumbles once, '
           'then goes quiet.'),
        code('''import time
from machine.live import Live

live = Live(humanoid, failsafe='0 run=stand seconds=0.8', timeout=1.0, horizon=2.0).start()
t0 = time.monotonic()
for chunk in ('0.5 left_knee=20 right_knee=20', '0 run=walk stride=15', '0.5 head=4O',
              '0 run=walk stride=15', '0 run=walk stride=15'):
    said = live.send(chunk)
    print('%4.1f s  %-30s %s' % (time.monotonic() - t0, chunk, said or 'queued'))
    time.sleep(0.4)
time.sleep(4.0)
streamed = live.stop()
print(streamed)'''),
    ),
    section(
        'Through the board chat',
        md('The `program` tool is what the local model calls: `op=card`, then `op=run` with its '
           'answer, on any type.'),
        code('''from coaxial.simulated.board import SimulatedSession
from coaxial_mcp.tools import program

session = SimulatedSession(port='AX')
print(program(session, op='run', machine='quad', text='0 run=take_off\\n0 run=land'))
session.nodes.close()
nodes.close()'''),
    ),
]

RESULTS = [
    code('''print('1. nodes      %d; %d channels offered' % (len(nodes), offered))
print('2. the prompt %d characters for %d joints and their routines' % (len(told), len(humanoid.actuators)))
print('3. the run    %s, %d steps; the target %.2f of the half field off centre'
      % (out.status, len(out.steps), out.rows[-1]['head_camera.vision.target.x']))
print('4. refused    %d of 4, each in a line' % len(refusals))
print('5. others     %s' % ', '.join('%s %s' % (k, o.status) for k, o in others.items()))
print('6. live       %s after %s; %d chunks played, worst wait %.2f s'
      % (streamed['status'], streamed['reason'], streamed['played'], streamed['wait_max']))'''),
    md('- The model writes lines and names routines, not code: the sequencer runs, the loop '
       'holds.\n'
       '- A wrong line costs one line back; silence costs the failsafe; a limit trips.'),
]

BENCH = ('Discover through the broker; arming empty (the STO chain released); every joint '
         'commissioned and its span set; the failsafe a routine that is safe from anywhere.')

REFERENCES = [
    ('host/machine/nodes.py', '`Node`, `Nodes`, `FAMILIES`: boards as IO nodes, capabilities first'),
    ('host/machine/machine.py', '`Machine`: actuators over nodes; `discover`, `prompt`, `run`'),
    ('host/coaxial/node.py', 'the Coaxial63100 family: its node, joint, surface, rotor, torque'),
    ('host/machine/routines.py', 'the types and their routines'),
    ('host/machine/live.py', '`Live`: chunks, a buffer, a watchdog, a failsafe'),
    ('host/machine/sequencer.py', 'lines or tables; `check`, `summary`, `GRAMMAR`'),
    ('host/coaxial_mcp/tools.py', '`program`: the board-chat tool'),
]
