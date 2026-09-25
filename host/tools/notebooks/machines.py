"""The machines: every board a node, a type over them, programs a model writes and streams."""
from .parts import code, md, section

TITLE = 'The machines'
SUMMARY = 'Every board a node; a humanoid, a quad, a fixed wing, an e-bike over them; programs a model writes, streamed live behind a failsafe.'

SECTIONS = [
    section(
        'The nodes',
        md('`Nodes.discover` loads each installed board family - here `coaxial.node` - which finds '
           'its boards on every bus, each a node named by its bus and unit (`LL_2`); then the '
           'robot\'s other boards, a battery pack and a head camera. Each is asked what it '
           'offers first.'),
        code('''from machine.nodes import Nodes

nodes = Nodes.discover(port=PORT, execution_mode=MODE)
kinds = sorted({n.type for n in nodes})
print('%d nodes: %s' % (len(nodes), ', '.join('%d %s' % (len(nodes.of_type(k)), k) for k in kinds)))
print(Nodes([nodes['battery'], nodes['head_camera']]).card())
offered = len(nodes.capabilities())
print(offered, 'channels in all')'''),
    ),
    section(
        'A humanoid, and what a model is told',
        md('`Machine(nodes, type=\'humanoid\')`: a limb a bus, bus 1 the axis. Which board is '
           'which joint is measured: a ring test on each, the lowest ring innermost. The pack '
           'and camera are read beside; `prompt()` is what a model is told.'),
        code('''from machine import Machine

humanoid = Machine(nodes, type='humanoid')
for name in ('left_hip', 'left_knee', 'left_ankle', 'left_foot'):
    board = humanoid.actuators[name].node
    ring = board.identify()
    print('%-10s %s  %.1f Hz  J %.2g' % (name, board.name, ring['hz'], ring['j']))
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
0.3 label=scan head+=10 head_camera.vision.target.seen.GE=1 then=aim
0 goto=scan times=6
0.2 label=aim head+=2 head_camera.vision.target.x.LE=0.05 then=found
0 goto=aim times=20
0 label=found
0 run=stand
"""
out = humanoid.run(PROGRAM)
print(out.summary('head.deg', 'head_camera.vision.target.x', 'battery.pack.volts'))'''),
        code('''from coaxial.draw.figures import figure, show

t = [r['t'] for r in out.rows]
half = nodes['head_camera'].identity['fov'] / 2            # deg
fig, (joints, head) = figure(rows=2, sharex=True)
for joint in ('left_hip', 'left_knee', 'right_shoulder'):
    joints.plot(t, [r[joint + '.deg'] for r in out.rows], label=joint)
joints.set_ylabel('deg')
joints.legend(loc='upper right')
head.plot(t, [r['head.deg'] for r in out.rows], label='head, deg')
head.plot(t, [half * r['head_camera.vision.target.x'] for r in out.rows], label='target off centre, deg')
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
        code('''from machine.sequencer import card

RUNS = {'quad': '0 run=take_off\\n0 run=hover seconds=1\\n0 run=yaw\\n0 run=land',
        'fixed_wing': '0 run=take_off\\n0 run=bank deg=-15\\n0 run=land',
        'ebike': '0 run=assist amps=3 seconds=2\\n0 run=coast'}
others = {}
for kind, steps in RUNS.items():
    craft = Machine(nodes, type=kind)
    print('%s, routines %s' % (kind, ', '.join(craft.routines)))
    print(card(craft.loop, craft.units, craft.ranges))
    others[kind] = craft.run(steps)
    print('  %s' % others[kind].summary().splitlines()[0])'''),
    ),
    section(
        'Live: a line at a time, woken with a line',
        md('`feed` takes the answer as the model writes it: each finished line is checked and '
           'plays at once. `wait` returns one line when the buffer runs low or something '
           'happens, with what changed. Silence plays the type\'s failsafe.'),
        code('''import time
from machine.live import Live

ANSWER = ('0.5 left_knee=20 right_knee=20\\n0 run=walk stride=15 period=0.25\\n'
          '0 run=walk stride=15 period=0.25\\n')
CPS = 160.0                                  # 40 tokens a second, 4 characters a token


def stream(live, fed):
    """Seconds from the answer's first token to the first move."""
    t0, first = time.monotonic(), None
    for i in range(0, len(ANSWER), 4):
        time.sleep(4 / CPS)
        if fed:
            live.feed(ANSWER[i:i + 4])
        if first is None and live.buffered() > 0:
            first = time.monotonic() - t0
    if not fed:
        live.send(ANSWER)
        first = time.monotonic() - t0
    return first


live = Live(humanoid, timeout=1.0, horizon=2.5).start()
first = {'whole': stream(live, fed=False)}
print(live.wait(low=0.0, timeout=6.0))
first['fed'] = stream(live, fed=True)
woke = live.wait(low=0.3, timeout=6.0)
print(woke)
print(live.feed('0.5 head=4O\\n'))
print(live.wait(low=-1.0, timeout=6.0))
streamed = live.stop()
print('first move %.2f s fed, %.2f s whole; woken in %d tokens'
      % (first['fed'], first['whole'], len(woke) / 4))'''),
    ),
    section(
        'Through the board chat',
        md('The `program` tool is what the local model calls on its session: `card` once, `run` '
           'a whole program (its summary back), or live - `start`, `send`, `wait`, `stop` - '
           'each reply a line.'),
        code('''from coaxial import Coaxial63100
from coaxial_mcp.tools import close_programs, program

rig = Coaxial63100(port=PORT, execution_mode=MODE).open()
session = rig.session
print(program(session, op='run', machine='quad', text='0 run=take_off\\n0 run=land'))
calls = [('start', ''), ('send', '0 run=take_off seconds=0.5\\n0 run=hover seconds=0.8'),
         ('wait', ''), ('stop', '')]
replies = [program(session, op=op, machine='quad', text=text) for op, text in calls]
for (op, _), reply in zip(calls, replies):
    print('%-5s %3d tokens  %s' % (op, len(reply) / 4, reply[:70]))
close_programs(session)
rig.close()
nodes.close()'''),
    ),
]

RESULTS = [
    code('''print('1. nodes      %d; %d channels offered' % (len(nodes), offered))
print('2. the prompt %d characters for %d joints and their routines' % (len(told), len(humanoid.actuators)))
print('3. the run    %s, %d steps; the target %.2f of the half field off centre'
      % (out.status, len(out.steps), abs(out.rows[-1]['head_camera.vision.target.x'])))
print('4. refused    %d of 4, each in a line' % len(refusals))
print('5. others     %s' % ', '.join('%s %s' % (k, o.status) for k, o in others.items()))
print('6. live       first move %.2f s fed, %.2f s sent whole; woken in %d tokens; %s'
      % (first['fed'], first['whole'], len(woke) / 4, streamed['status']))
print('7. the chat   %s' % ', '.join('%s %d' % (op, len(r) / 4) for (op, _), r in zip(calls, replies)))'''),
    md('- The model writes lines and names routines, not code: the sequencer runs, the loop '
       'holds.\n'
       '- A wrong line costs one line back; silence costs the failsafe; a limit trips.\n'
       '- A line moves the machine once written; a wait costs a line: what happened, what moved.'),
]

BENCH = ('Discover through the broker, the pack and camera given as `peripherals`; arming '
         'empty (the STO chain released); every joint commissioned and its span set; the '
         'failsafe a routine that is safe from anywhere.')

REFERENCES = [
    ('host/machine/nodes.py', '`Node`, `Nodes`, `FAMILIES`: boards as IO nodes, capabilities first'),
    ('host/machine/machine.py', '`Machine`: actuators over nodes; `discover`, `prompt`, `run`'),
    ('host/coaxial/node.py', 'the Coaxial63100 family: its node, joint, surface, rotor, torque'),
    ('host/machine/routines.py', 'the types and their routines'),
    ('host/machine/live.py', '`Live`: chunks, a buffer, a watchdog, a failsafe'),
    ('host/machine/sequencer.py', 'lines or tables; `check`, `summary`, `card`, `GRAMMAR`'),
    ('host/coaxial_mcp/tools.py', '`program`, `close_programs`: the board-chat tool'),
    ('host/coaxial/rig.py', '`Coaxial63100`: the board in its execution mode'),
]
