"""The humanoid: twenty nodes, one body, a program a small model writes."""
from .parts import code, md, section

TITLE = 'The humanoid'
SUMMARY = 'Every board a node, every node a joint; a small model reads a short card and answers with lines the sequencer runs.'

SECTIONS = [
    section(
        'The nodes',
        md('`Nodes.discover` asks every bus for its nodes and opens each on its own; a node is '
           'named where it sits. What each offers comes before anything is configured.'),
        code('''from coaxial.nodes import Nodes

nodes = Nodes.discover(port=PORT, device=SIMULATED)
print('%d nodes: %s' % (len(nodes), ', '.join(n.name for n in nodes)))
print(nodes.card('angle', 'drive', keys=('degrees', 'omega_hat', 'theta', 'iq_ref')).split('\\n')[:3])
offered = len(nodes.capabilities())
print(offered, 'channels in all')'''),
    ),
    section(
        'The body, and what a model is told',
        md('`Body`: a joint feedback per node - target deg, a slew, the drive holding the angle, '
           'read back as `<joint>.deg`. `prompt()` is the grammar and one line for every joint.'),
        code('''from coaxial.control.body import Body

body = Body(nodes)
told = body.prompt()
print(told)
print('%d characters, about %d tokens' % (len(told), len(told) / 4))'''),
    ),
    section(
        'A program, as a model answers',
        md('Asked for two squats and three waves, a model answers with lines: a counter `n`, '
           'a decision row that loops back until `n` runs out, `cleanup` to stand.'),
        code('''PROGRAM = """
# squat twice, then wave the right arm three times
0.3 group=init left_hip=0 right_hip=0 left_knee=0 right_knee=0
1.5 label=down left_hip=-30 right_hip=-30 left_knee=60 right_knee=60 left_knee.deg.H=55
1 left_hip=0 right_hip=0 left_knee=0 right_knee=0 goto=down times=1
0 n=3
0.6 label=wave right_shoulder=80 right_elbow=45
0.6 right_elbow=-20
0 n+=-1 n.L=0 then=rest else=wave
1 label=rest right_shoulder=0 right_elbow=0
1 group=cleanup left_hip=0 right_hip=0 left_knee=0 right_knee=0 right_shoulder=0
"""
out = body.run(PROGRAM)
print(out.summary('left_knee.deg', 'right_shoulder.deg', 'n'))'''),
        code('''from coaxial.draw.figures import figure, show

t = [r['t'] for r in out.rows]
fig, (legs, arm) = figure(rows=2, sharex=True)
for joint in ('left_hip', 'left_knee'):
    legs.plot(t, [r[joint + '.deg'] for r in out.rows], label=joint)
legs.set_ylabel('deg')
legs.legend(loc='upper right')
for joint in ('right_shoulder', 'right_elbow'):
    arm.plot(t, [r[joint + '.deg'] for r in out.rows], label=joint)
arm.set_ylabel('deg')
arm.set_xlabel('s')
arm.legend(loc='upper right')
show(fig)'''),
    ),
    section(
        'When the model is wrong',
        md('Checked before anything arms: a near miss comes back as the name meant, a target '
           'outside the range is refused - one line for the model to try again with.'),
        code('''from coaxial.errors import RigError

refusals = []
for wrong in ('1 left_kne=40', '1 left_knee=120', '1 left_knee=40 left_knee.degg.H=30'):
    try:
        body.run(wrong)
    except RigError as exc:
        refusals.append(str(exc).splitlines()[-1])
        print('%-36s -> %s' % (wrong, refusals[-1]))'''),
    ),
    section(
        'Through the board chat',
        md('The `program` tool is what the local model calls: `op=card`, then `op=run` with its '
           'answer. The same on a stand-in session, as the model sees it.'),
        code('''from coaxial.simulated.board import SimulatedSession
from coaxial_mcp.tools import program

session = SimulatedSession(port='AX')
card = program(session, op='card')
answer = program(session, op='run', text='0.5 left_knee=30 right_knee=30\\n0.5 left_knee=0 right_knee=0')
print(answer)
session.body.nodes.close()
nodes.close()'''),
    ),
]

RESULTS = [
    code('''print('1. nodes      %d, one body; %d channels offered' % (len(nodes), offered))
print('2. the prompt %d characters for %d joints' % (len(told), len(body.joints)))
print('3. the run    %s, %d steps, %.1f s' % (out.status, len(out.steps), sum(s[3] for s in out.steps)))
print('4. refused    %d of 3, each in a line' % len(refusals))'''),
    md('- The model writes lines, not code: the sequencer is the only thing that runs.\n'
       '- A wrong answer costs one line back; a joint past its range trips; cleanup always '
       'runs.'),
]

BENCH = ('Discover on each bus through the broker; arming empty (the STO chain released); '
         'each joint commissioned and its `span` set before a model drives it.')

REFERENCES = [
    ('host/coaxial/nodes/', '`Node`, `Nodes`: boards as IO nodes, capabilities first'),
    ('host/coaxial/control/body.py', '`Body`: a joint feedback per node; `prompt`, `run`'),
    ('host/coaxial/control/sequencer.py', 'lines or tables; `check`, `summary`, `GRAMMAR`'),
    ('host/coaxial_mcp/tools.py', '`program`: the board-chat tool'),
]
