"""Her movements' terms as static arrays: written by host/tools/cores/terms.py, not by hand."""

#: (part, movement): (joint kind, sign, low, high) - the movement its own way is the
#: joint's angle times sign, deg; low and high its amount's stops, None with none. A
#: movement toward a side (TOWARD) is told for limb 0's: times a limb's mirror.
TERMS = {
    ('hip', 'flexion'): ('hip', -1, None, None),
    ('hip', 'extension'): ('hip', 1, None, None),
    ('hip', 'abduction'): ('hip_roll', 1, None, None),
    ('hip', 'adduction'): ('hip_roll', -1, None, None),
    ('hip', 'lateral-rotation'): ('hip_yaw', 1, -25, 25),
    ('hip', 'medial-rotation'): ('hip_yaw', -1, -25, 25),
    ('knee', 'flexion'): ('knee', 1, -5, 165),
    ('knee', 'extension'): ('knee', -1, -165, 5),
    ('ankle', 'dorsiflexion'): ('ankle', -1, -45, 50),
    ('ankle', 'plantarflexion'): ('ankle', 1, -50, 45),
    ('foot', 'eversion'): ('ankle_roll', 1, -15, 15),
    ('foot', 'inversion'): ('ankle_roll', -1, -15, 15),
    ('trunk', 'flexion'): ('spine', 1, -30, 85),
    ('trunk', 'extension'): ('spine', -1, -85, 30),
    ('trunk', 'lateral-flexion'): ('spine_roll', -1, -35, 35),
    ('trunk', 'rotation'): ('waist', 1, -50, 50),
    ('neck', 'flexion'): ('neck', 1, -45, 60),
    ('neck', 'extension'): ('neck', -1, -60, 45),
    ('head', 'rotation'): ('head', 1, -80, 80),
    ('shoulder', 'flexion'): ('shoulder', 1, None, None),
    ('shoulder', 'extension'): ('shoulder', -1, None, None),
    ('elbow', 'flexion'): ('elbow', 1, -5, 160),
    ('elbow', 'extension'): ('elbow', -1, -160, 5),
}
TOWARD = (('trunk', 'lateral-flexion'), ('trunk', 'rotation'), ('head', 'rotation'),)
#: Movements anatomy names that no driven joint of hers makes.
ABSENT = (
    ('toes', 'extension'), ('toes', 'flexion'), ('neck', 'lateral-flexion'),
    ('shoulder', 'abduction'), ('shoulder', 'adduction'), ('shoulder', 'lateral-rotation'),
    ('shoulder', 'medial-rotation'), ('shoulder', 'elevation'), ('shoulder', 'depression'),
    ('shoulder', 'protraction'), ('shoulder', 'retraction'), ('forearm', 'supination'),
    ('forearm', 'pronation'), ('wrist', 'flexion'), ('wrist', 'extension'),
    ('wrist', 'radial-deviation'), ('wrist', 'ulnar-deviation'), ('fingers', 'flexion'),
    ('fingers', 'extension'), ('thumb', 'opposition'), ('thumb', 'reposition'),
)
#: Each limb's joints by kind: limb 0 the one on +x, as `reach_rows.LIMBS`.
LIMBS = (
    {'hip': 'left_hip', 'hip_roll': 'left_hip_roll', 'hip_yaw': 'left_hip_yaw', 'knee': 'left_knee',
     'ankle': 'left_ankle', 'ankle_roll': 'left_ankle_roll', 'shoulder': 'left_shoulder', 'elbow': 'left_elbow'},
    {'hip': 'right_hip', 'hip_roll': 'right_hip_roll', 'hip_yaw': 'right_hip_yaw', 'knee': 'right_knee',
     'ankle': 'right_ankle', 'ankle_roll': 'right_ankle_roll', 'shoulder': 'right_shoulder', 'elbow': 'right_elbow'},
)
