import xml.etree.ElementTree as ET
import numpy as np

def rpy_to_matrix(r, p, y):
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

tree = ET.parse('urdf/robot_esr_v2_rev.urdf')
root = tree.getroot()

joints = {}
for j in root.findall('joint'):
    name = j.attrib['name']
    orig = j.find('origin')
    if orig is not None:
        xyz = np.array([float(x) for x in orig.attrib.get('xyz', '0 0 0').split()])
        rpy = np.array([float(x) for x in orig.attrib.get('rpy', '0 0 0').split()])
    else:
        xyz = np.zeros(3)
        rpy = np.zeros(3)
    joints[name] = {'xyz': xyz, 'R': rpy_to_matrix(*rpy)}

# Calculate forward kinematics down the right leg
# Chain from root to foot
chain = [
    'root_base_fixed_joint',
    'right_knee_yaw_2',
    'right_knee_roll_2',
    'right_knee_pitch_2',
    'right_knee_pitch_4',
    'right_knee_pitch_6',
    'right_knee_roll_6'
]

current_pos = np.zeros(3)
current_R = np.eye(3)

print("Forward Kinematics (Zero joint angles):")
for c in chain:
    if c in joints:
        j = joints[c]
        # Translate
        current_pos = current_pos + current_R @ j['xyz']
        # Rotate
        current_R = current_R @ j['R']
        print(f"After {c}:")
        print(f"  Pos: {current_pos}")

print(f"\nFinal Right Foot Pos relative to base_link: {current_pos}")

# Apply initialRootAttitude from .cnoid
R_cnoid = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1]
])
world_pos = R_cnoid @ current_pos
print(f"\nFoot Pos in World (if spawn at 0,0,0): {world_pos}")
