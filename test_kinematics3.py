import xml.etree.ElementTree as ET
import numpy as np
import struct

def rpy_to_matrix(r, p, y):
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def get_stl_vertices(filename):
    verts = []
    with open(filename, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        for _ in range(num_triangles):
            normal = struct.unpack('<3f', f.read(12))
            for _ in range(3):
                verts.append(list(struct.unpack('<3f', f.read(12))))
            f.read(2)
    return np.array(verts)

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
    axis = j.find('axis')
    axis_vec = np.array([float(x) for x in axis.attrib.get('xyz', '0 0 1').split()]) if axis is not None else np.array([0,0,1])
    joints[name] = {'xyz': xyz, 'R0': rpy_to_matrix(*rpy), 'axis': axis_vec}

# initialJointDisplacements from .cnoid for right leg:
# right_hip_yaw: 0
# right_hip_roll: 0
# right_hip_pitch: -0.04
# right_knee_pitch: 0.04
# right_ankle_pitch: 0.08
# right_ankle_roll: 0
q_values = {
    'right_knee_yaw_2': 0,          # index 4 (0-based)
    'right_knee_roll_2': 0,         # index 5
    'right_knee_pitch_2': -0.04,    # index 6
    'right_knee_pitch_4': 0.04,     # index 7
    'right_knee_pitch_6': 0.08,     # index 8
    'right_knee_roll_6': 0          # index 9
}

# The axis of rotation depends on the joint type. For continuous/revolute, it's a rotation.
def axis_angle_to_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

chain = ['root_base_fixed_joint', 'right_knee_yaw_2', 'right_knee_roll_2', 'right_knee_pitch_2', 'right_knee_pitch_4', 'right_knee_pitch_6', 'right_knee_roll_6']
current_pos = np.zeros(3)
current_R = np.eye(3)

for c in chain:
    if c in joints:
        j = joints[c]
        q = q_values.get(c, 0)
        # Joint rotation
        R_joint = axis_angle_to_matrix(j['axis'], q)
        
        # Translate by origin
        current_pos = current_pos + current_R @ j['xyz']
        # Rotate by origin rpy, then by joint angle
        current_R = current_R @ (j['R0'] @ R_joint)

R_cnoid = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1]
])

verts = get_stl_vertices('meshes/right_knee_roll_6.STL')
verts_base = (current_R @ verts.T).T + current_pos
verts_world = (R_cnoid @ verts_base.T).T

min_z = np.min(verts_world[:, 2])
print(f"Lowest Z with initialJointDisplacements: {min_z}")
