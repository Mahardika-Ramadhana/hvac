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
    joints[name] = {'xyz': xyz, 'R': rpy_to_matrix(*rpy)}

chain = ['root_base_fixed_joint', 'right_knee_yaw_2', 'right_knee_roll_2', 'right_knee_pitch_2', 'right_knee_pitch_4', 'right_knee_pitch_6', 'right_knee_roll_6']
current_pos = np.zeros(3)
current_R = np.eye(3)

for c in chain:
    if c in joints:
        j = joints[c]
        current_pos = current_pos + current_R @ j['xyz']
        current_R = current_R @ j['R']

R_cnoid = np.array([
    [0, -1, 0],
    [1, 0, 0],
    [0, 0, 1]
])

# Get foot vertices
verts = get_stl_vertices('meshes/right_knee_roll_6.STL')
# Transform vertices to base_link frame
verts_base = (current_R @ verts.T).T + current_pos
# Transform vertices to world frame (assuming base_link is at 0,0,0)
verts_world = (R_cnoid @ verts_base.T).T

min_z = np.min(verts_world[:, 2])
print(f"Lowest Z point of foot relative to base_link origin (in world orientation): {min_z}")
print(f"If spawn at Z=0.360, lowest foot point is: {0.360 + min_z}")
