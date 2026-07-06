import xml.etree.ElementTree as ET
import numpy as np

def rpy_to_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

tree = ET.parse('urdf/robot_esr_v2_rev.urdf')
root = tree.getroot()

joints = {}
for j in root.findall('joint'):
    name = j.get('name')
    parent = j.find('parent').get('link')
    child = j.find('child').get('link')
    orig = j.find('origin')
    xyz = np.array([float(x) for x in orig.get('xyz').split()]) if orig is not None and orig.get('xyz') else np.zeros(3)
    rpy = np.array([float(x) for x in orig.get('rpy').split()]) if orig is not None and orig.get('rpy') else np.zeros(3)
    joints[child] = {'parent': parent, 'xyz': xyz, 'rpy': rpy, 'name': name}

def get_transform(link_name):
    if link_name not in joints:
        return np.eye(3), np.zeros(3)
    j = joints[link_name]
    R_p, t_p = get_transform(j['parent'])
    R_local = rpy_to_matrix(*j['rpy'])
    t_local = j['xyz']
    R_new = R_p @ R_local
    t_new = t_p + R_p @ t_local
    print(f"Link: {link_name} (via {j['name']}) -> Z={t_new[2]:.4f}")
    return R_new, t_new

print("Computing FK for right leg:")
R, t = get_transform('right_knee_pitch_6')
R, t2 = get_transform('right_knee_roll_6')

print(f"\nLowest ankle joint global Z: {t[2]:.4f}")
print(f"Lowest foot joint global Z: {t2[2]:.4f}")

