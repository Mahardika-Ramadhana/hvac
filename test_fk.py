import xml.etree.ElementTree as ET
import numpy as np

def R_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[1,0,0],[0,c,-s],[0,s,c]])

def R_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])

def R_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def rpy_matrix(r, p, y):
    return R_z(y) @ R_y(p) @ R_x(r)

joints = [
    # Right leg
    ("right_knee_yaw_2", [0.1233, -0.0797, 0.3246], [1.5708, 0, 0], [0,-1,0], 0.0),
    ("right_knee_roll_2", [0, 0, 0], [0, 0, 0], [-1,0,0], 0.0),
    ("right_knee_pitch_2", [0.03, 0, -0.0755], [0,0,0], [1,0,0], -0.04),
    ("right_knee_pitch_4", [0.009, -0.2982, 0.0313], [0,0,0], [1,0,0], 0.04),
    ("right_knee_pitch_6", [0.015, -0.3204, 0.032], [0,0,0], [1,0,0], 0.0), # WAS 0.08
    ("right_knee_roll_6", [-0.054, -0.083, -0.0118], [0,0,0], [-1,0,0], 0.0),
]

T = np.eye(4)
for name, xyz, rpy, axis, q in joints:
    T_local = np.eye(4)
    T_local[:3,3] = xyz
    T_local[:3,:3] = rpy_matrix(*rpy)
    
    T_joint = np.eye(4)
    # Rotation around axis
    ax = np.array(axis, dtype=float)
    ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    R = np.eye(3) + np.sin(q)*K + (1-np.cos(q))*(K@K)
    T_joint[:3,:3] = R
    
    T = T @ T_local @ T_joint

# World transformation: root pitch 0
# World Z is URDF Z
print(f"Foot link origin Z relative to base_link: {T[2,3]}")

# Lowest point of foot
lowest_z = 1000
import trimesh
mesh = trimesh.load('meshes/right_knee_roll_6.STL')
for v in mesh.vertices:
    p = np.array([v[0], v[1], v[2], 1.0])
    pw = T @ p
    if pw[2] < lowest_z:
        lowest_z = pw[2]
print(f"Lowest point of foot Z: {lowest_z}")
