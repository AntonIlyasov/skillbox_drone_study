import numpy as np

# import math

# def deg2rad(degrees):
#     radians = degrees * (math.pi / 180)
#     return radians

# def rad2deg(radians):
#     degrees = radians * (180 / math.pi)
#     return degrees

# def _rotation_matrix_3d(roll, pitch, yaw):
#     sr = np.sin(roll)
#     cr = np.cos(roll)
#     sp = np.sin(pitch)
#     cp = np.cos(pitch)
#     sy = np.sin(yaw)
#     cy = np.cos(yaw)

#     Rx = np.array([[1,  0,   0],
#                     [0, cr, -sr],
#                     [0, sr, cr]])
    
#     Ry = np.array([[cp,  0, sp],
#                     [0,  1,  0],
#                     [-sp,  0, cp]])

#     Rz = np.array([[cy, -sy, 0],
#                     [sy,  cy, 0],
#                     [0,   0, 1]])
    
#     R = Rz@Ry@Rx
#     return R


# _motor_trust = np.array([deg2rad(45),
#                         deg2rad(45),
#                         0.0])

# rot_mat = _rotation_matrix_3d(0, deg2rad(45), deg2rad(45))

# X_res = rot_mat@_motor_trust

# X = rot_mat.T@X_res

# print(rad2deg(X_res))
# print(rad2deg(X))

quadcopter_inertia = np.array([[5.82857e-04, 0.0, 0.0], 
              				   [0.0, 7.16914e-04, 0.0], 
              				   [0.0, 0.0, 0.01]])

_inertia_inv = np.linalg.inv(quadcopter_inertia)
print(_inertia_inv)








