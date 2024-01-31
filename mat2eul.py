import subprocess
import numpy as np
import transforms3d as t3d
from scipy.spatial.transform import Rotation  
import tf
from copy import deepcopy
import math


def execute(cmd, blocking=True, verbose=True):
    """ @brief Executes the command in the shell in a blocking or non-blocking manner
        @param cmd a string with teh command to execute
        @return
    """
    if verbose:
        print("Executing command: " + cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if blocking:  # if blocking is True:
        for line in p.stdout.readlines():
            if verbose:
                print
                line,
            p.wait()

# Example 4x4 transformation matrix (replace this with your actual matrix)
# transform_matrix = np.array([[0.581802,-0.522479,-0.623315,20.7373],
#                              [0.813015,0.394935,0.427823,10.6052],
#                              [0.0226401,-0.755673,0.654557,1.44539],
#                              [0, 0, 0, 1]])

# 7bb4f4f887354eb1948dc229515574fc_i0_0.jpg
# transform_matrix = np.array([[0.686372, -0.49883, -0.529208, 22.0124],
#                             [-0.727223, -0.464475, -0.505381, 9.0375],
#                             [0.00629532, 0.731731, -0.681564, 1.45343],
#                             [0,0,0,1]])

# 7bb4f4f887354eb1948dc229515574fc_i0_1.jpg
# transform_matrix = np.array([[-0.286486, -0.646401, -0.707171, 22.0158],
#                             [-0.958053, 0.18732, 0.2169, 9.02334],
#                             [-0.00462435, 0.751655, -0.65954, 1.45299],
#                             [0,0,0,1]])

# 7bb4f4f887354eb1948dc229515574fc_i0_2.jpg
transform_matrix = np.array([[-0.972955, -0.155704, -0.170628, 22.0052],
                            [-0.230946, 0.640914, 0.732047, 9.01336],
                            [-0.00462435, 0.751655, -0.65954, 1.45299],
                            [0,0,0,1]])

opengl_to_opencv = np.array([[0,-1,0,0],
                            [0,0,1,0],
                            [-1,0,0,0],
                            [0,0,0,1]])
# fros_T_fmp3d = np.identity(4)


composed_trans = np.dot(transform_matrix, opengl_to_opencv)

# Extract translation (x, y, z)
translation = composed_trans[:3, 3]
x, y, z = translation

# Extract rotation as a matrix
rotation_matrix = composed_trans[:3, :3]
print(rotation_matrix)

angles = tf.transformations.euler_from_matrix(rotation_matrix, axes = 'sxyz')
print(angles)

# r = Rotation.from_matrix(rotation_matrix)
# angles_scipy = r.as_euler('xyz', degrees=False)
# print(angles_scipy)

# yaw = math.atan2(rotation_matrix[2, 1], rotation_matrix[1, 1])
# pitch = math.atan2(-rotation_matrix[2, 0], math.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))
# roll = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
# print(f'{yaw}, {pitch}, {roll}')

execute(cmd=f'roslaunch synfeal_bringup bringup_camera.launch x_pos:={translation[0]} y_pos:={translation[1]} z_pos:={translation[2]} R_pos:={angles[0]} P_pos:={angles[1]} Y_pos:={angles[2]}')

# execute(cmd=f'roslaunch synfeal_bringup bringup_camera.launch x_pos:={translation[0]} y_pos:={translation[1]} z_pos:={translation[2]} R_pos:=0 P_pos:=0 Y_pos:=0')
