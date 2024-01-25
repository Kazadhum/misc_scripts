import subprocess
import numpy as np
import transforms3d as t3d
from scipy.spatial.transform import Rotation  
import tf
from copy import deepcopy


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
transform_matrix = np.array([[-0.286486, -0.646401, -0.707171, 22.0158],
                            [-0.958053, 0.18732, 0.2169, 9.02334],
                            [-0.00462435, 0.751655, -0.65954, 1.45299],
                            [0,0,0,1]])




transform_rot = deepcopy(transform_matrix)

transform_rot[0:3, 3] = 0
print(transform_rot)

# fros_T_fmp3d = np.array([[0,0,1,0],
#                         [-1,0,0,0],
#                         [0,-1,0,0],
#                         [0,0,0,1]])

# fros_T_fmp3d = np.array([[1,0,0,0],
#                         [0,-1,0,0],
#                         [0,0,-1,0],
#                         [0,0,0,1]])

# TESTING
fros_T_fmp3d = np.array([[1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0],
                        [0,0,0,1]])

# Composed rotation matrix
composed_rotation = np.dot(fros_T_fmp3d, transform_rot)
# composed_rotation = np.dot(transform_rot, fros_T_fmp3d)

# Extract translation (x, y, z)
translation = transform_matrix[:3, 3]
x, y, z = translation
# print(translation)

# Extract rotation as a matrix
rotation_matrix = composed_rotation[:3, :3]
print(rotation_matrix)


# Convert rotation matrix to quats
angles_quat = tf.transformations.quaternion_from_matrix(composed_rotation)
print(f'Quaternion: {angles_quat}')


# Convert rotation matrix to RPY (Roll, Pitch, Yaw)
# rpy = t3d.euler.mat2euler(rotation_matrix, axes='szyx')
# roll, pitch, yaw = rpy

# TO TRY: XYX, XZX,YXY, YXZ, YZX, YZY, ZXY, ZXZ, ZYX, ZYZ
# TRIED: xyz, XZY 

# angles = tf.transformations.euler_from_matrix(rotation_matrix, axes = 'rxyz') # close
angles = tf.transformations.euler_from_matrix(rotation_matrix, axes = 'szyx')

print(angles)
# r =  Rotation.from_matrix(rotation_matrix)
# angles = r.as_euler("zyx",degrees=False)
# print(angles)

# print(f'roslaunch synfeal_bringup bringup_camera.launch x_pos:=20.7373 y_pos:=10.6052 z_pos:=1.44539 R_pos:={angles[0]} P_pos:={angles[1]} Y_pos:={angles[2]}')

# execute(cmd=f'roslaunch synfeal_bringup bringup_camera.launch x_pos:={translation[0]} y_pos:={translation[1]} z_pos:={translation[2]} R_pos:={angles[0]} P_pos:={angles[1]} Y_pos:={angles[2]}')

# execute(cmd=f'roslaunch synfeal_bringup bringup_camera.launch x_pos:=22.015800 y_pos:=9.023340 z_pos:=1.452990 R_pos:=-0.014435 P_pos:=-0.761797 Y_pos:=0.853644') # Testing something


# Print the results
# print("Translation (x, y, z):", x, y, z)
# # print("Rotation (RPY):", np.degrees(roll), np.degrees(pitch), np.degrees(yaw))

# print("Rotation (RPY):", roll, pitch, yaw)
