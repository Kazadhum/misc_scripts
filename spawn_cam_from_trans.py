import rospy
import numpy as np
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


transform_matrix = np.array([[-0.286486, -0.646401, -0.707171, 22.0158],
                            [-0.958053, 0.18732, 0.2169, 9.02334],
                            [-0.00462435, 0.751655, -0.65954, 1.45299],
                            [0,0,0,1]])



# Initialize ROS node
rospy.init_node('spawn_urdf_node')

# Create a service proxy for the spawn_urdf service
spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

# Specify the URDF file path
urdf_file_path = "/home/diogo/catkin_ws/src/synfeal/synfeal_bringup/camera_description/urdf/localbot.urdf.xacro"

# Specify the name of the robot model
robot_name = "localbot"

# Specify the initial pose using the transformation matrix
initial_pose = Pose()

# Set the values of initial_pose based on your desired initial pose
initial_pose.position.x = transform_matrix[0,3]
initial_pose.position.y = transform_matrix[1,3]
initial_pose.position.z = transform_matrix[2,3]
initial_pose.orientation.x = 0
initial_pose.orientation.y = 0
initial_pose.orientation.z = 0
initial_pose.orientation.w = 0

# Call the spawn_urdf service
try:
    spawn_urdf(robot_name, open(urdf_file_path, 'r').read(), "/", initial_pose, "world")
except rospy.ServiceException as e:
    rospy.logerr("Spawn URDF service call failed: %s", str(e))