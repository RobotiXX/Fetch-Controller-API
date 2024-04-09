import rospy
import sys
import time
import os
sys.path.insert(1, os.path.abspath("."))
from lib.pose_tracker import *
from lib.board_tracker import *
from lib.utils import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped
from lib.params import *
import geometry_msgs
from src.fetch_controller_python.fetch_robot import FetchRobot
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# TF joint names
# Lists of joint angles in the same order as in joint_names
disco_poses = [[0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
               [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
               [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
               [0.385, -1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
               [0.266, -0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
               [0.133, 0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
               [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]


rospy.init_node('disco_py')
robot = FetchRobot()
rospy.sleep(3)

robot.getReady()


for pose in disco_poses:
    print(f"Executing {pose}...")
    robot.execute([pose, [0.0 for _ in range(8)], [0.0 for _ in range(8)]])
    rospy.sleep(4)


robot.getReady()
