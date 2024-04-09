# THIS FILE SHOULD BE RUN IN THE BACKGROUND
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import BASE_DEST_TRANSFORM, VISION_IMAGE_TOPIC
from lib.board_tracker import BoardTracker
from lib.utils import *
from src.fetch_controller_python.fetch_robot import FetchRobot
# from src.fetch_controller_python.fetch_gripper import Gripper
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_msgs
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray
# geometry_msgs/PoseArray.msg
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Pose, PoseStamped
import tf

# Initialize the CvBridge class
bridge = CvBridge()

def posefromMatrix(matrix):
    m = matrix[:,:3]

    cur_matrix = m.reshape(3,4)
    cur_matrix_homo = np.vstack((cur_matrix, np.array([0, 0, 0, 1]))) # to homogenous coordinates

    q = tf.transformations.quaternion_from_matrix(cur_matrix_homo)

    p = Pose()
    p.position.x = matrix[0][3]
    p.position.y = matrix[1][3]
    p.position.z = matrix[2][3]
    p.orientation.x = q[0]
    p.orientation.y = q[1]
    p.orientation.z = q[2]
    p.orientation.w = q[3]

    return p



 # Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    # rospy.loginfo(img_msg.header)

     # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    try:
        # Show the converted image
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        results = tracker.detect(cv_image)
        ids = []
        poses = []
        for result in results:
            tag_pose_m, _, _ = tracker.detectPose(result)

            op = MatrixOperation()
 
            M = robot.getTransformationBase2Camera()

            pose_in_baseFr = np.dot(M, tag_pose_m)

            identity = op.getTransformMatrix()

            identity[0:2, 3:] = pose_in_baseFr[0:2, 3:]

            tm = op.getTransformMatrix(translation=(-1.12, 0, 0))

            # target_end_effector_position = pose_in_baseFr[0:3, 3] - [1.01, 0, 0.08] # in base frame

            new_base_pose = np.dot(tm, identity)

            # new_base_pose =np.dot(op.getInverseMatrix(BASE_DEST_TRANSFORM), target_end_effector_transform)

            base2map_m = robot.fromBase2Map()

            target_in_map_transform = np.dot(base2map_m, new_base_pose)
            
            p = posefromMatrix(target_in_map_transform)

            poses.append(p)
            ids.append(result.tag_id)
        ##
        pose_array.poses = poses

        publisher.publish(pose_array)

        id_array = Int32MultiArray()
        id_array.data = ids
        id_publisher.publish(id_array)
    except Exception as e: 
        print('*************************************************************************')
        print(e)
        print('*************************************************************************')
        # my_msg = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        # publisher.publish(my_msg)

if __name__ == '__main__':
    rospy.init_node('head_movement_tracking')
    rospy.sleep(3)
    # Initialize a tracker
    last_translate = None
    robot = FetchRobot()

    tracker = BoardTracker()

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber(VISION_IMAGE_TOPIC, Image, image_callback)

    # Init a publisher
    # publisher = rospy.Publisher('board2cam', Float32MultiArray, queue_size=10)
    publisher = rospy.Publisher('tagPoses', PoseArray, queue_size=10)
    id_publisher = rospy.Publisher('tagID', Int32MultiArray, queue_size=10)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()