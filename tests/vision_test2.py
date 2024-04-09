
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import *
from lib.board_tracker import BoardTracker
from src.fetch_controller_python.fetch_robot import FetchRobot
# from src.fetch_controller_python.fetch_gripper import Gripper
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import apriltag
import argparse
import numpy as np
from lib.utils import *
        
# Initialize the CvBridge class
bridge = CvBridge()

# # Initialize a tracker
# last_translate = None

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(1)
    # if cv2.waitKey(1) == ord('q'):
    #     return

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
        
        result, overlay = apriltag.detect_tags(cv_image,
                                                apriltag.Detector(apriltag.DetectorOptions(families="tag36h11")),
                                                camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                                tag_size=APRIL_TAG_SIZE,
                                                vizualization=3,
                                                verbose=3,
                                                annotation=True
                                                )
        show_image(overlay)
    except:
        # print('*************************************************************************')
        # print("No APRILTAG detected")
        # print(np.eye(4, dtype=float))
        # print('*************************************************************************')
        show_image(cv_image)

if __name__ == '__main__':
    rospy.init_node('test')
    
    # Initialize a tracker
    last_translate = None
    robot = FetchRobot()
    # inputMatrix = [[0.0, 0.0], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
    # robot.lookAt(inputMatrix)
    # robot.getReady()
    # inputMatrix2 = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0 for _ in range(len(ARM_AND_TORSO_JOINTS))], [0.0 for _ in range(len(ARM_AND_TORSO_JOINTS))]]
    # robot.execute(inputMatrix2)

    tracker = BoardTracker()

    # inputMatrix = [[0, 0.5], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
    # robot.lookAt(inputMatrix)

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)

    # Initialize an OpenCV Window named "Image Window"
    # cv2.namedWindow("Image Window", 1)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()


    # robot.getReady()

    # inputMatrix = [[0.0, 0.0], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
    # robot.lookAt(inputMatrix)

    # tilt [-0.785 (U), 1.5708 (D) rad] = [-45, 90] 
    # pan [-1.5708 (R), 1.5708 (L) rad] = [-90, 90] 