
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import ARM_AND_TORSO_JOINTS, INTRINSIC_PARAM_CAMERA
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

class PoseTracker:
    def __init__(self):
        self.prev_tf = None
        self.cameraPoseTilt = 0
        self.cameraPosePan = 0
        

# Initialize the CvBridge class
bridge = CvBridge()

# # Initialize a tracker
# last_translate = None


def drawBoxes(results, image):
    for r in results:
	# extract the bounding box (x, y)-coordinates for the AprilTag
	# and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # print("[INFO] tag family: {}".format(tagFamily))

# def execute(T):
#     if ptk.prev_tf is None:
#         ptk.prev_tf = T
#     else:
#         xx, xy, xz = ptk.prev_tf
#         nx, ny, nz = T

#     # tilt [-0.785 (U), 1.5708 (D) rad] = [-45, 90] 
#     # pan [-1.5708 (R), 1.5708 (L) rad] = [-90, 90] 

#         h_threshold = 40
#         v_threshold = 30

#         if nx > h_threshold:
#             # move right
#             alpha_horizon = -.005
#         elif nx  < -h_threshold:
#             # move left
#             alpha_horizon = .005
#         else:
#             # stay
#             alpha_horizon = 0

#         if ny > v_threshold:
#             # move up
#             alpha_vertical = .005
#         elif ny < -v_threshold:
#             # move down
#             alpha_vertical = -.005
#         else:
#             # stay
#             alpha_vertical = 0

#         newHorizon = ptk.cameraPosePan + alpha_horizon

#         if np.abs(newHorizon) > 1.5708:
#             newHorizon = ptk.cameraPosePan

#         newVertical = ptk.cameraPoseTilt + alpha_vertical

#         if newVertical > 1.5708 or newVertical < -0.785:
#             newVertical = ptk.newVertical

#         inputMatrix = [[newHorizon, newVertical], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
#         robot.lookAt(inputMatrix)

#         # update
#         ptk.prev_tf = T
#         ptk.cameraPosePan = newHorizon
#         ptk.cameraPoseTilt = newVertical


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
    
    k = cv2.waitKey(1)
    if k%256 == 27:
        # ESC pressed
        print("Escape hit, closing...")
    elif k%256 == 32:
        # SPACE pressed
        img_name = "test_frame.png"
        cv2.imwrite(img_name, cv_image)
        print("{} written!".format(img_name))


    show_image(cv_image)

if __name__ == '__main__':
    
    rospy.init_node('test')
    # Initialize a tracker
    last_translate = None
    robot = FetchRobot()
    inputMatrix = [[0.0, 0.0], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
    # robot.lookAt(inputMatrix)
    # robot.getReady()
    # inputMatrix2 = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0 for _ in range(len(ARM_AND_TORSO_JOINTS))], [0.0 for _ in range(len(ARM_AND_TORSO_JOINTS))]]
    # robot.execute(inputMatrix2)

    # tracker = BoardTracker()
    # ptk = PoseTracker()

    # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
    sub_image = rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_callback)

    # Initialize an OpenCV Window named "Image Window"
    # cv2.namedWindow("Image Window", 1)

    # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
    while not rospy.is_shutdown():
        rospy.spin()

# import cv2

# cam = cv2.VideoCapture(0)

# cv2.namedWindow("test")

# img_counter = 0

# while True:
#     ret, frame = cam.read()
#     if not ret:
#         print("failed to grab frame")
#         break
#     cv2.imshow("test", frame)

#     k = cv2.waitKey(1)
#     if k%256 == 27:
#         # ESC pressed
#         print("Escape hit, closing...")
#         break
#     elif k%256 == 32:
#         # SPACE pressed
#         img_name = "opencv_frame_{}.png".format(img_counter)
#         cv2.imwrite(img_name, frame)
#         print("{} written!".format(img_name))
#         img_counter += 1

# cam.release()

# cv2.destroyAllWindows()