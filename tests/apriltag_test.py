import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import *
from lib.board_tracker import BoardTracker
from lib.utils import MatrixOperation
from src.fetch_controller_python.fetch_robot import FetchRobot
import cv2
import apriltag
import argparse
import numpy as np
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

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


def show_image(img):
    window_name = 'image'
    
    # Using cv2.imshow() method 
    # Displaying the image 
    cv2.imshow(window_name, img) 
    
    # waits for user to press any key 
    # (this is necessary to avoid Python kernel form crashing) 
    cv2.waitKey(0) 
    
    # closing all open windows 
    cv2.destroyAllWindows() 

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


################################3

rospy.init_node("frame_transform")

robot = FetchRobot()


rospy.sleep(5)

bt = BoardTracker() 
op = MatrixOperation()


for img_name in ["test_frame.png", "test_frame2.png"]:

    img = cv2.imread("tests/" + img_name) #("../bw_img.png")

    print("\n ************************************************************** \n")

    r = bt.detect(img)
    print(len(r))


    pose_m, _, _ = bt.detectPose(r[0])

    M = robot.getTransformationBase2Camera()

    pose_m_base = np.dot(M, pose_m)

    identity = op.getTransformMatrix()

    identity[:, 3:] = pose_m_base[:, 3:]

    print(identity)







# print(posefromMatrix(pose_m_base))

# print("\n++++++++++++++++++++++++++++++++++ \n ++++++++++++++++++++++++++++++++ \n")

# img2 = cv2.imread("tests/test_frame2.png") #("../bw_img.png")

# r2 = bt.detect(img2)
# print(len(r2))

# pose_m2, _, _ = bt.detectPose(r2[0])

# pose_m2_base = np.dot(M, pose_m2)

# # print(pose_m2)

# print(posefromMatrix(pose_m2_base))


# op = MatrixOperation()

# tag_pose_m = op.getInverseMatrix(pose_m)

# M = robot.getTransformationBase2Camera()

# pose_in_base = np.dot(M, tag_pose_m)

# print(pose_in_base)

# print(f"position of tag is: {pose_in_base[0:3, 3]}")

# # object Radius 0.04 and the height 0.17 

# # from gripper link to grasppable area is 6 + 6

# target_end_effector_position = pose_in_base[0:3, 3] - [0.06, 0, 0.08] # in base frame

# print(f"Target end effector is {target_end_effector_position}")

# target_end_effector_transform = op.getTransformMatrix(translation=target_end_effector_position)

# print(target_end_effector_transform) # in base frame

# new_base = np.dot(BASE_DEST_TRANSFORM, target_end_effector_transform)

# print(new_base)


# [[ 9.99998162e-01  1.91736032e-03  1.72762367e-06  9.23109441e-01]
#  [-1.91736104e-03  9.99997501e-01  1.14973513e-03 -1.70733538e-03]
#  [ 4.76837158e-07 -1.14973632e-03  9.99999339e-01  8.29872811e-01]
#  [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

# now, get transformation of the reaching position to base frame
