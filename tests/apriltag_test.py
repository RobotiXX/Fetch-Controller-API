import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import ARM_AND_TORSO_JOINTS, INTRINSIC_PARAM_CAMERA
from lib.board_tracker import BoardTracker
from lib.utils import MatrixOperation
from src.fetch_controller_python.fetch_robot import FetchRobot
import cv2
import apriltag
import argparse
import numpy as np


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


################################3

rospy.init_node("frame_transform")

robot = FetchRobot()


rospy.sleep(5)


img = cv2.imread("tests/test_frame.png") #("../bw_img.png")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 


tracker = BoardTracker()

# tracker = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

r = tracker.detect(gray)

# print(r)


a = r[0]


# print(a.homography)

ps, _, _ = tracker.getTransformation(a) 

print("\n========================== START ======================================\n")

print("Here, base is [0, 0, 0], we will just multiply with odom2Base matrix to get coordinate in world frame.")

print(f"\nDetected tag with ID:  {a.tag_id} \n")

print(f"Homography matrix from AprilTag is: \n {ps} \n")


camLoc = robot.getHeadCameraLocation()
print(f"\nThe camera position in base frame should be: {camLoc}\n")

op = MatrixOperation()

tagLoc2Base = op.transform(camLoc, op.getInverseMatrix(ps))


print(f"The tag position in base frame should be: {tagLoc2Base}\n")

print("\n========================== END =======================================\n")

# drawBoxes(r, img)


# show_image(img)