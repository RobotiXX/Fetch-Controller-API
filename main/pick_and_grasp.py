#!/usr/bin/env python
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot

def getArmReady():
    robot.execute([[0.35, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    rospy.sleep(6)

def pick():
    robot.execute([[0.35, 0.0, 0.0, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    rospy.sleep(6)
    robot.grasp()
    rospy.sleep(3)
    robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
    rospy.sleep(6)

def place():
    robot.execute([[0.35, 0.0, -0.05, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)

    # robot.release()

    robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)

    robot.execute([[0.35, 0.0, 0.0, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)

    robot.release()

    rospy.sleep(6)

    robot.execute([[0.35, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)

if __name__ == '__main__':
    # rospy.init_node("example_repub")
    robot = FetchRobot()

    # timeLimit = 6
    # robot.release()


    # rospy.sleep(2)


    # robot.getReady()

    # rospy.sleep(timeLimit)

    # print("Getting arm to position....")

    # getArmReady()

    # print("Picking....")

    # pick()

    # print("Placing...")

    # place()

    # robot.getReady()