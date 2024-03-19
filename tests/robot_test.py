#!/usr/bin/env python
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot

if __name__ == '__main__':
    rospy.init_node("example_repub")
    robot = FetchRobot()

    timeLimit = 6
    robot.release()


    rospy.sleep(2)


    robot.getReady()

    rospy.sleep(timeLimit)

    # robot.release()

    robot.execute([[0.35, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # robot.getUnReady()
    # print("hahaha")
    # robot.getReady()

    rospy.sleep(6)
    # # robot.go(1, 2)


    robot.execute([[0.35, 0.0, 0.0, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)

    robot.grasp()

    rospy.sleep(3)

    robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(3)

    # robot.release()

    robot.execute([[0.35, 0.0, -0.05, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(timeLimit)

    # robot.release()

    robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(timeLimit)

    robot.execute([[0.35, 0.0, 0.0, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(timeLimit)

    robot.release()

    rospy.sleep(timeLimit)

    robot.execute([[0.35, 0.0, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    rospy.sleep(6)
    robot.getReady()
    # robot.getUnReady()
    # rospy.sleep(2)

    # robot.report_fetch_state()
    # print(robot.getGripperLocation())

    # robot.execute([[0.35, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # rospy.sleep(5)

    # robot.report_fetch_state()
    # print(robot.getGripperLocation())

# def joint_states_callback(message):
#     # filter out joint0 position:
#     print('---------------------------------------------------------')
#     for i,name in enumerate(message.name):
#         pos = message.position[i]
#         if name in joints.keys():
#             joints[name] = pos   

#     print(joints)
    

# if __name__ == '__main__':
#     rospy.init_node("example_repub")
#     joints = {ARM_AND_TORSO_JOINTS[i]: 0.0 for i in range(len(ARM_AND_TORSO_JOINTS))}
#     # pub = rospy.Publisher("joint0_topic", Float64, queue_size=1)
#     rospy.Subscriber("joint_states", JointState, joint_states_callback, queue_size=1)
#     rospy.spin()
