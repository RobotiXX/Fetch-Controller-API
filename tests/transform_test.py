import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot

if __name__ == '__main__':
    
    rospy.init_node("frame_transform")
    robot = FetchRobot()


    rospy.sleep(5)

    print("\n =========================================== \n")

    print(robot.get_joint_states())


    print("\n =========================================== \n")

    print(robot.get_head_states())

    print("\n =========================================== \n")

    print(robot.getHeadCameraLocation())

    rospy.sleep(5)

    print(robot.getHeadCameraLocation())


    print("\n =========================================== \n")
    print("\n =========================================== \n")