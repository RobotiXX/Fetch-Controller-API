from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot
import geometry_msgs
from lib.params import *
from geometry_msgs.msg import PoseWithCovarianceStamped
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import apriltag
from lib.utils import *
from lib.board_tracker import *
from lib.pose_tracker import *

bridge = CvBridge()
pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

# orientation_list = [0, 0, 0.306952, 0.951725]
# (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
# print(roll)
# print(pitch)
# print(yaw)

def kick_start_localization():
    # rospy.init_node('talker', anonymous=True)
    # pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(5)
    rate.sleep()
    rospy.loginfo(INITIAL_POSE_SUGGESTION)
    pub.publish(INITIAL_POSE_SUGGESTION)


def movebase_client(client, goal):

    # client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    # client.wait_for_server()
    goal.target_pose.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    
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

def dock(client):
    new_goal =  MoveBaseGoal()
    new_goal.target_pose.header.frame_id = "map"
    new_goal.target_pose.pose.position = INITIAL_POSE_SUGGESTION.pose.pose.position
    new_goal.target_pose.pose.orientation = INITIAL_POSE_SUGGESTION.pose.pose.orientation
    new_goal.target_pose.header.stamp = rospy.Time.now()

    client.send_goal(new_goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
    

if __name__ == '__main__':

    try:
        rospy.init_node('movebase_client_py')
        robot = FetchRobot()
        table = PoseTracker()
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        robot.getReady()

        print("Kick start localization")
        kick_start_localization()
        rospy.sleep(5)
        print("Sending sample goal to move_base")
        print(f"Next position: {WAYPOINT_A_POSE}")
        result = movebase_client(client, WAYPOINT_A_POSE)
        if result:
            rospy.loginfo("Goal execution done!")

        print(result)

        inputMatrix = [[0, 0.6], [0.0 for _ in range(2)], [0.0 for _ in range(2)]]
        robot.lookAt(inputMatrix)
        rospy.sleep(5)

        # Load table
        table.load_hashTable()

        print(table.hashTable)

        # Pick the 1st object

        if not table.is_empty():
            obj_pose = table.get_by_index(0) # THIS IS THE BASE POSE TO PICK IT UP

            print("Obtained new position")

            # do something with it
            # create a movebase pose
            new_goal =  MoveBaseGoal()
            new_goal.target_pose.header.frame_id = "map"
            new_goal.target_pose.pose.position.x = obj_pose.position.x
            new_goal.target_pose.pose.position.y = obj_pose.position.y
            new_goal.target_pose.pose.orientation.z = obj_pose.orientation.z
            new_goal.target_pose.pose.orientation.w = obj_pose.orientation.w

            print(f"Next position: {new_goal}")

            print("\nSending new goal to movebase...\n")
            result = movebase_client(client, new_goal)
            if result:
                rospy.loginfo("Goal execution done!")

            print("Getting arm to position....")

            getArmReady()

            print("Picking....")

            pick()

            print("Placing...")

            place()

            robot.getReady()
        else:
            print("Table is empty")

        rospy.sleep(15)

        isDock = dock(client)
        if isDock:
            rospy.loginfo("Dock execution done!")

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
