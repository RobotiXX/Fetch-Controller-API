import rospy
import numpy as np
from tf import TransformListener
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from src.fetch_controller_python.fetch_robot import FetchRobot
from lib.utils import MatrixOperation
from lib.params import BASE_DEST_TRANSFORM
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Pose, PoseStamped
import tf

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

if __name__ == '__main__':
    
    rospy.init_node("frame_transform")

    listener = TransformListener()

    robot = FetchRobot()

    robot.getReady()

    robot.execute([[0.35, 0.0, 0.0, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # rospy.sleep(3)

    op = MatrixOperation()


    # # # rate = rospy.Rate(10.0)

    # # print(listener.allFramesAsString())

    listener.waitForTransform( '/base_link', '/gripper_link', rospy.Time(), rospy.Duration(0.0))

    t, r = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))

    m = listener.fromTranslationRotation(t, r)


    g = op.getTransformMatrix(translation=(0.92, 0, 0.8))

    print(g)
    print()

    ans = np.dot(op.getInverseMatrix(m), g)

    print(ans[:,3:])

    
    # print(posefromMatrix(m))

    op = MatrixOperation()

    # b = op.getTransformMatrix()

    # g = op.getTransformMatrix(translation=(0.92, 0, 0.8))

    # print(g)
    # print()

    # print(np.dot(op.getInverseMatrix(m), g))

    # print()

    # # ########################

    # # robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # # robot.execute([[0.35, 0.0, -0.05, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # # rospy.sleep(5)

    # # rate = rospy.Rate(10.0)

    # # print(listener.allFramesAsString())

    # listener.waitForTransform('/base_link', '/gripper_link' ,rospy.Time(), rospy.Duration(0.0))

    # t, r = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))

    # print(listener.fromTranslationRotation(t, r))
    # print()

    #########################3


    # robot.execute([[0.35, 0.0, -0.05, 0.0, 2.0, 0.0, -2.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # rospy.sleep(6)

    # robot.release()

    # robot.execute([[0.35, 0.0, -0.05, 0.0, 1.2, 0.0, -1.2, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

    # rospy.sleep(6)

    # while not rospy.is_shutdown():
    #     # print(listener.allFramesAsString())

    #     listener.waitForTransform('/map','/base_link',rospy.Time(), rospy.Duration(1.0))

    #     t, r = listener.lookupTransform('/map', '/base_link', rospy.Time(0))

    #     print(listener.fromTranslationRotation(t, r))

    #     map = np.eye(4,4)

    #     base = np.dot(listener.fromTranslationRotation(t, r), np.transpose(np.array([0,0,0,1])))

    #     print("*************************************************")

    #     print(base)

    #     print("============================================================================")

    #     rate.sleep()
