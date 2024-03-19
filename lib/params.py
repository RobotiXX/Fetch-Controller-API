import numpy as np

### JOINT STATES
JOINT_STATES = "joint_states"

### BASE POS
BASE_POS = [0.0, 0.0, 0.0]

### GRIPPER Params
CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
MIN_EFFORT = 35   # Min grasp force, in Newtons
MAX_EFFORT = 100  # Max grasp force, in Newtons
GRIPPER_CONTROL_GROUP = 'gripper_controller/gripper_action'

### MANIPULATOR Params
# ARM_AND_TORSO
ARM_REST_POSITION = [0.05, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                     1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
ARM_READY_POSITION = [0.35, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                      1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
MAX_JOINT_VEL = [0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26]
ARM_AND_TORSO_JOINTS = ['torso_lift_joint',
                        'shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'upperarm_roll_joint',
                        'elbow_flex_joint',
                        'forearm_roll_joint',
                        'wrist_flex_joint',
                        'wrist_roll_joint']
# HEAD
HEAD_JOINTS = ['head_pan_joint',
               'head_tilt_joint']

# tilt_joint ranges [-0.785 (U), 1.5708 (D) rad] = [-45, 90] 
# pan_joint ranges [-1.5708 (R), 1.5708 (L) rad] = [-90, 90] 

ARM_TORSO_CONTROL_GROUP = 'arm_with_torso_controller/follow_joint_trajectory'
HEAD_CONTROL_GROUP = 'head_controller/follow_joint_trajectory'

### MOVE Params
# /cmd_vel allows controller commandd to take over the robot
MOVE_NODE = '/cmd_vel' # /cmd_vel OR /teleop/cmd_vel OR /base_controller/command 
CONTROL_RATE = 10
MAX_LINEAR_VELOCITY = 4.8

### VISION PARAM
VISION_IMAGE_TOPIC = "/head_camera/rgb/image_raw" # "/head_camera/rgb/image_rect_color"
VISION_CAMERA_INFO_TOPIC = "/head_camera/rgb/camera_info"

# Calibration matrix for head camera
'''  
[fx  0 cx]
[ 0 fy cy]
[ 0  0  1]
'''
CAMERA_CALIBRATION_MATRIX = [574.0527954101562, 0.0, 319.5, 0.0, 574.0527954101562, 239.5, 0.0, 0.0, 1.0]

# Intrinsic parameters for camera (fx, fy, cx, cy)
INTRINSIC_PARAM_CAMERA = (CAMERA_CALIBRATION_MATRIX[0], CAMERA_CALIBRATION_MATRIX[4], CAMERA_CALIBRATION_MATRIX[2], CAMERA_CALIBRATION_MATRIX[5])

#APRILTAG PARAM in m
APRIL_TAG_SIZE = 0.06 # for cabinet project tags 

### TRANSFORMATION MATRIX (TRANSLATION) in meters

'''
At [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] joint pos
'''

# ODOM2BASE = []

BASE2TORSO = [-0.087, 0.000, 0.410] 

# TORSO TO HEAD CAMEAR

TORSO2HEADPAN = [0.053, 0.000, 0.603]

HEADPAN2HEADTILT = [0.143, 0.000, 0.058]

HEADTILT2HEADCAM = [0.055, 0.000, 0.022]

# TORSO TO ARM

TORSO2SHOULDERPAN = [0.120, 0.000, 0.349]

SHOULDERPAN2SHOULDERLIFT = [0.117, 0.000, 0.060]

SHOULDERLIFT2UPPERARM = [0.219, 0.000, 0.000]

UPPERARM2ELBOW = [0.133, 0.000, 0.000]

ELBOW2FOREARM = [0.197, 0.000, 0.000]

FOREARM2WRISTFLEX = [0.124, 0.000, 0.000]

WRISTFLEX2WRISTROLL = [0.139, 0.000, 0.000]

WRISTROLL2GRIPPER = [0.166, 0.000, 0.000]