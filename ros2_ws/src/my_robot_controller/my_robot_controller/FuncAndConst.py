import time
import cmath
import rclpy
import logging
import numpy as np
import message_filters
import cmath
import math
import numpy as np


from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from transforms3d.euler import quat2euler
from std_msgs.msg import Float32MultiArray,  Empty, Bool, UInt16MultiArray, Int32, Int32MultiArray
from robot_interfaces.msg import Direction, MoveServo, Command, FinalInput, State
from sympy import symbols, Eq, solve, sin, tan, pi
from serial import Serial
from irobot_create_msgs.msg import IrIntensityVector
from message_filters import ApproximateTimeSynchronizer, Subscriber
from transforms3d.euler import quat2euler
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve
from scipy.optimize import minimize
from math import pi as PI
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import Parameter

# some definitions
dis_IR_rec = 1.60 # distance between two IR_receivers
Corners_Location = [[-dis_IR_rec/2, dis_IR_rec/2], [dis_IR_rec/2, dis_IR_rec/2], [-dis_IR_rec/2, -dis_IR_rec/2], [dis_IR_rec/2, -dis_IR_rec/2]]

#ser = Serial ("/dev/ttyACM0", 9600)
IR_SENSORS = {
    "ir_intensity_side_left":          0,
    "ir_intensity_left":               1,
    "ir_intensity_front_left":         2,
    "ir_intensity_front_center_left":  3,
    "ir_intensity_front_center_right": 4,
    "ir_intensity_front_right":        5,
    "ir_intensity_right":              6
}


queue_size = 100            #QUEUE SIZE PARAMETER FOR APPROX. TIME SYNC.
coupling_delay = .02        #COUPLING DELAY PARAMETER FOR APPROX. TIME SYNC.



CALIB_STEP = PI/30  #A STEP FOR THE SERVO MOTOR DURING CALIBRATION
CALIB_MAX = 0.95*PI  #CALIBRATION SERVO MOTOR TURNS FROM -CALIB_MAX TO CALIB_MAX OR VICE-VERSA


eps_ANG = 0.03       # IF THE ANGLE DIFFERENCE TO TARGET IS LOWER THAN THAT, WE ASSUME IT POINTS TO THE TARGET
eps_LIN_PICK = 0.08   #IF THE DISTANCE TO LOCATION FOR COLLECTING THE OBJECT IS LESS THAN THAT VALUE, ASSUME WE ARE ARRIVED
eps_LIN_LEAVE = 0.01 #IF THE DISTANCE TO LOCATION FOR LEAVING THE OBJECT IS LESS THAN THAT VALUE, ASSUME WE ARE ARRIVED
R_GRIPPER = 0.30     #gripper to center
W_FINE = 0.1         #speed of last turn 
TH_TUNING = 2*PI/180

FINE_MOVEMENT_SPEED = .1 #THE SPPED WHEN THE ROBOT SLOWLY MOVES TO OBJECT BEFORE CLOSING THE GRIPPER
SMALL_REVERSE_DISTANCE = 0.1 #THE DISTANCE FOR REVERSE MOVEMENT AFTER PUTTING DOWN THE OBJECT
SMALL_REVERSE_TIME = 0.5

MIN_OBSTACLE_DIST = 0.25 #FORCE-SPEED-FUNCTION PARAMATERS
OBSTACLE_FORCE = 1.0     #FORCE-SPEED-FUNCTION PARAMATERS
MAX_SPEED = 0.5          #FORCE-SPEED-FUNCTION PARAMATERS
TURNING_SPEED = 3.0      #FORCE-SPEED-FUNCTION PARAMATERS

GRIPPER_SERVO_OPENING_ANGLE = 180 #THE ANGLE OF SERVO WHEN THE GRIPPER IS OPEN
GRIPPER_SERVO_CLOSING_ANGLE = 40  #THE ANGLE OF SERVO WHEN THE GRIPPER IS CLOSED
ARM_SERVO_UP_ANGLE = 170    #THE ANGLE OF SERVO WHEN THE ARM IS UP
ARM_SERVO_DOWN_ANGLE = 68    #THE ANGLE OF SERVO WHEN THE ARM IS DOWN
ARM_SERVO_CALIB_ANGLE = 105  #THE ANGLE OF SERVO WHEN THERE IS CALIBRATION   
CALIB_SERVO_OFFSET = 142      #IT WILL PUBLISH TO SERVO THE GIVEN ANGLE - OFFSET, JUST IN CASE WE NEED
CALIB_SERVO_VEL_ZERO = 83.0 
CALIB_SERVO_VEL_POSITIVE = 84.5
CALIB_SERVO_VEL_NEGATIVE = 78.0
LED_SERVO_OFFSET = -60        #IT WILL PUBLISH TO SERVO THE GIVEN ANGLE - OFFSET, JUST IN CASE WE NEED
CALIB_WAIT_SERVO_TIME = 1.2 #THE DURATION BETWEEN GIVING THE SERVO MOVE COMMAND AND START TO LOCALIZATION
CALIB_WAIT_LOCALIZE_TIME = 0.5 #THE DURATION FOR LACALIZE AND READS FOR EACH ANGLE

ARM_UP_TIME = 1       #THE DURATION FOR UPWARDS MOVEMENT OF THE ARM
ARM_DOWN_TIME = 1      #THE DURATION FOR DOWNWARDS MOVEMENT OF THE ARM

SMALL_MOVEMENT_TIME = 1.5  #THE DURATION OF SLOWLY GETTING CLOSE TO OBJECT
GRASP_TIME = 1         #THE DURATION OF CLOSING THE GRIPPER
RELEASE_TIME = 1       #THE DURATION OF OPENING THE GRIPPER

LED_TH = -PI/2   #THE ANGLE OF LED ON THE ROBOT
LED_R = 0.15  #THE DISTANCE TO CENTER OF LED ON THE ROBOT
CALIBRATION_TIME = 25
N_CALIB_TOUR = 2
TURN_ANGLE_LIST_DEG = [-2,2]
#TURN_ANGLE_LIST_DEG = [-10,5,5,10,5,5]
TURN_ANGLE_LIST = [th*PI/180 for th in TURN_ANGLE_LIST_DEG]
logging.basicConfig(
         format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s',
         level=logging.INFO,
         datefmt='%Y-%m-%d %H:%M:%S')


def timing(delta_t):
    if not hasattr(timing, "state"):
        timing.state = "idle"
    if timing.state == "idle":
        timing.start = time.time()
        timing.state = "running"
    if timing.state =="running":
        if time.time()< timing.start + delta_t:
            return None
        else:
            timing.state = "idle"
            return 1


def angle_calc(th00, th01):
    th0 = th00%(2*PI)
    th1 = th01%(2*PI)
    mag =  min( max(th1-th0,th0-th1), 2*PI-max(th1-th0,th0-th1) )
    if th1-th0>PI:
        sgn = -1
    elif th1-th0>=0:
        sgn = 1
    elif th1-th0>=-PI:
        sgn = -1
    else:
        sgn =1
    return mag*sgn




def target_nav(x,y,theta, XTARGET, YTARGET):
    r, th = cmath.polar(complex(XTARGET-x, YTARGET-y))
    th_change = angle_calc(theta, th)
    return r, th_change


def odom_reader(odom: Odometry):
    pose_obj = odom.pose.pose
    position_obj = pose_obj._position
    orientation_obj = pose_obj.orientation
    x = position_obj._x
    y = position_obj._y
    theta = R.from_quat([orientation_obj.x,orientation_obj.y, orientation_obj.z, orientation_obj.w]).as_euler("xyz")[2]
    theta = theta%(2*PI)
    return [x,y, theta]
    


def process_loc(odom_current, offset, odom_calib, conv_angle):
    delta_x_robot_frame = odom_current[0] - odom_calib[0]
    delta_y_robot_frame = odom_current[1] - odom_calib[1]

    delta_x_lab_frame = delta_x_robot_frame*np.cos(conv_angle) - delta_y_robot_frame*np.sin(conv_angle)
    delta_y_lab_frame = delta_x_robot_frame*np.sin(conv_angle) + delta_y_robot_frame*np.cos(conv_angle)

    loc_x = offset[0] + delta_x_lab_frame
    loc_y = offset[1] + delta_y_lab_frame
    loc_theta = (odom_current[2] - odom_calib[2] + offset[2])%(2*PI)  

    return [loc_x, loc_y, loc_theta]


def finished(angle_list):
    if len(angle_list)<4:
        return False
    bins = [i*PI/9 for i in range(19)]
    x = np.histogram(angle_list, bins=bins)[0]

    return np.prod(x)>0 and abs(angle_calc(angle_list[0],angle_list[-1]))<4*PI/180



