
from __future__ import annotations

import rclpy
from rclpy.node import Node

from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import IrIntensityVector
from scipy.spatial.transform import Rotation as R
from rclpy.qos import qos_profile_sensor_data
import math

from .Vel_Control_Test1 import *

class Nav_Test(Node):

    def __init__(self):
        super().__init__('Navigation_Test')
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/odom", self.odom_received, qos_profile_sensor_data)
        self.ir_subscription = self.create_subscription(IrIntensityVector, "/ir_intensity", self.ir_received, qos_profile_sensor_data)
        self.create_timer(0.02, self.publish_move)

        self.get_logger().info("Nav_Test node initalized")

        self.target = [1, 0]
        self.robot_loc = [0, 0, 0]
        self.ir = [0]*7
        self.i = 0
        self.init_loc = [0, 0, 0]
        self.rot_mat = [0, 0, 0, 0]
        self.diff1 = 0
        self.diff2 = 0

    def odom_received(self, msg: Odometry):
        '''Save current position and orientation whenever there are new measurements coming from the odometry'''
        o = msg.pose.pose.orientation
        pose = msg.pose.pose
        
        self.theta = R.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz")[2]
        self.robot_loc = [pose.position.x, pose.position.y, self.theta]
        
        if self.i == 0:
            self.init_loc = self.robot_loc
            self.rot_mat = [math.cos(self.init_loc[2]), math.sin(self.init_loc[2]), -math.sin(self.init_loc[2]), math.cos(self.init_loc[2])]
        else:
            
            self.diff1 = self.robot_loc[0] - self.init_loc[0]
            self.diff2 = self.robot_loc[1] - self.init_loc[1]
            

            self.robot_loc[0] = self.rot_mat[0]*(self.diff1) + self.rot_mat[1]*(self.diff2)
            self.robot_loc[1] = self.rot_mat[2]*(self.diff1) + self.rot_mat[3]*(self.diff2)
            self.robot_loc[2] = self.robot_loc[2] - self.init_loc[2]
            self.robot_loc[2] = self.robot_loc[2]%(2*math.pi)

        print(self.robot_loc)
        
        self.i += 1

    

    def ir_received(self, msg: IrIntensityVector):

        for reading in msg.readings:
            if reading.header.frame_id == "ir_intensity_side_left":
                self.ir[0] = reading.value
            elif reading.header.frame_id == "ir_intensity_left":
                self.ir[1] = reading.value
            elif reading.header.frame_id == "ir_intensity_front_left":
                self.ir[2] = reading.value
            elif reading.header.frame_id == "ir_intensity_front_center_left":
                self.ir[3] = reading.value
            elif reading.header.frame_id == "ir_intensity_front_center_right":
                self.ir[4] = reading.value
            elif reading.header.frame_id == "ir_intensity_front_right":
                self.ir[5] = reading.value
            elif reading.header.frame_id == "ir_intensity_right":
                self.ir[6] = reading.value


    def publish_move(self):
        
        msg = Twist()

        t_vel, r_vel = vel_control(self.robot_loc, self.target, self.ir)


        msg.angular.z = r_vel
        msg.linear.x = t_vel

        # msg.angular.z = 0.0
        # msg.linear.x = 1.0

        if self.i != 0:
            self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init()
    node = Nav_Test()
    
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        node.destroy_node()

