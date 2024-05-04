! /usr/bin/env python3

import rclpy
import rclpy.clock
from rclpy.node import Node

import numpy as np
import math
import sys
import time

import rclpy.time
import rclpy.timer
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import String

#define global variables
global desired_gap, desired_align
#Initialize setpoints
desired_gap = 0.3       #Distance gap of 30 cm or 0.3 meters
desired_align = 0       #Drive straight

class Controller(Node):

    #Initialize input data and error signals
    current_distance = 0
    current_align = 0
    error_distance = 0
    error_align = 0

    #Initialize Derivative error terms
    current_dThrot = 0
    current_dStr = 0

    current_velThr = 0
    current_velStr = 0

    #Initialize Integral Error terms
    di_Thr = 0
    di_Str = 0
    current_time = 0
    
    def __init__(self):
        super().__init__("controller")
        self.get_logger().info("Node Started")

        #Create Subscriptions
        self.camera_info_data = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_callback, 10 )
        self.current_vel = self.create_subscription(Twist, 'cmd_vel', self.get_currentVel, 10 )
        self.point_data = self.create_subscription(Point, 'point', self.geterror, 10 )
        self.imu_data = self.create_subscription(Imu, 'imu', self.imu_callback, 10 )

        #Create Publisher
        self.my_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        #self.get_logger().info("Vehicle Linear Acceleration: %s" )
        #self.get_logger().info("Vehicle Angular Velocity: %s" )
        #Integral Controller Initial values
        Controller.last_time = rclpy.time.Time()
        Controller.di_Thr = 0
        Controller.di_Str = 0


    def imu_callback(self, msg:Imu):
         lin_accel_x = msg.linear_acceleration.x
         ang_vel_z = msg.angular_velocity.z

         self.get_logger().info("Vehicle Linear Acceleration: %s" % lin_accel_x)
         self.get_logger().info("Vehicle Angular Velocity: %s" % ang_vel_z)
    def camera_callback(self,msg:CameraInfo):
         Camera_Frame_Height = msg.height
         Camera_Frame_Width = msg.width
         self.get_logger().info("Camera Frame Size (Height,Width): (%s,%s)" % Camera_Frame_Height % Camera_Frame_Width )
        
    def get_currentVel(self, msg:Twist):
         #For Integral Error calculation
         #Read Twist cmd_vel outputs (could also use integrated IMU linear acceleration )
         self.current_velThr = msg.linear.x
         self.current_velStr = msg.angular.z

    
    def geterror(self, msg:Point):
            #PID Gain Parameters
            Kp_throttle = 1
            Kp_steer = 1

            Kd_throttle = 1
            Kd_steer = 1

            Ki_throttle = 0
            Ki_steer = 0

            #For Derivative Error
            current_dThrot = self.current_velThr
            current_dStr = self.current_velStr

            #Map input variables
            current_distance = msg.z    #Map Point Z to current Distance
            current_align = msg.x       #Map Point X to current Alignment

            self.get_logger().info("Marker Horizontal Position: %s" % current_align)
            self.get_logger().info("Marker Distance: %s" % current_distance )


            #Calculate Errors
            error_distance = desired_gap - current_distance
            error_align = desired_align - current_align

            #print 'error_distance:' ,error_distance
            #print 'error_align' ; error_align

            #Compute Integral Error delta t
            #Controller.current_time = rclpy.time.Time()
            #dt = Controller.current_time - Controller.last_time()

            #Compute Integral Error
            #Controller.di_Thr = Controller.di_Thr + dt*error_distance
            #Controller.di_Str = Controller.di_Str + dt*error_align

            

            #Generate Control Signals
            uv_throttle = Kp_throttle*error_distance + Kd_throttle*(-current_dThrot) #+ Ki_throttle*Controller.di_Thr
            uv_steering = Kp_steer*error_align + Kd_steer*(-current_dStr) #+ Ki_steer*Controller.di_Str

            #Acceleration Limit
            #Insert if needed
            #Fine tune the velocity step and max acceleration


            #Map control signals to Twist cmd_vel
            self.publish_cmd_vel(uv_throttle, uv_steering)



    def publish_cmd_vel(self, uv_throttle, uv_steering):
         cmd_vel = Twist()
         cmd_vel.linear.x = uv_throttle
         cmd_vel.angular.z = uv_steering
         self.my_cmd_vel.publish(cmd_vel)

         

def main(args=None):
    rclpy.init(args=args)

    #Nodes go here
    controller = Controller()
    
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
