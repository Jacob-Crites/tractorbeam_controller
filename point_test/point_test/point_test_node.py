#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import Point

class Point_Test_Node(Node): #Node Class (just for program)

    def __init__(self):
        super().__init__("point_test_node") #Node name
                                             #What is typed in terminal
        self.my_point = self.create_publisher(Point, 'point', 10)
        self.get_logger().info("Publishing Fake Data")
        self.timer = self.create_timer(1.0, self.point_publish)
        

    def point_publish(self):
        fake_data = Point()
        fake_data.x = float(5)
        fake_data.z = float(3)
        self.get_logger().info("Publishing X: " + str(fake_data.x) )
        self.get_logger().info("Publishing Z: " + str(fake_data.z) )
        self.my_point.publish(fake_data)
            


def main(args = None):
    rclpy.init(args=args)
    node = Point_Test_Node()
    #
    rclpy.spin(node)

    rclpy.shutdown()
if __name__ == '__main__':
    main()
