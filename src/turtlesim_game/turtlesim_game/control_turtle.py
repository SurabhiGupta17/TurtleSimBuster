#!/usr/bin/env python3
"""
Next Step: Implement PID control on the angular and linear velocity.                                                      
"""

import rclpy
from rclpy.node import Node
import math

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.srv import Kill
from functools import partial
from scipy.spatial import distance

class ControlTurtleNode(Node):
    def __init__(self):
        super().__init__("control_turtle")

        #Initialise values
        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_theta = 0.0
        self.main_x = 5.544445
        self.main_y = 5.544445
        self.main_theta = 0.0
        self.Kp = 0.2
        self.linear_x = 0.0
        self.angular_z = 0.0

        #Create subscriptions
        self.spawned_turtle_pose_sub = self.create_subscription(
            Pose, "spawn_coordinates", self.spawned_turtle_pose_cb, 10
            )
        
        self.main_turtle_pose_sub = self.create_subscription(
            Pose, "/turtle1/pose", self.main_turtle_pose_cb, 10
            )
        
        self.turtle_name_sub = self.create_subscription(
            String, "turtle_name", self.turtle_name_cb, 10
            )
        
        #Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
            )
        
        self.create_timer(0.5, self.cmd_vel_pub_cb) 
        
    def cmd_vel_pub_cb(self):
        velocity = Twist()
        self.update_velocity()
        velocity.linear.x = self.linear_x
        velocity.angular.z = self.angular_z
        self.cmd_vel_pub.publish(velocity) 
        
    #Create callbacks
    def spawned_turtle_pose_cb(self, msg):
        self.spawn_x = msg.x
        self.spawn_y = msg.y
        self.spawn_theta = msg.theta
        self.linear_x = 0.0
        self.angular_z = 0.0

    def main_turtle_pose_cb(self, msg):
        self.main_x = msg.x
        self.main_y = msg.y
        self.main_theta = msg.theta

    def turtle_name_cb(self, msg):
        self.kill_turtle()
        self.turtle_name = msg.data

    #Update velocity
    def update_velocity(self):
        self.desired_theta = math.atan2(self.spawn_y - self.main_y, self.spawn_x - self.main_x)
        self.angular_z = self.Kp*(self.desired_theta - self.main_theta)

        if (self.desired_theta - self.main_theta)<0.07:
            self.linear_x = 2.0
            self.angular_z = 0.0
            #self.kill_turtle()

    #Kill turtle
    def kill_turtle(self):
        if (abs(self.spawn_x-self.main_x)<0.5 and abs(self.spawn_y-self.main_y)<0.5):
            self.call_kill_turtle(self.turtle_name)
            self.linear_x = 0.0
            self.linear_y = 0.0

    def call_kill_turtle(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server......")
            
        request = Kill.Request()
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.kill_turtle_callback, name=name))

    def kill_turtle_callback(self, future, name):
        try:
            response = future.result()
            self.get_logger().info(str(response.name))
        except Exception as e:
            self.get_logger().error("Service call failed")

def main(args=None):
    rclpy.init(args=args)
    node = ControlTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()