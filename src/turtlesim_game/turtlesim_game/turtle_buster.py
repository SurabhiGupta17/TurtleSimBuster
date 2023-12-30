#!/usr/bin/env python3

import rclpy
import random
import math
from rclpy.node import Node
from functools import partial
from scipy.spatial import distance

from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from geometry_msgs.msg import Twist

class TurtleBusterNode(Node):
    def __init__(self): 
        super().__init__("turtle_buster")

        #Initialise values
        self.counter_ = 1
        self.name_of_turtle = self.turtle_updated = "turtle_0"
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.Kp_angular = 2.0
        self.Kp_linear = 1.3

        #Create subscribers
        self.main_turtle_pose_sub = self.create_subscription(
            Pose, "/turtle1/pose", self.main_turtle_pose_cb, 10
            )

        #Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10
            )

        #Create timers
        self.create_timer(0.5, self.cmd_vel_pub_cb)

        self.spawn_turtle_timer_cb()

    #Create callbacks
    def main_turtle_pose_cb(self, msg):
        self.main_x = msg.x
        self.main_y = msg.y
        self.main_theta = msg.theta


    def update_velocity(self):
            self.spawn_location = (self.spawn_x, self.spawn_y)
            self.main_location = (self.main_x, self.main_y)
            self.linear_x = self.Kp_linear*distance.euclidean(self.spawn_location, self.main_location)

            self.desired_theta = math.atan2(self.spawn_y - self.main_y, self.spawn_x - self.main_x)
            self.theta_difference = (self.desired_theta - self.main_theta)

            #Normalization of angle difference
            if self.theta_difference > math.pi:
                self.theta_difference -= 2 * math.pi
            elif self.theta_difference < -math.pi:
                self.theta_difference += 2 * math.pi
            self.angular_z = self.Kp_angular*self.theta_difference
        
    def cmd_vel_pub_cb(self):
        velocity = Twist()
        self.update_velocity()
        velocity.linear.x = self.linear_x
        velocity.angular.z = self.angular_z
        self.cmd_vel_pub.publish(velocity)
        self.kill_turtle()


    #Spawn turtle
    def spawn_turtle_timer_cb(self):
        # Call the spawn_turtle function with random values
        self.spawn_x = random.uniform(0.5, 10.5)
        self.spawn_y = random.uniform(0.5, 10.5)
        self.spawn_theta = random.uniform(0.0, 8.0)
        self.name_of_turtle = f"turtle_{self.counter_}"
        self.spawn_turtle(self.spawn_x, self.spawn_y, self.spawn_theta, self.name_of_turtle)

    def spawn_turtle(self, x, y, theta, name):  
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.spawn_turtle_callback, x=x, y=y, theta=theta, name=name))

    def spawn_turtle_callback(self, future, x, y, theta, name):
        self.counter_ = self.counter_ + 1
        try:
            response = future.result()
            self.get_logger().info(str(response.name))
        except Exception as e:
            self.get_logger().error("Service call failed %r", (e,))

    
    #Kill turtle
    def kill_turtle(self):
            if abs(self.spawn_x - self.main_x)<0.2 and abs(self.spawn_y - self.main_y)<0.2:
                self.call_kill_turtle(self.name_of_turtle)
                self.spawn_turtle_timer_cb()
        
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
    node = TurtleBusterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()