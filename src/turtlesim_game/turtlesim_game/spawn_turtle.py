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

class SpawnTurtleNode(Node):
    def __init__(self): 
        super().__init__("spawn_turtle")

        #Initialise values
        self.counter_ = 1
        self.name_of_turtle = self.turtle_updated = "turtle_0"
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.NoMotion = False
        self.FaceTarget = False
        self.Kp_angular = 1
        self.Kp_linear = 0.7

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
        #self.create_timer(10, partial(self.spawn_turtle_timer_cb))

        self.spawn_turtle_timer_cb()

    #Create callbacks
    def main_turtle_pose_cb(self, msg):
        self.main_x = msg.x
        self.main_y = msg.y
        self.main_theta = msg.theta

    def check_phase(self):
        if (self.name_of_turtle == "turtle_0"):
            self.NoMotion = True
            self.FaceTarget = False
            self.MoveToTarget = False
            self.KillTurtle = False
        elif (self.turtle_updated != self.name_of_turtle):
            self.FaceTarget = True
            self.NoMotion = False
            self.MoveToTarget = False
            self.KillTurtle = False
        elif (abs(self.spawn_x-self.main_x)<0.2 and abs(self.spawn_y-self.main_y)<0.2):
            self.MoveToTarget = False
            self.NoMotion = False
            self.FaceTarget = False
            self.KillTurtle = True
        else: #(abs(self.spawn_x-self.main_x)<0.8 and abs(self.spawn_y-self.main_y)<0.8):
            self.KillTurtle = False
            self.NoMotion = False
            self.MoveToTarget = True
            self.FaceTarget = False


    def update_velocity(self):
        if (self.NoMotion):
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.get_logger().info("NoMotion")
        elif(self.FaceTarget):
            self.get_logger().info("FaceTarget")
            self.linear_x = 0.0
            self.desired_theta = math.atan2(self.spawn_y - self.main_y, self.spawn_x - self.main_x)
            self.angular_z = self.Kp_angular*(self.desired_theta - self.main_theta)
            if abs(self.desired_theta - self.main_theta)<0.01:
                self.angular_z = 0.0
                self.turtle_updated = self.name_of_turtle
        elif (self.MoveToTarget):
            self.get_logger().info("MoveToTarget")
            self.angular_z = 0.0
            self.spawn_location = (self.spawn_x, self.spawn_y)
            self.main_location = (self.main_x, self.main_y)
            self.linear_x = self.Kp_linear * distance.euclidean(self.spawn_location, self.main_location)
        elif (self.KillTurtle):
            self.get_logger().info("KillTurtle")
            self.linear_x = 0.0
            self.angular_z = 0.0
            self.kill_turtle()
            
        
    def cmd_vel_pub_cb(self):
        velocity = Twist()
        self.check_phase()
        self.update_velocity()
        velocity.linear.x = self.linear_x
        velocity.angular.z = self.angular_z
        self.cmd_vel_pub.publish(velocity)


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
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
