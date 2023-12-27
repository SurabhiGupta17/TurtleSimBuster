#!/usr/bin/env python3
"""
This node spawns turtles at random coordinates at intervals and publishes the said 
coordinates over a topic.
"""

import rclpy
import random
from rclpy.node import Node
from functools import partial

from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from std_msgs.msg import String

class SpawnTurtleNode(Node):
    def __init__(self): 
        super().__init__("spawn_turtle")

        self.counter_ = 1
        # Set up the timer without calling the function
        self.publish_coordinates = self.create_publisher(Pose, "spawn_coordinates", 10)
        self.publish_turtle_name = self.create_publisher(String, "turtle_name", 10)
        self.create_timer(15, partial(self.spawn_turtle_timer_callback))

    def spawn_turtle_timer_callback(self):
        # Call the spawn_turtle function with random values
        self.x_coordinate = random.uniform(0.5, 10.5)
        self.y_coordinate = random.uniform(0.5, 10.5)
        self.theta_value = random.uniform(0.0, 8.0)
        self.name_of_turtle = f"turtle_{self.counter_}"
        self.spawn_turtle(self.x_coordinate, self.y_coordinate, self.theta_value, self.name_of_turtle)
        self.coordinate_publisher()
        self.turtle_name_publisher()

    def coordinate_publisher(self):
        turtle = Pose()
        turtle.x = self.x_coordinate
        turtle.y = self.y_coordinate
        turtle.theta = self.theta_value
        turtle.linear_velocity = 0.0
        turtle.angular_velocity = 0.0
        self.publish_coordinates.publish(turtle)

    def turtle_name_publisher(self):
        turtle_name = String()
        turtle_name.data = self.name_of_turtle
        self.publish_turtle_name.publish(turtle_name) 
          
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


def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
