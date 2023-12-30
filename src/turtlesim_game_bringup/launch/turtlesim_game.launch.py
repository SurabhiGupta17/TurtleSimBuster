from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtle_buster = Node(
        package="turtlesim_game",
        executable="turtle_buster"
    )

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_buster)
    #ld.add_action(control_turtle)
    
    return ld