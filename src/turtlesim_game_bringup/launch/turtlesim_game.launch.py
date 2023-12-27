from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    spawn_turtle = Node(
        package="turtlesim_game",
        executable="spawn_turtle"
    )

    #control_turtle = Node(
    #    package="turtlesim_game",
    #    executable="control_turtle"
    #)

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(spawn_turtle)
    #ld.add_action(control_turtle)
    
    return ld