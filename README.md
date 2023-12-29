# TurtleSimBuster
A game in Turtlesim where the main turtle goes and kills spawned turtles. This is a project I built to get familiar with ROS2 concepts and PID control.

TurtleSimBuster uses ROS2 services to spawn and kill the turtles. It uses subscribers to get the pose of the spawned turtles and the leader turtle. Currently, this uses the P controller to calculate the linear.x and angular.z velocities. Those are then published on the cmd_vel topic.

## Demo

<p align="center">
  <img src="https://github.com/SurabhiGupta17/TurtleSimBuster/blob/main/assets/Demo.gif" alt="Demo">
</p>

