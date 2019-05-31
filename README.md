# TurtleBot3 Demos

A few simple demonstrations exploring the capabilities of the TurtleBot3 platform (with OpenManipulator X arm).

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- Gazebo7
- [turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc_setup) ROS packages
- [openmanipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_setup/#install-ros-packages) ROS packages

## Usage

- Clone this repo to `~/catkin_ws/src`
- Set turtlebot model environment variable in `~/.bashrc`: `export TURTLEBOT3_MODEL="waffle_pi"`
- Follow the examples below

## Simulation Examples

### Basic Operation

Launch the turtlebot Gazebo simulation in an empty world:

`roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`

Control the turtlebot by publishing to the `/cmd_vel` topic:

`rosrun turtlebot3_teleop turtlebot3_teleop_key`

See [the turtlebot manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo) for different possible worlds.

### Building a Map with SLAM 

Launch the Gazebo simulation:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Launch SLAM (this opens RVIZ):

`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`

Drive around with the keyboard:

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

When the map looks good, save the map to the specified location:

`rosrun map_server map_saver -f ~/catkin_ws/src/tbot3_demos/maps/turtle_world_map`

This saves two files, a pgm file and a yaml file, in the specified directory ("maps" above). 

### Navigation in a Known Map

Launch the Gazebo simulation:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Start the ROS navigation stack with the corresponding saved map file:

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/tbot3_demos/maps/turtle_world_map.yaml`

Now use the RVIZ interface to set an initial pose estimate and send navigation goals. Note that this corresponds to publishing on `/initialpose` and `/move_base/goal`.

### Navigation with Unknown Map via SLAM

Launch the Gazebo Simulation:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Launch SLAM and navigation jointly. This essentially replaces a static map from a file with a map that
is updated in real-time from SLAM.

`roslaunch tbot3_demos turtlebot3_slam_navigation.launch slam_methods:=gmapping`

Now use the RVIZ interface to set an initial pose estimate and send navigation goals.

### Using a Manipulator Arm

Launch a Gazebo simulation that includes the robot arm:

`roslaunch open_manipulator_with_tb3_gazebo empty_world.launch paused:=false`

Start the robot arm controller:

`roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false use_robot_name:=om_with_tb3`

Launch a GUI to send commands to the robot arm:

`roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch robot_name:=om_with_tb3`

Note that you need to press "start timer" first. 


### Pick-and-place with Known Map

Launch Gazebo simulation:

`roslaunch open_manipulator_with_tb3_gazebo rooms.launch use_platform:=false paused:=false`

Launch navigation and moveit. Be sure to correct the map.yaml file provided in the open_manipulator_with_tb3_tools package first. 

`roslaunch open_manipulator_with_tb3_tools rooms.launch use_platform:=false`

Launch the task controller, which uses [smach](http://wiki.ros.org/smach) to manage high-level tasks. 

`roslaunch open_manipulator_with_tb3_tools task_controller.launch`

