# TurtleBot3 Demos

A few simple demonstrations exploring the capabilities of the TurtleBot3 platform (with OpenManipulator X arm).

## Dependencies

- Ubuntu 16.04
- ROS Kinetic
- Gazebo7
- [turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/#pc_setup) ROS packages
- [openmanipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_setup/#install-ros-packages) ROS packages

## Usage

- Install the dependencies above
- Clone this repo to `~/catkin_ws/src`
- Set turtlebot model environment variable in `~/.bashrc`: e.g. `export TURTLEBOT3_MODEL="waffle_pi"`
- Follow the examples below

## Startup

### In Simulation

Launch the turtlebot Gazebo simulation:

`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

Alternatively, launch a simulation with a robot arm:

`roslaunch open_manipulator_with_tb3_gazebo rooms.launch use_platform:=false paused:=false`

See [the turtlebot manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-gazebo) for different possible worlds.

### In Real Life

Connect the Turtlebot and the RemotePC to the same wifi network. 

Make sure ROS environment variables are set correctly in `~/.bashrc`

[On the Turtlebot]
```
export ROS_MASTER_URI=http://REMOTE_IP:11311
export ROS_HOSTNAME=TURTLE_IP
```

[On the RemotePC]
```
export ROS_MASTER_URI=http://REMOTE_IP:11311
export ROS_HOSTNAME=REMOTE_IP
```

[On the RemotePC] Start ROS

`roscore`

[On the Turtlebot] Run packages with basic functionality

`roslaunch turtlebot3_bringup turtlebot3_robot.launch`

## Examples

### Basic Operation

Control the turtlebot by publishing to the `/cmd_vel` topic:

`rosrun turtlebot3_teleop turtlebot3_teleop_key`

### Building a Map with SLAM 

Launch SLAM (this opens RVIZ):

`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`

Drive around with the keyboard:

`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

When the map looks good, save the map to the specified location:

`rosrun map_server map_saver -f ~/catkin_ws/src/tbot3_demos/maps/turtle_world_map`

This saves two files, a pgm file and a yaml file, in the specified directory ("maps" above). 

### Navigation in a Known Map

Start the ROS navigation stack with the corresponding saved map file:

`roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/catkin_ws/src/tbot3_demos/maps/turtle_world_map.yaml`

Now use the RVIZ interface to set an initial pose estimate and send navigation goals. Note that this corresponds to publishing on `/initialpose` and `/move_base/goal`.

### Navigation with Unknown Map via SLAM

Launch SLAM and navigation jointly. This essentially replaces a static map from a file with a map that
is updated in real-time from SLAM.

`roslaunch tbot3_demos turtlebot3_slam_navigation.launch slam_methods:=gmapping`

Now use the RVIZ interface to set an initial pose estimate and send navigation goals.

### Using a Manipulator Arm

Start the robot arm controller:

`roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false use_robot_name:=om_with_tb3`

Launch a GUI to send commands to the robot arm:

`roslaunch open_manipulator_control_gui open_manipulator_control_gui.launch robot_name:=om_with_tb3`

Note that you need to press "start timer" first. 

### Pick-and-place with Known Map

Launch navigation and moveit. Be sure to correct the map.yaml file provided in the open_manipulator_with_tb3_tools package first. 

`roslaunch open_manipulator_with_tb3_tools rooms.launch use_platform:=false`

Launch the task controller, which uses [smach](http://wiki.ros.org/smach) to manage high-level tasks. 

`roslaunch open_manipulator_with_tb3_tools task_controller.launch`

