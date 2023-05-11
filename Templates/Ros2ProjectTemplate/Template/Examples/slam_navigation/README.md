# ROS 2 Project example

With this example you will learn how to run ROS 2 navigation with a project created with this template.

These instructions assume that you have created and built your project following the [Template README](https://github.com/o3de/o3de-extras/tree/development/Templates/Ros2ProjectTemplate/README.md).

[ROS 2 navigation stack](https://navigation.ros.org/) is a set of packages which can deliver robot navigation almost out of the box following configuration for your specific robot.
In other cases, it can serve as a great basis to build your custom robot navigation.

## How to run the example

### Install navigation packages

To run the navigation example with your project, you need to install ROS 2 navigation stack packages.

```shell
sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan
```

### Start the simulation 

1. Run your project in O3DE editor and open the only level.
1. Start the simulation by clicking the `Play Game` button or pressing `CTRL+G`

### Launch the navigation stack

Open a bash console terminal (always make sure your [ROS 2 is sourced](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/project-configuration/#ros-2-ecosystem)!) and run:

```shell
cd #{O3DE_EXTRAS_HOME}/Templates/Ros2ProjectTemplate/Template/Examples/slam_navigation/launch
ros2 launch navigation.launch.py
```
You should see RViz2, a ROS 2 visualisation tool.

### Set the goal 

In RViz2, set robot target goal by using the `2D Goal Pose` tool (upper toolbar). 
The robot in your simulation should be on its way to the goal! You will also notice it is building a map.
