# ROS2 Project navigation example

To run the navigation example with your project:

1. Make sure you have all `Ros2ProjectTemplate` requirements installed (see template README.md)

1. Make sure your project is based on `Ros2ProjectTemplate`

1. Install ROS2 packages:
```shell
sudo apt install ros-${ROS_DISTRO}-slam-toolbox ros-${ROS_DISTRO}-navigation2 ros-${ROS_DISTRO}-nav2-bringup ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-teleop-twist-keyboard
```

1. Run your project in O3DE editor

1. Open `DemoLevel`

1. Start the simulation by clicking the `Play Game` button or pressing `CTRL+G`

1. Source ROS2 (humble version is assumed):
```shell
. /opt/ros/humble/setup.bash
```

1. Start navigation stack:
```shell
cd examples/slam_navigation/launch
ros2 launch navigation.launch.py
```

1. Set robot target goal. Use RViz2 GUI to set the goal by using the `2D Goal Pose` tool (upper toolbar).
