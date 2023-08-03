# ROS2 Fleet Template

Welcome to Robotic Simulation Fleet template for ROS 2 projects!
This project is a template designed to help you quickly start simulating robots in Open 3D Engine (O3DE) with Robot Operating System (ROS) 2.

This template focuses on use-cases with multiple robots in environments such as warehouses and fulfillment centers.
The warehouse assets used in the template are modular and well-suited for reconfiguration to match a custom warehouse.
If your simulation does not match this and you would like to start with a simple and versatile template, consider using a more generic template:
- [ROS 2 Project Template](https://github.com/o3de/o3de-extras/tree/development/Templates/Ros2ProjectTemplate).

![Template picture](Screenshots/fleet_template.png)

## Set up a new project with the template

Please follow the instructions in [ROS 2 Gem documentation](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/project-configuration/)
to install all required dependencies and create your project with a template (make sure to use chose this template during the process).

## Fleet navigation

This template comes with the example fleet navigation ROS 2 package called `o3de_fleet_nav`. You can find a prepared ROS 2 workspace in the `Examples` directory.

This package contains a modified code from `nav2_bringup` package (https://github.com/ros-planning/navigation2).

In this example, a fleet of robots is automatically spawned and each individual robot can be controlled via the Rviz2. An AMCL localization is used for robot localization.

### Fleet configuration

You can configure the fleet by modifying `Example/ros2_ws/src/o3de_fleet_nav/config/fleet_config.yaml` file:

```
fleet:
  - robot_name: proteus
    robot_namespace: robot1
    position: 
      x: -6.0
      y: 0.5
      z: 0.2
  - robot_name: proteus
    robot_namespace: robot2
    position: 
      x: -6.0
      y: 7.5
      z: 0.2
  - robot_name: proteus
    robot_namespace: robot3
    position: 
      x: -6.0
      y: -6.0
      z: 0.2
```

This configuration file contains the data about each robot in a fleet:
- name (a type of the robot to spawn), 
- namespace (must be unique per spawned robot),
- spawning position (spawn position is also used as an AMCL initial estimation). 

In this example, only the `proteus` robot is supported.

You can modify contents of this file to add/remove robots or change their initial positions.

> Notice: You have to rebuild the ROS 2 workspace for changes to update.

### Topics and frames

Every spawned robot will have its own namespace for all topics. For the first robot ('robot1' namespace), these will be:

- `/robot1/cmd_vel`: The topic to [control the robot](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/concepts-and-components-overview/#robot-control).
- `/robot1/scan` - The topic of simulated 2D laser scanner sensor.

The first spawned robot also provides the following transformations:

- `/robot1/odom`
- `/robot1/base_link`
- `/robot1/lidar`

To understand more about transformations, see ROS 2 navigation [documentation](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html).

### Building

- Source ROS 2:
```
. /opt/ros/humble/setup.bash
```

- Go to the ROS 2 workspace:
```
cd Example/ros2_ws
```

- Install ROS 2 dependencies:
```
rosdep update
rosdep install --from-paths src -y --ignore-src
```

- Build workspace:
```
colcon build --symlink-install
```

### Running

- Run the `Warehouse` level in O3DE editor.

- Source ROS 2 and the `o3de_fleet_nav` workspace:
```
. /opt/ros/humble/setup.bash
```

- Source the `o3de_fleet_nav` workspace:
```
cd Example/ros2_ws
. ./install/setup.bash
```

- Run the fleet example:
```
ros2 launch o3de_fleet_nav o3de_fleet_nav_launch.py
```

Few Rviz2 windows should appear. You can use the "Nav2 Goal" button to send goal to the robot.

![Rviz2](Screenshots/fleet_rviz.png)
