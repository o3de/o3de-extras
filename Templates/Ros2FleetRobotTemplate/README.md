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

## Spawning robots

The level contains spawn points configured to easily add more Proteus robots through ROS 2 calls.
This is done with the [Spawner Component](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/concepts-and-components-overview/#spawner).
There are 4 spawn points already added in the level. You can use them all with the following service calls:

```shell
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint1'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint2'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint3'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint4'}'
```

## Topics and frames

Every spawned robot will have its own namespace for all topics. For the first robot, these will be:

- `/proteus_1/cmd_vel`: The topic to [control the robot](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/concepts-and-components-overview/#robot-control).
- `/proteus_1/pc` - The topic of simulated LIDAR point cloud.

The first spawned robot also provides the following transformations:

- `/proteus_1/odom`
- `/proteus_1/base_link`
- `/proteus_1/lidar`

To understand more about transformations, see ROS 2 navigation [documentation](https://navigation.ros.org/setup_guides/transformation/setup_transforms.html).
