[![Apache License, Version 2.0][apache_shield]][apache]

# RosRobotSample Gem for Open 3D Engine (O3DE)

## A bit of context

* [Open 3D Engine](https:://o3de.org) - an open-source game & simulation engine. O3DE is extendable through modules called Gems. This is one of such Gems.
* [Robot Operating System (ROS)](https://docs.ros.org/en/rolling/index.html) - an open-source middleware and, de facto, standard for robotics.
* [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) - an open-source module for O3DE that enables simulation for robotics using modern ROS.
* [Husarion ROSbot XL](https://husarion.com/manuals/rosbot-xl/) - an autonomous mobile robot platform equipped with a 4x4 drive, allowing for trouble-free expansion with optional devices such as LIDAR, RGB-D cameras or a manipulator developed by [Husarion](https://husarion.com).

Please refer to [O3DE documentation](https://docs.o3de.org/docs/user-guide/gems/) to learn more about Gems and about registering Gems in the system and O3DE projects.

## Requirements
- Any O3DE project with the [O3DE ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) enabled.

## Description
This is an Asset Gem only. It contains the Husarion ROSbot XL asset stored as O3DE prefab: `ROSbot.prefab`. The robot is delivered as a ready-to-use prefab containing models, physics and the following ROS 2 components:
- `ROS2 Frame`
- `ROS2 Robot Control`
- `ROS2 Skid Steering Twist Control`

Additionally, the model is equipped with links that allow simple extension with `ROS2 Lidar Sensor` and `ROS2 Imu` sensors. There are two extra O3DE prefabs with 2D and 3D LiDAR sensors attached: `ROSBot_slamtec.prefab` and `ROSbot_velodyne.prefab` with Slamtec RPLIDAR S1 and Velodyne Puck VLP-16 sensors respectively. The components are visualized with the corresponding meshes. 

The robot publishes Lidar Sensor's output on the`/base_link/pc` ROS 2 topic and can be driven using the `/base_link/cmd_vel` ROS 2 topic. An example of its use can be found in [ROS2 Project Template](https://github.com/o3de/o3de-extras/tree/development/Templates/Ros2ProjectTemplate).

## Screenshots
![](docs/images/front.png)
From left: `ROSbot_velodyne.prefab`, `ROSBot_slamtec.prefab`, and `ROSbot.prefab`

![](docs/images/back.png)
From left: `ROSbot.prefab`, `ROSBot_slamtec.prefab`, and `ROSbot_velodyne.prefab`

## Acknowledgments
This work is licensed under [Apache License, Version 2.0][apache]. You may elect at your option to use the [MIT License][mit] instead. Contributions must be made under both licenses.

Models were created based on STL and URDF files kindly shared by Husarion.

[apache]: https://opensource.org/licenses/Apache-2.0
[mit]: https://opensource.org/licenses/MIT
[apache_shield]: https://img.shields.io/badge/License-Apache_2.0-blue.svg