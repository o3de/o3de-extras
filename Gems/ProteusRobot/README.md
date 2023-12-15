[![Apache License, Version 2.0][apache_shield]][apache]

# ProteusRobot Gem for Open 3D Engine (O3DE)

## Introduction

This Gem contains a set of models and prefabs suitable for robotic simulations. More details about such simulations in [Open 3D Engine](https:://o3de.org) can be found in [ROS2 Gem documentation](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2). The Gem was implemented primarily for [ROS2 Project Template](https://github.com/o3de/o3de-extras/tree/development/Templates/Ros2ProjectTemplate) and it was separated for easier reuse in other works.

Please refer to [O3DE documentation](https://docs.o3de.org/docs/user-guide/gems/) to learn more about Gems and about registering Gems in the system and O3DE projects.

Please refer to [O3DE documentation](https://docs.o3de.org/docs/user-guide/gems/) to learn more about Gems and about registering Gems in the system and O3DE projects.

## Requirements
- Any O3DE project with the [O3DE ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) enabled.

## Description
This is an Asset Gem. It contains the [Proteus Robot](https://robotsguide.com/robots/proteus) - an autonomous mobile robot that can pick up, transport, and drop off containers in [Amazon](https://www.aboutamazon.com) fulfillment facilities. It is delivered as a ready-to-use O3DE prefab, `Proteus.prefab`, containing visual models, physics and the following ROS 2 components:
- `ROS2 Frame`
- `ROS2 Robot Control`
- `ROS2 Skid Steering Twist Control`
- `ROS2 Lidar Sensor`

Additionally, the model is equipped with links that allow simple extension with `ROS2 Camera` and `ROS2 Imu` sensors.

The robot publishes Lidar Sensor's output on the`/base_link/pc` ROS 2 topic and can be driven using the `/base_link/cmd_vel` ROS 2 topic. 

## Screenshots
![](docs/images/front.png)
![](docs/images/back.png)

## Acknowledgments
This work is licensed under [Apache License, Version 2.0][apache]. You may elect at your option to use the [MIT License][mit] instead. Contributions must be made under both licenses.

[apache]: https://opensource.org/licenses/Apache-2.0
[mit]: https://opensource.org/licenses/MIT
[apache_shield]: https://img.shields.io/badge/License-Apache_2.0-blue.svg