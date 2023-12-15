[![Apache License, Version 2.0][apache_shield]][apache]

# Warehouse Automation Gem for Open 3D Engine (O3DE)

## Introduction

This Gem contains a set of models and prefabs with underlying O3DE components that can be used to create a warehouse project suitable for robotic simulations. More details about such simulations in [Open 3D Engine](https:://o3de.org) can be found in [Robot Operating System (ROS) documentation](https://docs.ros.org/en/rolling/index.html) and [ROS2 Gem documentation](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2). The Gem was implemented primarily for [ROSCon2023Demo](https://github.com/RobotecAI/ROSCon2023Demo) project and it was separated for easier reuse in other works.

Please refer to [O3DE documentation](https://docs.o3de.org/docs/user-guide/gems/) to learn more about Gems and about registering Gems in the system and O3DE projects.

## Requirements

- Any O3DE project with the [O3DE ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2) enabled.

## Description

This Gem delivers a conveyor belt simulation and a proximity sensor that can be used to control the belt. E.g., it can be used to stop to belt when transported goods reach a certain position on a belt. The conveyor belt is fragmented into smaller pieces which can be put together in any configuration. Additionally, this Gem delivers an asset with a sample palletizer configuration, in which one conveyor belt splits into two pick-up stations for robotic arms and an extra roller conveyor for putting aside goods.

The following assets are available in the Gem as O3DE prefabs:
- `./Assets/Factory/ConveyorLine.prefab` - a sample palletizer configuration
- `./Assets/Factory/ConveyorElements/ConveyorSegments_Turnout.prefab` - conveyor belt's element that splits into two and allows for goods distribution using a mechanized flap
- `./Assets/Factory/ConveyorElements/ConveyorSegments_Straight.prefab` - conveyor belt's straight element
- `./Assets/Factory/ConveyorElements/ConveyorSegments_Turn_90.prefab` - conveyor belt's turn element (90 degrees)
- `./Assets/Factory/ConveyorElements/ConveyorSegments_End.prefab` - conveyor belt's end element with an extra flap to keep transported goods positioned

Images of all prefabs are given in the next section.

The proximity sensor is an O3DE component with no corresponding visualization. It uses `AzPhysics::RayCastRequest` to check for collisions in its range and returns continuously the result of this test via `ProximitySensorNotificationBus`. Any O3DE component can connect to this bus and react to certain events. The sensor and the bus are also available for scripting in [ScriptCanvas](https://docs.o3de.org/docs/user-guide/scripting/script-canvas/). It is possible to set the frequency and the range of the sensor. The visualization beam can be disabled.

## Screenshots

### Conveyor belt parts
![](docs/images/conveyor.png)
![](docs/images/conveyor2.png)
From left: `ConveyorSegments_End.prefab`, `ConveyorSegments_Turn_90.prefab`, `ConveyorSegments_Turnout.prefab`, and `ConveyorSegments_Straight.prefab`

### Sample palletizer configuration
![](docs/images/palletizer.png)
![](docs/images/palletizer2.png)
Sample palletizer configuration with conveyor belt split into two pick-up stations (no robotic arms available in this Gem) and a roller conveyor for pallets.

### Proximity sensor
![](docs/images/proximity.png)
Three instances of proximity sensors with a cube mesh primitive used for visualization. Note different range configurations between the left and the middle sensors. The beam of the right sensor is green due to the detection of a sample box.

## Acknowledgments
This work is licensed under [Apache License, Version 2.0][apache]. You may elect at your option to use the [MIT License][mit] instead. Contributions must be made under both licenses.

[apache]: https://opensource.org/licenses/Apache-2.0
[mit]: https://opensource.org/licenses/MIT
[apache_shield]: https://img.shields.io/badge/License-Apache_2.0-blue.svg