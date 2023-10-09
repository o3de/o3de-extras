# O3DE ROS2 Gem

With the ROS2 Gem you can develop robotic simulations with [ROS (Robot Operating System) 2](https://www.ros.org/) in Open 3D Engine (O3DE).

Access the the whole ROS2 Gem documentation by clicking [here](https://www.o3de.org/docs/user-guide/interactivity/robotics/). Other sections in this README link to specific parts of the provided documentation.

## Features

* Direct and natural support of ROS2 ecosystem:
  * No bridges. Your simulation node will function as any other ROS2 node.
    * This is also good for performance
  * Easy way to include ROS2 dependencies.
* Sensors:
  * Sensor Component serving as a handy abstraction.
  * Example implementations of Lidar, Camera, IMU sensors.
    * Including a few Assets and prefabs which are ready to use. 
* Automated handling of:
  * Simulation time, publishing `/clock` supporting non-real time.
  * Publishing of transformation frames (`/tf`, `/tf_static`).
  * Validation for topic and namespace names.
* Robot Control Component:
  * A quick to use method of controlling your robot with Twist messages.
  * Can be used with custom LUA scripting. 

For a "feel" of these features, see an [example project](#example-project) which uses this Gem to run navigation stack.

## Example project

You can test the O3DE ROS2 Gem with the [Robot Vacuum](https://github.com/o3de/RobotVacuumSample) project. It will allow you to run a robot navigation simulation. All necessary assets are included.

## Installation

Refer to [this part of the documentation](https://www.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/) to configure your project with the ROS2 Gem.


## Learn about ROS2 Gem

Visit [this part of the documentation](https://www.o3de.org/docs/user-guide/interactivity/robotics/concepts-and-components-overview/) to learn about the ROS2 Gems Concepts and Structure. Additionally you can learn about ROS 2 Concepts [here](https://docs.ros.org/en/humble/Concepts.html).

## How to create your own robotic simulation

Refer [here](https://www.o3de.org/docs/user-guide/interactivity/robotics/creating-robotic-simulation/) to access a list of steps guiding you through the creation of your own simulation.

## Importing robots

Access [this part](https://www.o3de.org/docs/user-guide/interactivity/robotics/importing-robot/) of the documentation to learn about importing robots from common formats such as the [Unified Robot Description Format (URDF)](http://wiki.ros.org/urdf). 

## Troubleshooting

If you have any problems with the ROS2 Gem, visit [here](https://www.o3de.org/docs/user-guide/interactivity/robotics/troubleshooting/) for troubleshooting guides and suggestions.

## User Guides

If you plan on contributing please follow the [Pull Request Publishing](docs/guides/pr_publishing.md) guide. For those using the Clion IDE we advise to follow the [Development in Clion](docs/guides/development_in_clion.md) guide.

## Acknowledgements

This project was originally developed by [Robotec.ai](https://robotec.ai) in cooperation with [AWS Game Tech](https://aws.amazon.com/gametech/) and [AWS RoboMaker](https://aws.amazon.com/robomaker/).
