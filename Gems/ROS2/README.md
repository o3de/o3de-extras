# O3DE ROS 2 Gem

With the ROS 2 Gem you can develop robotic simulations with [ROS (Robot Operating System) 2](https://www.ros.org/) in [Open 3D Engine (O3DE)](https://o3de.org).

### Platform
The Gem is tested with ROS 2 Humble and Iron on Ubuntu 22.04. ROS 2 Jazzy support will be available on distro release day. 

>> The Gem is not available for Windows and does not support ROS 1.

## Installation

Refer to [this part of the documentation](https://www.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/) to learn how to create a new project with the ROS 2 Gem.

## Documentation

A great place to start learning about this Gem is the [O3DE Robotics Documentation](https://www.o3de.org/docs/user-guide/interactivity/robotics/).

## Features

* Direct and natural support of ROS 2 ecosystem:
  * No bridges. Your simulation node will function as any other ROS 2 node. You can write ROS 2 code in simulation! 
    * This is also good for performance.
  * Easy way to include ROS 2 dependencies.
  * "Just works" with custom messages, services and actions.
* Sensors:
  * Customizable sensors: Lidar3D, Lidar2D, Camera, IMU, Odometry, GNSS, Contact.
    * Easy to implement other sensors through handy abstractions.
* Robot control and dynamics components:
  * A quick to use method of controlling your mobile robot with Twist or Ackermann messages.
  * Control and state components for joint systems, such as robotic manipulator arms.
  * Vacuum and finger grippers.
* Robot Importer feature working with `URDF` and `SDFormat` (including `.xacro` and `.world` files)
  * Plugin support which allows you to import robots with sensors already there!
* Automated handling of:
  * Simulation time, publishing `/clock` supporting non-real time.
  * Publishing of transformation frames (`/tf`, `/tf_static`).
  * Validation for topic and namespace names.

For a "feel" of these features, see an [example project](#example-project) which uses this Gem to run navigation stack.

## Importing robots

Please refer to [Robot Importer documentation](https://www.o3de.org/docs/user-guide/interactivity/robotics/importing-robot/).

## Troubleshooting

See [Troubleshooting guide](https://www.o3de.org/docs/user-guide/interactivity/robotics/troubleshooting/).

## User Guides

If you plan on contributing please follow the [Pull Request Publishing](docs/guides/pr_publishing.md) guide. For those using the Clion IDE we advise to follow the [Development in Clion](docs/guides/development_in_clion.md) guide.

## Acknowledgements

This project was originally developed by [Robotec.ai](https://robotec.ai) in cooperation with [AWS Game Tech](https://aws.amazon.com/gametech/) and [AWS RoboMaker](https://aws.amazon.com/robomaker/).
