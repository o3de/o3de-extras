# O3DE ROS2 Gem

This Gem enables users to develop robotic simulations through ROS2 tools and components.

## Requirements

> To be determined: supported versions

* [O3DE](https://www.o3de.org/)
* [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation.html) (not tested but could also work with Foxy and Humble). 
* Ubuntu 20.04.

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

## Building

The Gem is built through building a project which uses it. Make sure to
[source your ros2 workspace](https://docs.ros.org/en/rolling/Tutorials/Configuring-ROS2-Environment.html#source-the-setup-files)
before building.

## Adding Gem to your project

To use this Gem in your project, you need to register the Gem with O3DE. First, clone this repository. Then,
in o3de folder:
```
scripts/o3de.sh register --gem-path <PATH_TO_CLONED_ROS2_GEM>
scripts/o3de.sh enable-gem -gn ROS2 -pp <PATH_TO_YOUR_PROJECT>
```

## Example project

You can test O3DE ROS2 Gem with [this project](https://github.com/RobotecAI/Ros2WarehouseDemo).
It will allow you to run robot navigation. All necessary assets are included.

## User Guide

Follow the [ROS 2 Gem User Guide](docs/user-guide/ros2-gem.md) to understand its concepts and components.

## How to create your own robotic simulation

>This section is to be detailed.

Once you are set up and familiar with the example project, consider the following steps:
1. [Create a new O3DE project](https://www.o3de.org/docs/welcome-guide/create/) with [this Gem enabled](#adding-gem-to-your-project).
2. Create or import Assets for your robots and environment. 
   1. You can use formats supported by O3DE.
   2. It will be possible to import your robot from URDF in the future. The work is ongoing.
3. Determine which sensors you need to simulate. 
   1. Some sensors are already implemented in this Gem.
      1. They might require specialization (implementation specific for particular models).
      2. You might want to consider tradeoffs between performance and realism in each case.
   2. Use ROS2SensorComponent as a base class if you are implementing a new sensor.
4. Develop necessary sensors and their prefabs.
5. Consider developing additional abstraction to handle spawning and despawning robots.
   1. This would also be a valuable contribution to the Gem.
6. Develop your scene and simulation scenario, placing Assets and configuring Components.

Enjoy simulation with some of many [ROS2 packages](https://index.ros.org/packages/#galactic) and projects in [ROS2 ecosystem](https://project-awesome.org/fkromer/awesome-ros2).