# O3DE ROS2 Gem

This Gem enables users to develop robotic simulations through ROS2 tools and components.

Documentation about configuring and using this gem can be found [here](https://www.o3de.org/docs/user-guide/interactivity/robotics/).

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


## User Guides

If you plan on contributing please follow the [Pull Request Publishing](docs/guides/pr_publishing.md) guide. For those using the Clion IDE we advise to follow the [Development in Clion](docs/guides/development_in_clion.md) guide.

## Acknowledgements

This project was originally developed by [Robotec.ai](https://robotec.ai) in cooperation with [AWS Game Tech](https://aws.amazon.com/gametech/) and [AWS RoboMaker](https://aws.amazon.com/robomaker/).
