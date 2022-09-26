
## ROS 2 Gem

The Gem and its build instructions can be found in [this repository](https://github.com/RobotecAI/o3de-ros2-gem).
The ROS 2 Gem helps to build robotic simulations with [ROS 2 / Robot Operating System](https://www.ros.org/).
For an example of use see [Warehouse Demo Project](https://github.com/RobotecAI/Ros2WarehouseDemo).

## Doxygen documentation

You can browse Doxygen-generated documentation on [Gem's GitHub page](https://robotecai.github.io/o3de-ros2-gem/).

## Components overview

- __Central Singleton__
  - ROS2SystemComponent
- __Core abstractions__
  - ROS2FrameComponent
  - ROS2SensorComponent
- __Sensors__
  - ROS2CameraComponent
  - ROS2GNSSComponent
  - ROS2IMUComponent
  - ROS2LidarComponent
- __Robot control__
  - ROS2RobotControlComponent
- __Spawner__
  - ROS2SpawnerComponent
  
## The Gem and ROS 2 ecosystem

### Supported Platforms and versions

The Gem is currently Linux-only and is being tested with ROS 2 Humble on Ubuntu 22.04 as well as ROS 2 Galactic with Ubuntu 20.04.

It is intended to support any modern ROS 2 version, following these priorities:
- The most recent LTS version (e.g. in June 2022, [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)).
- The most recent non-LTS version ([ROS 2 Galactic](https://docs.ros.org/en/galactic/Installation.html)).
- The older LTS version ([ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation.html)).
- The always-fresh [ROS 2 Rolling](https://docs.ros.org/en/rolling/Installation.html).
- Older versions.

Currently tested and validated versions / platforms will be detailed in the [project repository](https://github.com/RobotecAI/o3de-ros2-gem).

If you have multiple versions installed, make sure you [source](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay) the one you want to use.
You can check which version is sourced in your console by checking the value of `ROS_DISTRO` environment variable (`echo $ROS_DISTRO`).

### ROS 2 Concepts

Please refer to [ROS 2 Concepts documentation](https://docs.ros.org/en/humble/Concepts.html) if you are not familiar with how ROS 2 works.

### Structure and Communication

The Gem creates a [ROS 2 node](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Nodes.html) which is directly a part of ROS 2 ecosystem. As such, your simulation will not use any bridges to communicate and is subject to configuration through settings such as Environment Variables. It is truly a part of the ecosystem.

Note that the simulation node is handled through `ROS2SystemComponent` - a singleton. However, you are free to create and use your own nodes if you need more than one.

Typically, you will be creating publishers and subscriptions. This is done through [rclcpp API](https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1Node.html). Example:

```
auto ros2Node = ROS2Interface::Get()->GetNode();
AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), m_MyTopic);
m_myPublisher = ros2Node->create_publisher<sensor_msgs::msg::PointCloud2>(fullTopic.data(), QoS());
```

Note that QoS class is a simple wrapper to [rclcpp::QoS](https://docs.ros2.org/galactic/api/rclcpp/classrclcpp_1_1QoS.html). 

#### Frames

`ROS2FrameComponent` is a representation of an interesting physical part of the robot.
It handles spatio-temporal relationship between this part and other frames of reference.
It also encapsulates namespaces, which help to distinguish between different robots and different parts of the robot,
such as in case of multiple identical sensors on one robot.

All Sensors and the Robot Control Component require `ROS2FrameComponent`.

#### Sensors

Sensors are Components deriving from `ROS2SensorComponent`. They acquire data from the simulated environment and publish it to ROS 2 domain.
- Each sensor has a configuration, including one or more Publishers.
- Sensors publish at a given rate (frequency).
- Some sensors can be visualised.

If you intend to add your own sensor, it might be useful to look at how sensors already provided within the Gem are implemented.

#### Robot Control

The Gem comes with `ROS2RobotControlComponent`, which you can use to move your robot through [Twist](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/msg/Twist.msg) messages.
The component subscribes to these messages on a configured topic. 
You can use tools such as [rqt_robot_steering](https://index.ros.org/p/rqt_robot_steering/#galactic) to move your robot with Twist messages.

It is possible to implement your own control mechanisms with this Component.

#### Spawner

`ROS2SpawnerComponent` handles spawning entities during simulation.
Available spawnables have to be set up as the component's field before the simulation.

During the simulation user can access names of available spawnables and request spawning using ros2 services. The names of services are "/get_available_spawnable_names" and "/spawn_entity" respectivly. GetWorldProperties.srv and SpawnEntity.srv types from gazebo_msgs are used to handle these features.

- Spawning: spawnable name should be passed in request.name and the position of entity in request.initial_pose
  - example call: `ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'robot', initial_pose: {position:{ x: 4, y: 4, z: 0.2}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'`
- Available spawnable names access: names of available spawnables are sent in response.model_names
  - example call: `ros2 service call /get_available_spawnable_names gazebo_msgs/srv/GetWorldProperties`

### Handling custom ROS 2 dependencies

The ROS 2 Gem will respect your choice of [__sourced__](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html#source-the-overlay) ROS 2 environment.
The Gem comes with a number of ROS 2 packages already included and linked, but you might want to include additional packages in your project. To do so, use the `target_depends_on_ros2` function:

```
target_depends_on_ros2_packages(<your_target> <ros_package1> <ros_package2>)
```

in your project's `Gem/CMakeLists.txt`.

##### Example

It could be the case that you need to create new type of sensor publishing a custom message.

Lets assume your project is called `MyProject`, the custom message package is called `my_sensor_msgs` and ROS 2 workspace
`my_ros2_ws`.   Take following steps:
1. Build your ROS 2 message package in a workspace as you normally would (e.g. `~/projects/my_ros2_ws`)
2. Source the overlay: `source ~/projects/my_ros2_ws/install/setup.bash`.
3. Put `target_depends_on_ros2_packages(MyProject my_sensor_msgs)` in your `Gem/CMakeLists.txt` file.
4. You can now build `MyProject` and use the new messages.

Remember to __always have your ROS 2 overlay sourced__ when building and running the project as sourcing provides visibility of ROS 2 package paths.

## Diagram of classes

Some classes with relationships and functions are presented on this diagram. Specific sensor classes (e.g. Lidar) are not included.

![classes diagram](static/diagrams/diagram_ros2_gem.png)
