# Manipulation in O3DE - implementation roadmap

This roadmap outlines the implementation steps for the light version of the
manipulation diagram:

![manipulation_light](https://user-images.githubusercontent.com/48955695/201618734-d8998631-1472-4326-822e-c5ddd66db903.svg)

## Mesh models and configurations

- Model the arm and end-effector to be loaded into the simulation and MoveIt.
- Model the collision geometry, using simple primitives to approximate the arm
  and end-effector.
- Set up MoveIt configuration (SRDF, etc.) for the robot.
  See [`panda_ign_moveit2`](https://github.com/AndrejOrsula/panda_ign_moveit2)
  for an example of an arm model and MoveIt 2 configuration.

## Object detection

- If using camera to do actual detection (as opposed to using ground truth)
  - Choose camera configuration - RGB, depth, resolution, FOV, etc.
  - Set up O3DE [`ROS2SensorComponent`](https://github.com/RobotecAI/o3de-ros2-gem/blob/de2d1cdfa68310c6c0d885d65c0d62fa972a4a69/Code/Include/ROS2/Sensor/ROS2SensorComponent.h).
    Choose camera transport topic name and type (
    [`sensor_msgs/*`](https://docs.ros2.org/latest/api/sensor_msgs/index-msg.html))
    for communication with ROS.
  - Set up ROS node for object detection, using OpenCV, for example.
- Determine ROS topic and type (e.g.
  [`geometry_msgs/PoseStamped`](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)),
  to publish object pose in simulator to ROS.

## Motion planning

- Scripting
  - Calculate end-effector goal pose based on object pose in simulator.
    Publish end-effector goal pose for MoveIt.
- Create `ROS2JointStatePubComponent` to obtain joints information from
  simulation and publish a message of
  [`sensor_msgs/JointState`](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
  type on topic `/joint_states`, to be consumed by MoveIt and the controller.
- Use MoveIt interface (`moveit::planning_interface::MoveGroupInterface`) to
  set the goal pose and to plan a trajectory.

## Motion execution and simulation

- Interface to communicate the joint trajectory from MoveIt 2 with the
  controller and the simulation.
  - If using [`ros2_control`](https://github.com/ros-controls/ros2_control),
    an `o3de_ros2_control` can be modeled after 
    [`ign_ros2_control`](https://github.com/ros-controls/gz_ros2_control/tree/master/ign_ros2_control),
    which communicates between the Gazebo simulator, ROS 2, and MoveIt 2.
  - The main interface needed is documented in a Gazebo example
    [`ign_moveit2_examples`](https://github.com/j-rivero/ign_moveit2_examples/pull/1).
  - For starters, a new `ros2_velocity_controller` can be a minimal
    implementation of `o3de_ros2_control`, with only the velocity controller.
  - If using native O3DE controllers, then some interface should communicate
    between O3DE, ROS 2, and MoveIt 2.
- Create `ROS2PoseTrajectoryComponent` to receive the joint trajectory and
  execute the trajectory in simulation, where collisions, contacts, and dynamics
  are simulated.
