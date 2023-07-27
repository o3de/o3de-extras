# ROS2  Robotic Manipulation Template

Welcome to the Robotic manipulation template.
This project is a template designed to help you quickly start simulating robotic arms in Open 3D Engine (O3DE) with Robot Operating System (ROS) 2.

This template focuses on robotic arm manipulation with ROS 2 and [MoveIt 2](https://moveit.picknik.ai/main/index.html).


## Set up a new project with the template

Please follow the instructions in [ROS 2 Gem documentation](https://development--o3deorg.netlify.app/docs/user-guide/interactivity/robotics/project-configuration/)
to install all required dependencies and create your project with a template (make sure to use chose this template during the process).


## Using simulated Panda Franka manipulator with RGBD camera

[Panda Franka Research 3](https://www.franka.de/) is a popular choice in AI and manipulation research in the robotic community.
The simulated robot is imported from URDFs available [panda_description](https://github.com/ros-planning/moveit_resources/tree/humble/panda_description)

![Panda o3de](Screenshots/RoboticManipulation.png)

The level contains a robot and some small objects to try manipulation with. 

Install necessary packages:
```
sudo apt-get install ros-humble-moveit ros-humble-moveit-resources ros-humble-depth-image-proc
```
And run the launch file in the project:

```
ros2 launch Examples/panda_moveit_config_demo.launch.py
```

You should be able to use `Motion Planning` from `moveit_ros_visualization package` : 
![Panda rviz](Screenshots/RoboticManipulationRviz.png)

To understand more about Move It and robotic manipulation, see [tutorials](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html#getting-started).