# ROS2 Fleet Robot Template

A project template that allows building a multi-robot ROS2-enabled simulation. The template contains a level with a large warehouse scene called `Warehouse.prefab`. 

## Requirements

Due to ROS2 dependency, this project was prepared and tested on Linux operating system. It is recomended to use Ubuntu [22.04](https://www.releases.ubuntu.com/22.04/), however any distribution meeting following requirements may be used.

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met. 
This project has the following dependencies:

- [O3DE](https://github.com/o3de/o3de)
- [ROS2 Humble](https://www.ros.org)
- [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2)
- [Proteus robot](https://github.com/o3de/o3de-extras/tree/development/Gems/ProteusRobot)
- [WarehouseAssets](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAssets)

Please make sure that `clang` was installed and configured. For details refer to [this section](https://www.o3de.org/docs/welcome-guide/requirements/#linux) of O3DE documentation.

## Setup Instructions

The following steps will assume the following:

- You have ROS2 humble [installed](https://docs.ros.org/en/humble/Installation.html) and environment is [sourced](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).
- The O3DE gems, projects and templates will be placed in the O3DE home folder: `/home/${USER}/O3DE`.
- You have the O3DE engine [downloaded and built](https://www.o3de.org/docs/welcome-guide/setup/setup-from-github) and it is located in `/home/${USER}/O3DE/Engines/Development`.

In this tutorial, CLI tools will be used. It is also possible to use O3DE GUI to set up a project from template. See the [O3DE Project manager documentation](https://www.o3de.org/docs/user-guide/project-config/project-manager/) for more details. In such cases it is required to source your ROS2 distro before launching O3DE manager:

```shell
source /opt/ros/humble/setup.bash
./build/linux/bin/profile/o3de
```

### 1. Install ROS2 packages

```shell
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-control-toolbox
```

### 2. Download the template and asset gems

For convenienience, we'll define a shell variables with O3DE folders:

```shell
export O3DE_HOME=${HOME}/O3DE
```

Clone the `o3de-extras` repository containing the template and asset gems

```shell
mkdir -p ${O3DE_HOME}/Projects
cd ${O3DE_HOME}/Projects
git clone https://github.com/o3de/o3de-extras.git 
cd o3de-extras 
git lfs install && git lfs pull
```

Copy gems and template to the O3DE home.

```shell
mkdir -p ${O3DE_HOME}/Gems
mkdir -p ${O3DE_HOME}/Templates
cp -r ${O3DE_HOME}/Projects/o3de-extras/Gems/ROS2 ${O3DE_HOME}/Gems \
cp -r ${O3DE_HOME}/Projects/o3de-extras/Gems/WarehouseAssets ${O3DE_HOME}/Gems \
cp -r ${O3DE_HOME}/Projects/o3de-extras/Gems/ProteusRobot ${O3DE_HOME}/Gems \
cp -r ${O3DE_HOME}/Projects/o3de-extras/Templates/Ros2FleetRobotTemplate ${O3DE_HOME}/Templates/
```

Register these gems and project template.

```shell
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/ROS2
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/WarehouseAssets
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/ProteusRobot
${O3DE_HOME}/scripts/o3de.sh register --template-path ${O3DE_HOME}/Templates/Ros2FleetRobotTemplate 
```

### 3. Create a ROS2 project from the template

Assign a name for the new project. In this example, it is assumed that it will be: `Ros2FleetRobotTest`, and it will be located in `${O3DE_HOME}/Projects/Ros2FleetRobotTest` folder. 

```shell
export PROJECT_NAME=Ros2FleetRobotTest
export PROJECT_PATH=${O3DE_HOME}/Projects/${PROJECT_NAME}
${O3DE_HOME}/scripts/o3de.sh create-project --project-path $PROJECT_PATH --template-name Ros2FleetRobotTemplate
```

### 4. Build the project

Next, let us build the project with necessary elements of the O3DE engine and ROS2 Gem.

```shell
cd $PROJECT_PATH
source /opt/ros/humble/setup.bash
cmake -B build/linux -G "Ninja Multi-Config" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF
cmake --build build/linux --config profile --target $PROJECT_NAME.GameLauncher Editor
```

### 5. Launch Editor

Finally, the O3DE with preloaded gems can be run:

```shell
$PROJECT_PATH/build/linux/bin/profile/Editor
```

## Spawning robots

The level contains spawn points configured to spawn Proteus robots.
To spawn all spawn points call the following services in ROS2:

```shell
source /opt/ros/humble/setup.bash
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint1'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint2'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint3'}'& \
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{name: 'proteus', xml: 'spawnPoint4'}'
```

## Topics and frames

Every spawned robot has its namespace.
The first spawned robot publishes and subscribes to topics:

- `/proteus_1/cmd_vel` - the requested velocity
- `/proteus_1/pc` - the LiDAR pointcloud

The first spawned robot also provides the following transformations:

- `/proteus_1/odom`
- `/proteus_1/base_link`
- `/proteus_1/lidar`

### Example

To drive the second spawned robot use the following tool with topic name `/proteus_2/cmd_vel`:

```shell
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/proteus_2/cmd_vel
```
