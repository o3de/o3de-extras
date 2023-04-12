# ROS2 Project Template

This template allows to create a ROS2 project with sample content. 

The example ROS2 navigation stack launchfile is bundled with the template.

## Requirements

Due to ROS2 dependency, this project was prepared and tested on Linux operating system. It is recomended to use Ubuntu [20.04](https://releases.ubuntu.com/focal) or [22.10](https://releases.ubuntu.com/kinetic), however any distribution meeting following requirements may be used.

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met. 
This project has the following dependencies:

- [O3DE](https://github.com/o3de/o3de)
- [ROS2](https://www.ros.org/) (galactic or humble)
- [ROS2 Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ROS2)
- [RosRobotSample Assets](https://github.com/o3de/o3de-extras/tree/development/Gems/RosRobotSample)
- [WarehouseSample Assets](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseSample)

Please make sure that `clang` was installed and configured. For details refer to [this section](https://www.o3de.org/docs/welcome-guide/requirements/#linux) of O3DE documentation.

To run the navigation example, two ROS2 packages are also required:
- [navigation2](https://github.com/ros-planning/navigation2)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

## Setup Instructions

The following steps will assume the following:

- You have ROS2 humble [installed](https://docs.ros.org/en/humble/Installation.html) and environment is [sourced](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files).
- The O3DE gems, projects and templates will be placed in the O3DE home folder: `/home/${USER}/O3DE`.
- You have the O3DE engine [downloaded and built](https://www.o3de.org/docs/welcome-guide/setup/setup-from-github) and it is located in `/home/${USER}/O3DE/Engines/Development`.

In this tutorial, CLI tools will be used. It is also possible to use O3DE gui to set up a project from template. See the [O3DE Project manager documentation](https://www.o3de.org/docs/user-guide/project-config/project-manager/) for more details. In such case it is required to source your ROS2 distro before launching O3DE manager:

```shell
source /opt/ros/humble/setup.bash
./build/linux/bin/profile/o3de
```

### 1. Install ROS2 packages

```shell
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-control-toolbox
```

### 2. Download the template and asset gems

For convenienience, we'll define a shell variables with o3de folders:

```shell
export O3DE_HOME=/home/${USER}/O3DE
export O3DE_ENGINE=${O3DE_HOME}/Engines/Development
```

Clone the `o3de-extras` repository containing the template and asset gems

```shell
mkdir -p ${O3DE_HOME}/Projects
cd ${O3DE_HOME}/Projects
git clone git@github.com:o3de/o3de-extras.git
```

Copy gems to the O3DE home.

```shell
mkdir -p ${O3DE_HOME}/Gems
cp -r o3de-extras/Gems/ROS2 ${O3DE_HOME}/Gems
cp -r o3de-extras/Gems/WarehouseSample ${O3DE_HOME}/Gems
cp -r o3de-extras/Gems/RosRobotSample ${O3DE_HOME}/Gems
```

Register these gems.

```shell
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/ROS2
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/WarehouseSample
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_HOME}/Gems/RosRobotSample
```

### 3. Create a ROS2 project from the template

Assign a name for the new project. In this example, it is assumed that it will be: `WarehouseTest`, and it will be located in `$DEMO_BASE/WarehouseTest` folder. 

```shell
export PROJECT_NAME=WarehouseTest
export PROJECT_PATH=${O3DE_HOME}/Projects/${PROJECT_NAME}
${O3DE_HOME}/scripts/o3de.sh create-project --project-path $PROJECT_PATH --template-path ${O3DE_HOME}/Projects/o3de-extras/Templates/Ros2ProjectTemplate/ -f 
```

Enable gems.

```shell
${O3DE_HOME}/scripts/o3de.sh enable-gem --gem-name ROS2 --project-path $PROJECT_PATH
${O3DE_HOME}/scripts/o3de.sh enable-gem --gem-name WarehouseSample --project-path $PROJECT_PATH
${O3DE_HOME}/scripts/o3de.sh enable-gem --gem-name RosRobotSample --project-path $PROJECT_PATH
```

### 4. Build the project

Next, let us the build project with the necessary elements of the O3DE engine and ROS2 Gem.

```shell
cd $PROJECT_PATH
source /opt/ros/humble/setup.bash
cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF
cmake --build build/linux --config profile --target $PROJECT_NAME.GameLauncher Editor
```

### 5. Launch Editor

Finally, previously built O3DE with preloaded ROS2 Gem can be run:

```shell
$PROJECT_PATH/build/linux/bin/profile/Editor
```

## Running ROS example

Refer to `$PROJECT_PATH/Examples/slam_navigation/README.md` for instructions.
