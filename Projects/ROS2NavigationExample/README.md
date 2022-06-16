# O3DE ROS 2 Gem navigation demo project

This project will demonstrate the ROS2 Gems and O3DE using assets from the Loft demo scene and ROS 2 navigation stack.

## Requirements

This project will only run on Ubuntu 20.04 since the ROS 2 Gem is not yet developed for Windows. 

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met

This project has the following dependencies:

- [O3DE](https://github.com/o3de/o3de)
  -  Should work with `develop` branch and newest releases, but the project was tested with commit #89ab3eae.
- [ROS2 Gem](https://github.com/RobotecAI/o3de-ros2-gem)
  - `develop` branch (the default) should work. The project was tested with version tag `0.3`.
  - ROS 2 (Galactic) itself is also required, see [Gem Requirements](https://github.com/RobotecAI/o3de-ros2-gem#requirements)  
- [Loft Scene Sample](https://github.com/aws-lumberyard/loft-arch-vis-sample)
  - Use `scene_fixes` branch 


## Setup Instructions

The following steps will assume the following

- The instructions will be based off of a common base folder: $DEMO_BASE. For the steps below, we will use DEMO_BASE of ~/ for simplicty. 
- This current project has been fetched to $DEMO_BASE
- You have ROS2 installed and sourced 
  - for debian package Galactic installation, in your bash console, run `source /opt/ros/galactic/setup.bash`
  - you could also add this line to your `.profile`
  - check if ROS 2 is sourced in your current console with `echo $ROS_DISTRO`. You should see `galactic`.


### 1. Clone O3DE (or install) and register the engine

```
$ cd $DEMO_BASE
~$ git clone https://github.com/o3de/o3de.git
~$ cd o3de
~/o3de$ git lfs install
~/o3de$ git lfs pull
~/o3de$ scripts/o3de.sh register --this-engine
```

### 2. Clone and register the ROS2 Gem locally

```
$ cd $DEMO_BASE
~$ git clone https://github.com/RobotecAI/o3de-ros2-gem.git
~$ $DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-ros2-gem
```

### 3. Clone and register the Loft Scene project locally

```
$ cd $DEMO_BASE
~$ git clone https://github.com/aws-lumberyard/loft-arch-vis-sample.git
~$ cd loft-arch-vis-sample
~/loft-arch-vis-sample$ git checkout scene_fixes
~/loft-arch-vis-sample$ git lfs install
~/loft-arch-vis-sample$ git lfs pull
~$ $DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/loft-arch-vis-sample/Gems/ArchVis
```

### 4. Register this project and build it

```
$ cd $DEMO_BASE
~/$ o3de/scripts/o3de.sh register -pp $DEMO_BASE/o3de-demo-project
~/$ cd o3de-demo-project
~/o3de-demo-project$ cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF
~/o3de-demo-project$ cmake --build build/linux --config profile --target ROS2-Gem-Demo Editor AssetProcessor
```

### 5. Launch Editor

```
$ cd $DEMO_BASE
~/$ cd o3de-demo-project/build/linux/bin/profile
~/o3de-demo-project/build/linux/bin/profile$ ./Editor
```

## Troubleshooting

#### AssetProcessor resource problems

Sometimes when there were problems while the AssetProcessor was working (for example, disk space ran out),
subsequent executions of the Editor fail to re-start the process for such Assets. This might be due to a
limitation of the number of files that can be watched by a single user. You can fix this by increasing the
value, for example:

```
sudo sysctl -w fs.inotify.max_user_watches=524288
```

To make this setting permanent, add it to `/etc/systctl.conf` file.

#### No ROS 2 traffic on topics

This could be caused by a firewall, disabled multicast or issues with docker.

Please refer to [ROS 2 troubleshooting guide](https://docs.ros.org/en/galactic/How-To-Guides/Installation-Troubleshooting.html).

