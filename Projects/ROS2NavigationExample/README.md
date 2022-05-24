# o3de-demo-project

This project will demonstrate the ROS2 Gems and O3DE using assets from the Loft demo scene

## Requirements

Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met

This project has the following dependencies


- [O3DE](https://github.com/o3de/o3de) 
- [ROS2 Gem](https://github.com/RobotecAI/o3de-ros2-gem)
- [Loft Scene Sample](https://github.com/aws-lumberyard/loft-arch-vis-sample)


## Setup Instructions

The following steps will assume the following

- All of the requirements are met
- You have permissions to all of the github repos descripted in the requirements
- The instructions will be based off of a common base folder: $DEMO_BASE. For the steps below, we will set DEMO_BASE to ~/github. 
- This current project has been fetched to $DEMO_BASE



### 1. Clone O3DE (or install) and register the engine


```

$ cd $DEMO_BASE


~/github$ git clone https://github.com/o3de/o3de.git

~/github$ cd o3de

~/github/o3de$ git lfs install

~/github/o3de$ git lfs pull

~/github/o3de$ scripts/o3de.sh register --this-engine

```


### 2. Clone and register the ROS2 Gem locally


```

$ cd $DEMO_BASE

~/github$ git clone https://github.com/RobotecAI/o3de-ros2-gem.git

~/github$ $DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/o3de-ros2-gem


```

### 3. Clone and register the Loft Scene project locally


```

$ cd $DEMO_BASE

~/github$ git clone https://github.com/aws-lumberyard/loft-arch-vis-sample.git

~/github$ cd loft-arch-vis-sample

~/github/loft-arch-vis-sample$ git lfs install

~/github/loft-arch-vis-sample$ git lfs pull

~/github$ $DEMO_BASE/o3de/scripts/o3de.sh register --gem-path $DEMO_BASE/loft-arch-vis-sample/Gems/ArchVis

```

### 4. Register this project and build it

```
$ cd $DEMO_BASE

~/github$ o3de/scripts/o3de.sh register -pp $DEMO_BASE/o3de-demo-project

~/github$ cd o3de-demo-project

~/github/o3de-demo-project$ cmake -B build/linux -G "Ninja Multi-Config" -DLY_UNITY_BUILD=OFF -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_PARALLEL_LINK_JOBS=16 -DLY_STRIP_DEBUG_SYMBOLS=OFF

~/github/o3de-demo-project$ cmake --build build/linux --config profile --target ROS2-Gem-Demo Editor AssetProcessor


```


### 5. Launch Editor


```
$ cd $DEMO_BASE

~/github$ cd o3de-demo-project

~/github/o3de-demo-project$ cd build/linux/bin/profile

~/github/o3de-demo-project/build/linux/bin/profile$ ./Editor



```

## Troubleshooting

Sometimes when there were problems while the AssetProcessor was working (for example, disk space ran out),
subsequent executions of the Editor fail to re-start the process for such Assets. This might be due to a
limitation of the number of files that can be watched by a single user. You can fix this by increasing the
value, for example:

```
sudo sysctl -w fs.inotify.max_user_watches=524288
```