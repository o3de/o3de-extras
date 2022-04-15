# o3de-demo-project

Requires o3de-ros2-gem to run. Follow these steps to register the game with your o3de engine:

follow instructions to setup o3de engine:
- clone o3de-ros2-gem
- clone this project

In o3de folder:
```
scripts/o3de.sh register --gem-path <PATH_TO_GEM>
scripts/o3de.sh enable-gem -gn ROS2 -pp <PATH_TO_PROJECT>
```

In project folder:
```
cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=$HOME/o3de-packages
cmake --build build/linux --target <ProjectName>.GameLauncher Editor --config profile -j 4
```

Run the Editor:

```
build/linux/bin/profile/Editor
```

