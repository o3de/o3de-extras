# Fleet Robot Template

A project template that allows building multi-robot ROS2-enabled simulation. The template consists level with a large warehouse scene called `Warehouse.prefab`. 

## Spawning robots

The level contains spawn points configured to spawn Proteus robots.
To spawn all spawn points call the following services in ROS2:

```bash
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
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/proteus_2/cmd_vel
```