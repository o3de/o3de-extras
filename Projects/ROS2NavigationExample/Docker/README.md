# Docker scripts for running the O3DE Demo Project

The following Dockerfiles defined in this path will prepare the appropiate ROS2 package <br>
(Ubuntu 20.04/Focal + Galactic or Ubuntu 22.04/Jammy + Humble) based environment and build<br>
the components necessary to run the O3DE demo project simulator through the O3DE engine.

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Ubuntu 20.04 (Focal) or 22.04 (Jammy)
* At least 60 GB of free disk space
* Docker installed and configured
* [NVidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
* SUDO Access

## Building the Docker Image

By default, the docker script provided in this project will build a docker image to run the Robot Vacuum sample project
on Ubuntu 22.04 (jammy) with the ROS2 Humble distribution. For example, to build the docker image, run the following
command:

```
sudo docker build -t o3de_robot_vacuum_simulation:latest .
```

This will create a docker image named 'o3de_robot_vacuum_simulation' with the tag 'latest' that contains both the simulation launcher and the 
navigation stack. It will also contain helper scripts that will launch either the simulation (LaunchSimulation.bash) or 
the Rviz2 (LaunchNavStack.bash).

You can also create a separate docker image that only contains the navigation stack and Rviz2 by supplying the argument 
```IMAGE_TYPE``` and setting it to 'navstack':

```
sudo docker build --build-arg IMAGE_TYPE=navstack -t o3de_robot_vacuum_navstack:latest .
```

ROS2 allows for communication across multiple docker images running on the same host, provided that they specify the 'bridge' 
network type when launching the docker image.


## Running the Docker Image

Launching O3DE applications in a Docker container requires GPU acceleration support. (Make sure that the [nvidia-docker 2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) is installed.)

### Direct Access to the X Server
The simulation docker image should be launched first before bringing up the robot application. To run the robot application, 
first allow the container root user to access the running X server for display

```
xhost +local:root
```

Then launch the built simulation docker image with the following command

```
sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_robot_vacuum_simulation:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.

```
sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_robot_vacuum_navstack:latest /data/workspace/LaunchNavStack.bash

```

Make sure to revoke access to the X server when the simulation ends.

```
xhost -local:root
```

### Running using Rocker

Alternatively, you can use [Rocker](https://github.com/osrf/rocker) to run a GPU-accelerated docker image. 

Launch the built simulation docker image with the following rocker command

```
rocker --x11 --nvidia o3de_robot_vacuum_simulation:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.

```
rocker --x11 --nvidia o3de_robot_vacuum_navstack:latest /data/workspace/LaunchNavStack.bash
```

## Advanced Options

### Target ROS2 Distribution
The Docker script defaults to building an image based on Ubuntu 20.04 (bionic) and the ROS2 Humble distribution. This can be overridden 
with a combination if the ```ROS_VERSION``` and ```UBUNTU_VERSION``` arguments.

| ROS2 Distro   | Repository                                |
|---------------|-------------------------------------------|
| galactic      | ROS_VERSION=galactic UBUNTU_VERSION=focal |
| humble        | ROS_VERSION=humble UBUNTU_VERSION=humble  |


### Custom source repos and branches

The Dockerscripts use the following arguments to determine the repository to pull the source from. 

| Argument              | Repository                       | Default     |
|-----------------------|----------------------------------|-------------|
| O3DE_REPO             | O3DE                             | https://github.com/o3de/o3de.git                   |
| ROS2_GEM_REPO         | O3DE ROS2 Gem                    | https://github.com/RobotecAI/o3de-ros2-gem.git     |
| LOFT_GEM_REPO         | Loft ArchVis Sample Scene        | https://github.com/o3de/loft-arch-vis-sample.git   |
| ROBOT_VAC_SAMPLE_REPO | Loft Scene Simulation repository | https://github.com/RobotecAI/o3de-demo-project.git |

In addition the repositories, the following arguments target the branch, commit, or tag to pull from their corresponding repository

| Argument                | Repository                       | Default     |
|-------------------------|----------------------------------|-------------|
| O3DE_BRANCH             | O3DE                             | development |
| ROS2_GEM_BRANCH         | O3DE ROS2 Gem                    | development |
| LOFT_GEM_BRANCH         | Loft ArchVis Sample Scene        | main        |
| ROBOT_VAC_SAMPLE_BRANCH | Loft Scene Simulation repository | main        |

