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

There are two demo simulation Dockerfile scripts defined

* Dockerfile.simulation.ubuntu-galactic (Focal / Galactic)
* Dockerfile.simulation.ubuntu-humble (Jammy / Humble)

There are also two robot navigation stack Dockerfile scripts defined

* Dockerfile.navstack.ubuntu-galactic (Focal / Galactic)
* Dockerfile.navstack.ubuntu-humble (Jammy / Humble)


Select which docker image you would like to build and run the following commands to build the simulation and navigation stack docker images

```

sudo docker build -t o3de_loft_demo_simulation:latest -f Dockerfile.simulation.ubuntu-galactic .

sudo docker build -t o3de_loft_demo_navstack:latest -f Dockerfile.navstack.ubuntu-galactic .

```

```

sudo docker build -t o3de_loft_demo_simulation:latest -f Dockerfile.simulation.ubuntu-humble .

sudo docker build -t o3de_loft_demo_navstack:latest -f Dockerfile.navstack.ubuntu-humble .

```

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
sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_loft_demo_simulation:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.

```
sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_loft_demo_navstack:latest /data/workspace/LaunchNavStack.bash

```

Make sure to revoke access to the X server when the simulation ends.

```
xhost -local:root
```

### Running using Rocker

Alternatively, you can use [Rocker](https://github.com/osrf/rocker) to run a GPU-accelerated docker image. 

Launch the built simulation docker image with the following rocker command

```
rocker --x11 --nvidia o3de_loft_demo_simulation:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.

```
rocker --x11 --nvidia o3de_loft_demo_simulation:latest /data/workspace/LaunchNavStack.bash
```

### Advanced Options

The Dockerscripts as written are designed to pull from the latest branches of the following github repos:

[O3DE main repository](https://github.com/o3de/o3de.git)  (development)
[O3DE ROS2 Gem repository](https://github.com/RobotecAI/o3de-ros2-gem.git) (development)
[O3DE Loft ArchVis Sample Scene repository](https://github.com/o3de/loft-arch-vis-sample.git) (main)
[Loft Scene Simulation repository](https://github.com/RobotecAI/o3de-demo-project.git) (main)

The following build arguments are supported to pull from alternative branches, tags, or commits:

| Argument         | Repository                       | Default     |
|------------------|----------------------------------|-------------|
| o3de_branch      | O3DE                             | development |
| ros2_gem_branch  | O3DE ROS2 Gem                    | development |
| loft_gem_branch  | Loft ArchVis Sample Scene        | main        |
| o3de_demo_branch | Loft Scene Simulation repository | main        |
