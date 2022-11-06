# Docker scripts for running the O3DE Demo Project

The following Dockerfiles defined in this path will prepare the appropiate
ROS2 package (Ubuntu 20.04/Focal + Galactic or Ubuntu 22.04/Jammy + Humble)
based environment and build the components necessary to run the O3DE demo
project simulator through the O3DE engine.

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* At least 60 GB of free disk space
* Docker installed and configured
* SUDO Access

## Building the Docker Image

There are two demo simulation Dockerfile scripts defined

* Dockerfile.simulation.ubuntu-galactic (Focal / Galactic)
* Dockerfile.simulation.ubuntu-humble (Jammy / Humble)

There are also two robot navigation stack Dockerfile scripts defined

* Dockerfile.robot.ubuntu-galactic (Focal / Galactic)
* Dockerfile.robot.ubuntu-humble (Jammy / Humble)


Select which docker image you would like to build and run the following commands to build the client and rviz docker images

```

sudo docker build -t o3de_loft_demo_simulation:latest -f Dockerfile.simulation.ubuntu-galactic .

sudo docker build -t o3de_loft_demo_robot:latest -f Dockerfile.robot.ubuntu-galactic .

```

```

sudo docker build -t o3de_loft_demo_simulation:latest -f Dockerfile.simulation.ubuntu-humble .

sudo docker build -t o3de_loft_demo_robot:latest -f Dockerfile.robot.ubuntu-humble .

```

## Running the Docker Image


The simulation docker image should be launched first before bringing up the robot application. To run the robot application, 
first allow the root user to access the running X server

```
xhost +local:root
```

Then launch the built simulation docker image with the following command

```

sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_loft_demo_simulation:latest /data/workspace/LaunchClient.bash

```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.


```

sudo docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_loft_demo_robot:latest /data/workspace/LaunchRViz.bash

```

