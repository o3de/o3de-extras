# Docker scripts for running the O3DE Demo Project

The following Dockerfiles defined in this path will prepare the appropiate
ROS2 package (Ubuntu 20.04/Focal + Galactic or Ubuntu 22.04/Jammy + Humble)
based environment and build the components necessary to run the O3DE demo
project simulator through the O3DE engine.

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* At least 120 GB of free disk space
* Docker installed and configured
* SUDO Access

## Building the Docker Image

There are two Dockerfile images defined

* Dockerfile.ubuntu-20.04 (Focal / Galactic)
* Dockerfile.ubuntu-22.04 (Jammy / Humble)

Select which docker image you would like to build and run the following command

```
sudo docker build --build-arg minimal=1  -t o3de_loft_demo:latest -f Dockerfile.ubuntu-20.04 .
```

```
sudo docker build --build-arg minimal=1  -t o3de_loft_demo:latest -f Dockerfile.ubuntu-22.04 .
```

## Running the Docker Image




