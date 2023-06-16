# Docker scripts for running the O3DE Ros2Project based on O3DE Ros2ProjectTemplate

The following Dockerfiles defined in this path will prepare the appropriate ROS2 package (Ubuntu 20.04/Focal + Galactic or Ubuntu 22.04/Jammy + Iron) based environment and build the components necessary to run the O3DE demo project simulator through the O3DE engine using the Ros2Project template.

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Ubuntu 20.04 (Focal) or 22.04 (Jammy)
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and in a secure manner so that the docker commands in this guide do not require elevated priviledges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [NVidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Building the Docker Image

By default, the docker script provided in this template will build a docker image to run the Ros2Project sample using Ubuntu 22.04 (jammy) with the ROS2 Iron distribution. Run the following command to build the docker image with a default configuration:

```
docker build -t o3de_ros2project:latest -f Dockerfile .
```

**Note** 
The above command example tags specific commits for O3DE, and the ROS2 gem repos and based on known working commits. See the Advanced Options section below for more information.

This will create a docker image named 'o3de_ros2project' with the tag 'latest' that contains both the simulation launcher and the navigation stack. 

It will also contain helper scripts that will launch either the simulation (LaunchSimulation.bash) or 
the RViz2 (LaunchNavStack.bash).

You can also create a separate docker image that only contains the navigation stack and RViz2 by supplying the argument `IMAGE_TYPE` and setting it to 'navstack':

```
docker build --build-arg IMAGE_TYPE=navstack -t o3de_ros2project_nav:latest .
```

ROS2 allows for communication across multiple docker images running on the same host, provided that they specify the 'bridge' network type when launching.

## Running the Docker Image

Launching O3DE applications in a Docker container requires GPU acceleration support. (Make sure that the [nvidia-docker 2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) is installed.)

### Direct Access to the X Server
The simulation docker image should be launched before bringing up the robot application. To run the robot application, first allow the container root user to access the running X server for display

```
xhost +local:root
```

Then launch the built simulation docker image with the following command

```
docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_ros2project:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the navigation stack inside the simulation docker image, which will bring up RViz to control the robot.

```
docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_ros2project:latest /data/workspace/LaunchNavStack.bash

```

If you created a separate docker image 'o3de_ros2project_nav:latest', which only contains the navigation stack and RViz2, you can launch it using that image, provided that the simulation docker image 'o3de_ros2project' is running.

```
docker run --rm --network="bridge" --gpus all -e DISPLAY=:1 -v /tmp/.X11-unix:/tmp/.X11-unix -it o3de_ros2project_nav:latest /data/workspace/LaunchNavStack.bash
```
Make sure to revoke access to the X server when the simulation ends.

```
xhost -local:root
```

### Running using Rocker

Alternatively, you can use [Rocker](https://github.com/osrf/rocker) to run a GPU-accelerated docker image. 

Launch the built simulation docker image with the following rocker command

```
rocker --x11 --nvidia o3de_ros2project:latest /data/workspace/LaunchSimulation.bash
```

Once the simulation is up and running, launch the robot application docker image, which will bring up RViz to control the robot.

```
rocker --x11 --nvidia o3de_ros2project:latest /data/workspace/LaunchNavStack.bash
```

## Advanced Options

### Target ROS2 Distribution
The Docker script defaults to building an image based on Ubuntu 22.04 (jammy) and the ROS2 Iron distribution. This can be overridden with a combination if the `ROS_VERSION` and `UBUNTU_VERSION` arguments.

| ROS2 Distro   | Repository                                |
|---------------|-------------------------------------------|
| galactic      | ROS_VERSION=galactic UBUNTU_VERSION=focal |
| humble        | ROS_VERSION=humble   UBUNTU_VERSION=jammy |
| iron          | ROS_VERSION=iron     UBUNTU_VERSION=jammy |

### Custom source repos and branches

The Dockerscripts use the following arguments to determine the repository to pull the source from. 

| Argument              | Repository                 | Default                                      |
|-----------------------|----------------------------|----------------------------------------------|
| O3DE_REPO             | O3DE                       | https://github.com/o3de/o3de.git             |
| O3DE_EXTRAS_REPO      | O3DE Extras                | https://github.com/o3de/o3de-extras.git      |

In addition the repositories, the following arguments target the branch, commit, or tag to pull from their corresponding repository

| Argument                | Repository                       | Default     |
|-------------------------|----------------------------------|-------------|
| O3DE_BRANCH             | O3DE                             | 2305.0      |
| O3DE_EXTRAS_BRANCH      | O3DE Extras                      | 2305.0      |

### Optimizing the build process ###
The docker script provides a cmake-specific argument override to control the number of parallel jobs that can be used during the build of the docker image. `CMAKE_JOBS` sets the maximum number of concurrent jobs cmake will run during its build process and defaults to 8 jobs. This number can be adjusted to better suit the hardware which is running the docker image build.

