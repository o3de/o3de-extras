# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

ARG ROS_VERSION=jazzy

FROM ros:${ROS_VERSION}-ros-base

# Argument to determining the image type ('simulation' or 'navstack')
ARG IMAGE_TYPE=simulation  # Default to 'simulation'

# Arguments for the source repos needed for the Ros2Template sample docker
ARG O3DE_REPO=https://github.com/o3de/o3de.git
ARG O3DE_BRANCH=main

ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ARG O3DE_EXTRAS_BRANCH=main

# Additional argument to control build concurrency
ARG CMAKE_JOBS=8

ENV WORKSPACE=/data/workspace

ENV PROJECT_PATH=/data/workspace/Ros2Project

WORKDIR $WORKSPACE

RUN apt-get update && apt-get upgrade -y

# Install packages needed for cloning and building from the source repos
RUN apt-get install -y --no-install-recommends \
    git \
    git-lfs \
    clang \
    ninja-build \
    cmake \
    libglu1-mesa-dev \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxcb-xinput-dev \
    libxcb-xfixes0-dev \
    libxcb-xkb-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    libfontconfig1-dev \
    libcurl4-openssl-dev \
    libsdl2-dev \
    zlib1g-dev \
    mesa-common-dev \
    libssl-dev libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-randr0 \
    libnvidia-gl-470 \
    ufw \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    ros-${ROS_DISTRO}-gazebo-msgs \
    ros-${ROS_DISTRO}-ackermann-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-desktop


# Copy script used to remove build artifacts
COPY cleanup.bash /data/workspace/cleanup.bash

# Clone and register O3DE repos 
RUN if [ "${IMAGE_TYPE}" = "simulation" ]; then \
        cd $WORKSPACE && \
        git clone --recursive $O3DE_REPO && \
        git -C $WORKSPACE/o3de checkout $O3DE_BRANCH &&\
        git -C $WORKSPACE/o3de lfs install && \
        git -C $WORKSPACE/o3de lfs pull && \
        $WORKSPACE/o3de/python/get_python.sh && \
        $WORKSPACE/o3de/scripts/o3de.sh register -ep $WORKSPACE/o3de && \
        git clone $O3DE_EXTRAS_REPO && \
        git -C $WORKSPACE/o3de-extras checkout $O3DE_EXTRAS_BRANCH && \
        $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/ROS2 && \
        $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/RosRobotSample && \
        $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/WarehouseAssets && \
        $WORKSPACE/o3de/scripts/o3de.sh register -gp $WORKSPACE/o3de-extras/Gems/WarehouseSample; \
    elif [  "${IMAGE_TYPE}" = "navstack" ]; then \
        cd $WORKSPACE && \
        git clone $O3DE_EXTRAS_REPO && \
        git -C $WORKSPACE/o3de-extras checkout $O3DE_EXTRAS_BRANCH; \
    else \
        echo "Unsupported IMAGE_TYPE: ${IMAGE_TYPE}" && exit 1; \
    fi

# Build and cleanup in the same layer to reduce the size
RUN if [ "${IMAGE_TYPE}" = "simulation" ]; then \
        . /opt/ros/${ROS_DISTRO}/setup.sh && \
        $WORKSPACE/o3de/scripts/o3de.sh create-project --project-path $PROJECT_PATH --template-path $WORKSPACE/o3de-extras/Templates/Ros2ProjectTemplate && \
        cmake -B $PROJECT_PATH/build/linux -S $PROJECT_PATH -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON && \
        cmake --build $PROJECT_PATH/build/linux --config profile --target Ros2Project Editor Ros2Project.Assets Ros2Project.GameLauncher -j $CMAKE_JOBS && \
        $WORKSPACE/cleanup.bash ; \
    fi

COPY LaunchSimulation.bash /data/workspace/LaunchSimulation.bash
COPY LaunchNavStack.bash /data/workspace/LaunchNavStack.bash

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
ENV LAUNCH_FULLSCREEN_OPT=0
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

ENTRYPOINT ["/bin/bash"]
