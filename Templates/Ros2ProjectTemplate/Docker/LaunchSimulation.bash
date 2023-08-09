#!/bin/bash

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

unset LD_LIBRARY_PATH

source /opt/ros/$ROS_DISTRO/setup.bash

export LD_LIBRARY_PATH=/data/workspace/Ros2Project/build/linux/bin/profile:$LD_LIBRARY_PATH

if [ -d /data/workspace/Ros2Project/build/linux/bin/profile ]
then
    cd /data/workspace/Ros2Project/build/linux/bin/profile
    ./Ros2Project.GameLauncher -r_fullscreen=$LAUNCH_FULLSCREEN_OPT -bg_ConnectToAssetProcessor=0 > /data/workspace/simulation_launch.log 2>&1
else
    echo "Simulation not installed on this image"
fi

