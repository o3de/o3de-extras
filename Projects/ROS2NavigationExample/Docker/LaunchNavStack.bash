#!/bin/bash

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

unset LD_LIBRARY_PATH

source /opt/ros/$ROS_DISTRO/setup.bash

cd /data/workspace/o3de-demo-project/launch

ros2 launch navigation.launch.py

