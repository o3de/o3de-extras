#!/bin/bash

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Delete obsolete files after the build of the code and assets are complete

DELETE_LIST=(~/.o3de/3rdParty/ \
             o3de/.git \
             o3de/AutomatedTesting \
             o3de/python/downloaded_packages \
             o3de/Code \
             o3de/Gems \
             Ros2Project/build/linux/Azcg/ \
             Ros2Project/build/linux/CMake \
             Ros2Project/build/linux/CMakeFiles/ \
             Ros2Project/build/linux/External/ \
             Ros2Project/build/linux/Testing/ \
             Ros2Project/build/linux/_deps/ \
             Ros2Project/build/linux/cmake \
             Ros2Project/build/linux/lib/ \
             Ros2Project/build/linux/o3de/ \
             Ros2Project/build/linux/packages/ \
             Ros2Project/build/linux/runtime_dependencies/ 
             Ros2Project/build/linux/bin/profile/EditorPlugins \
             Ros2Project/build/linux/bin/profile/Editor \
             Ros2Project/build/linux/bin/profile/AssetProcessor \
             Ros2Project/build/linux/bin/profile/AssetProcessorBatch \
             Ros2Project/build/linux/bin/profile/MaterialEditor \
             Ros2Project/build/linux/bin/profile/AssetBuilder \
             Ros2Project/build/linux/bin/profile/MaterialCanvas )

for i in ${DELETE_LIST[@]}
do
   echo "Deleting /data/workspace/$i"
   rm -rf $i
done

exit 0

