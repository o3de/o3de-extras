#!/bin/bash

# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Delete files that were only needed for the Client and AssetProcessing after a build of the code and assets are complete

DELETE_LIST=(o3de-ros2-gem/ \
             loft-arch-vis-sample/ \
             o3de/.git \
             o3de/AutomatedTesting \
             o3de/python/downloaded_packages \
             o3de/Code \
             o3de/Gems \
             o3de-demo-project/.git \
             o3de-demo-project/Gem \
             o3de-demo-project/Source \
             o3de-demo-project/Levels \
             o3de-demo-project/ReflectionProbes \
             o3de-demo-project/build/linux/Azcg/ \
             o3de-demo-project/build/linux/CMake \
             o3de-demo-project/build/linux/CMakeFiles/ \
             o3de-demo-project/build/linux/External/ \
             o3de-demo-project/build/linux/Testing/ \
             o3de-demo-project/build/linux/_deps/ \
             o3de-demo-project/build/linux/cmake \
             o3de-demo-project/build/linux/lib/ \
             o3de-demo-project/build/linux/o3de/ \
             o3de-demo-project/build/linux/packages/ \
             o3de-demo-project/build/linux/runtime_dependencies/ \
             ~/.o3de/3rdParty/ \
             ~/.o3de/3rdParty/packages/ \
             o3de-demo-project/build/linux/bin/profile/*.Editor.so \
             o3de-demo-project/build/linux/bin/profile/EditorPlugins \
             o3de-demo-project/build/linux/bin/profile/Editor \
             o3de-demo-project/build/linux/bin/profile/AssetProcessor \
             o3de-demo-project/build/linux/bin/profile/AssetProcessorBatch \
             o3de-demo-project/build/linux/bin/profile/MaterialEditor \
             o3de-demo-project/build/linux/bin/profile/AssetBuilder \
             o3de-demo-project/build/linux/bin/profile/MaterialCanvas )

for i in ${DELETE_LIST[@]}
do
   echo "Deleting /data/workspace/$i"
   rm -rf $i
done

exit 0

