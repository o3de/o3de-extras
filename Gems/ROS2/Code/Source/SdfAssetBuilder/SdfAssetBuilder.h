/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AssetBuilderSDK/AssetBuilderBusses.h>

namespace ROS2
{
    //! Builder to convert the following file types into procedural prefab assets:
    //! * sdf (Simulation Description Format: http://sdformat.org/ )
    //! * urdf (Unified Robotics Description Format: http://wiki.ros.org/urdf )
    //! * world (Gazebo sdf files typically containing a full simulation world description: https://classic.gazebosim.org/tutorials?tut=components )
    //! * Xacro (XML macro used for sdf/urdf file generation: http://wiki.ros.org/xacro )
    //! source folders into procprefab assets in the cache folder.
    class SdfAssetBuilder
        : public AssetBuilderSDK::AssetBuilderCommandBus::Handler
    {
    public:
        AZ_RTTI(ROS2::SdfAssetBuilder, "{F5A45C1B-1D9F-4898-8E31-499C3787DA76}");

        SdfAssetBuilder() = default;
        ~SdfAssetBuilder() = default;

        void RegisterBuilder();

        // AssetBuilderSDK::AssetBuilderCommandBus overrides...
        void CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const;
        void ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const;
        void ShutDown() override { }
    private:
        AZStd::string CreateDefaultProcPrefab(
            const AssetBuilderSDK::ProcessJobRequest& request,
            AssetBuilderSDK::ProcessJobResponse& response) const;
    };
} // ROS2
