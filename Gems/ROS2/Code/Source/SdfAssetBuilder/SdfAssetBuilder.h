/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AssetBuilderSDK/AssetBuilderBusses.h>
#include <AssetBuilderSDK/AssetBuilderSDK.h>

namespace ROS2
{
    //! Builder to convert Sdf, Urdf, World, and XAcro assets in the
    //! source folders into procprefab assets in the cache folder.
    class SdfAssetBuilder
        : public AssetBuilderSDK::AssetBuilderCommandBus::Handler
    {
    public:
        AZ_RTTI(ROS2::SdfAssetBuilder, "{F5A45C1B-1D9F-4898-8E31-499C3787DA76}");

        SdfAssetBuilder() = default;

        void RegisterBuilder();

        // Asset Builder Callback Functions...
        void CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const;
        void ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const;

        // AssetBuilderSDK::AssetBuilderCommandBus overrides...
        void ShutDown() override { }

    };
} // ROS2
