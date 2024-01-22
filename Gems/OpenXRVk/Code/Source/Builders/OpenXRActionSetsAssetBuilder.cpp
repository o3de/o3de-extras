/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AssetBuilderSDK/SerializationDependencies.h>

#include "OpenXRActionSetsAssetBuilder.h"

#pragma optimize( "", off ) // GALIB

namespace OpenXRVkBuilders
{
    // [[maybe_unused]] const char* AnyAssetBuilderName = "AnyAssetBuilder";
    // const char* AnyAssetBuilderJobKey = "Any Asset Builder";
    // const char* AnyAssetBuilderDefaultExtension = "azasset";
    // const char* AnyAssetSourceExtensions[] =
    // {
    //     "azasset",
    //     "attimage",
    //     "azbuffer",
    // };
    // const uint32_t NumberOfSourceExtensions = AZ_ARRAY_SIZE(AnyAssetSourceExtensions);
    
    void OpenXRActionSetsAssetBuilder::CreateJobs([[maybe_unused]] const AssetBuilderSDK::CreateJobsRequest& request, [[maybe_unused]] AssetBuilderSDK::CreateJobsResponse& response) const
    {

    }
    
    
    void OpenXRActionSetsAssetBuilder::ProcessJob([[maybe_unused]] const AssetBuilderSDK::ProcessJobRequest& request, [[maybe_unused]] AssetBuilderSDK::ProcessJobResponse& response) const
    {

    }
    
} // namespace OpenXRVkBuilders

#pragma optimize( "", on ) // GALIB
