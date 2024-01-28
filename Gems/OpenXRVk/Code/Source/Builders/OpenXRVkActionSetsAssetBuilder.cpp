/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/StringFunc/StringFunc.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include <OpenXRVk/OpenXRVkInteractionProfilesAsset.h>
#include <OpenXRVk/OpenXRVkActionSetsAsset.h>

#include "OpenXRVkActionSetsAssetBuilder.h"

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
    
    void OpenXRActionSetsAssetBuilder::CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        //! First get the extension 
        constexpr bool includeDot = false;
        AZStd::string fileExtension;
        bool result = AZ::StringFunc::Path::GetExtension(request.m_sourceFile.c_str(), fileExtension, includeDot);
        if (result && (fileExtension == OpenXRVk::OpenXRInteractionProfilesAsset::s_assetExtension))
        {
            CreateInteractionProfilesAssetJobs(request, response);
            return;
        }

        if (result && (fileExtension == OpenXRVk::OpenXRActionSetsAsset::s_assetExtension))
        {
            CreateActionSetsAssetJobs(request, response);
            return;
        }

        //! Unknown extension.
        AZ_Error(LogName, false, "Unknown file extension [%s] for this builder. Source file [%s]", fileExtension.c_str(), request.m_sourceFile.c_str());
        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Failed;
    }
    
    
    void OpenXRActionSetsAssetBuilder::ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        //! First get the extension 
        constexpr bool includeDot = false;
        AZStd::string fileExtension;
        bool result = AZ::StringFunc::Path::GetExtension(request.m_sourceFile.c_str(), fileExtension, includeDot);
        if (result && (fileExtension == OpenXRVk::OpenXRInteractionProfilesAsset::s_assetExtension))
        {
            ProcessInteractionProfilesAssetJob(request, response);
            return;
        }
        if (result && (fileExtension == OpenXRVk::OpenXRActionSetsAsset::s_assetExtension))
        {
            ProcessActionSetsAssetJob(request, response);
        }
    }


    /////////////////////////////////////////////////////////////////////////////////
    // OpenXRInteractionProfilesAsset Support Begin
    void OpenXRActionSetsAssetBuilder::CreateInteractionProfilesAssetJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            // Very high priority because this asset is required to initialize the OpenXR runtime
            // and initialize the I/O actions system.
            jobDescriptor.m_priority = 1000;
            jobDescriptor.m_critical = true;
            jobDescriptor.m_jobKey = InteractionProfilesAssetJobKey;
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());
            response.m_createJobOutputs.emplace_back(AZStd::move(jobDescriptor));
        } // for all request.m_enabledPlatforms

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void OpenXRActionSetsAssetBuilder::ProcessInteractionProfilesAssetJob([[maybe_unused]] const AssetBuilderSDK::ProcessJobRequest& request, [[maybe_unused]] AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // Open the file, and make sure there's no redundant data, the OpenXR Paths are well formatted, etc.
       auto interactionProfilesAssetPtr = AZ::Utils::LoadObjectFromFile<OpenXRVk::OpenXRInteractionProfilesAsset>(request.m_fullPath);
       //AZ_Error(LogName, false, "The interaction profiles contain %zu profiles", interactionProfilesAssetPtr->m_interactionProfileDescriptors.size());

       // FIXME: TODO: All this builder is supposed to do is validate the data.
       // If the validation passes, then we simply generate a product which is just a copy of the original.

       // We keep exact same asset name and extension.
       AZStd::string assetFileName;
       AZ::StringFunc::Path::GetFullFileName(request.m_fullPath.c_str(), assetFileName);

       // Construct product full path
       AZStd::string assetOutputPath;
       AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), assetFileName.c_str(), assetOutputPath, true);

       bool result = AZ::Utils::SaveObjectToFile(assetOutputPath, AZ::DataStream::ST_XML, interactionProfilesAssetPtr);
       if (result == false)
       {
           AZ_Error(LogName, false, "Failed to save asset to %s", assetOutputPath.c_str());
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }

       // This step is very important, because it declares product dependency between ShaderAsset and the root ShaderVariantAssets (one for each supervariant).
       // This will guarantee that when the ShaderAsset is loaded at runtime, the ShaderAsset will report OnAssetReady only after the root ShaderVariantAssets
       // are already fully loaded and ready.
       AssetBuilderSDK::JobProduct jobProduct;
       if (!AssetBuilderSDK::OutputObject(interactionProfilesAssetPtr, assetOutputPath, azrtti_typeid<OpenXRVk::OpenXRInteractionProfilesAsset>(),
           aznumeric_cast<uint32_t>(0), jobProduct))
       {
           AZ_Error(LogName, false, "FIXME this message.");
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }
       response.m_outputProducts.emplace_back(AZStd::move(jobProduct));
       response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }
    // OpenXRInteractionProfilesAsset Support End
    /////////////////////////////////////////////////////////////////////////////////////

    void OpenXRActionSetsAssetBuilder::CreateActionSetsAssetJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            if (platformInfo.m_identifier != "pc")
            {
                continue;
            }
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            // Very high priority because this asset is required to initialize the OpenXR runtime
            // and initialize the I/O actions system.
            jobDescriptor.m_priority = 999;
            jobDescriptor.m_critical = true;
            jobDescriptor.m_jobKey = ActionSetsAssetJobKey;
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());
            response.m_createJobOutputs.emplace_back(AZStd::move(jobDescriptor));
        } // for all request.m_enabledPlatforms

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void OpenXRActionSetsAssetBuilder::ProcessActionSetsAssetJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        AZ::ObjectStream::FilterDescriptor loadFilter = AZ::ObjectStream::FilterDescriptor(&AZ::Data::AssetFilterNoAssetLoading, AZ::ObjectStream::FILTERFLAG_IGNORE_UNKNOWN_CLASSES);
        auto actionSetsAssetPtr = AZ::Utils::LoadObjectFromFile<OpenXRVk::OpenXRActionSetsAsset>(request.m_fullPath, nullptr, loadFilter);
        if (!actionSetsAssetPtr)
        {
            AZ_Error(LogName, false, "Failed to LoadObjectFromFile %s", request.m_fullPath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // The Action Sets Asset contains an asset reference to the OpenXRInteractionProfilesAsset that was used
        // to construct the data in it. Because we are running in a builder context, the OpenXRInteractionProfilesAsset
        // is loaded with a null handle, BUT the AssetHint is valid and we'll use the AssetHint to discover
        // the OpenXRInteractionProfilesAsset and load it manually.


        // We keep exact same asset name and extension.
        AZStd::string assetFileName;
        AZ::StringFunc::Path::GetFullFileName(request.m_fullPath.c_str(), assetFileName);

        // Construct product full path
        AZStd::string assetOutputPath;
        AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), assetFileName.c_str(), assetOutputPath, true);

        bool result = AZ::Utils::SaveObjectToFile(assetOutputPath, AZ::DataStream::ST_XML, actionSetsAssetPtr);
        if (result == false)
        {
            AZ_Error(LogName, false, "Failed to save asset to %s", assetOutputPath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // This step is very important, because it declares product dependency between ShaderAsset and the root ShaderVariantAssets (one for each supervariant).
        // This will guarantee that when the ShaderAsset is loaded at runtime, the ShaderAsset will report OnAssetReady only after the root ShaderVariantAssets
        // are already fully loaded and ready.
        AssetBuilderSDK::JobProduct jobProduct;
        if (!AssetBuilderSDK::OutputObject(actionSetsAssetPtr, assetOutputPath, azrtti_typeid<OpenXRVk::OpenXRActionSetsAsset>(),
            aznumeric_cast<uint32_t>(0), jobProduct))
        {
            AZ_Error(LogName, false, "FIXME this message.");
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }
        response.m_outputProducts.emplace_back(AZStd::move(jobProduct));
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }

    
} // namespace OpenXRVkBuilders

#pragma optimize( "", on ) // GALIB
