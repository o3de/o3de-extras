/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SdfAssetBuilder/SdfAssetBuilder.h>

#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/IOUtils.h>
#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzFramework/StringFunc/StringFunc.h>
#include <AzToolsFramework/Entity/EntityUtilityComponent.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>

#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

namespace ROS2
{
        namespace
        {
            [[maybe_unused]] constexpr const char* SdfAssetBuilderName = "SdfAssetBuilder";
            constexpr const char* SdfAssetBuilderJobKey = "Sdf Asset Builder";
            constexpr AZStd::array<const char*,4> SdfAssetSourceExtensions =
            {
                "*.sdf",
                "*.urdf",
                "*.world",
                "*.xacro"
            };
        }

    void SdfAssetBuilder::RegisterBuilder()
    {
        AssetBuilderSDK::AssetBuilderDesc sdfAssetBuilderDescriptor;

        sdfAssetBuilderDescriptor.m_name = SdfAssetBuilderJobKey;
        sdfAssetBuilderDescriptor.m_version = 1; // bump this to rebuild all sdf files
        sdfAssetBuilderDescriptor.m_busId = azrtti_typeid<SdfAssetBuilder>();

        for(const auto& extension : SdfAssetSourceExtensions)
        {
            sdfAssetBuilderDescriptor.m_patterns.push_back(AssetBuilderSDK::AssetBuilderPattern(extension, AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));
        }

        sdfAssetBuilderDescriptor.m_createJobFunction = AZStd::bind(&SdfAssetBuilder::CreateJobs, this,
            AZStd::placeholders::_1, AZStd::placeholders::_2);
        sdfAssetBuilderDescriptor.m_processJobFunction = AZStd::bind(&SdfAssetBuilder::ProcessJob, this,
            AZStd::placeholders::_1, AZStd::placeholders::_2);

        // Listen for SdfAssetBuilder asset builder notifications.
        BusConnect(sdfAssetBuilderDescriptor.m_busId);

        // Register this builder with the AssetBuilderSDK.
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, sdfAssetBuilderDescriptor);
    }

    void SdfAssetBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request,
        AssetBuilderSDK::CreateJobsResponse& response) const
    {
        // For each input file, create an output job.
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "SDF (Simulation Description Format) Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void SdfAssetBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request,
        AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // The following code is just a stub while building the SdfAssetBuilder. 
        // Right now, it just generates an empty default procedural prefab. It still needs to be modified
        // to create a procedural prefab that contains the correct information in it.

        AZStd::string outputFilename = request.m_sourceFile;
        AzFramework::StringFunc::Path::ReplaceExtension(outputFilename, "procprefab");

        AZStd::string tempAssetOutputPath;
        AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), outputFilename.c_str(), tempAssetOutputPath, true);

        AZStd::string prefabJson = CreateDefaultProcPrefab(request, response);

        // If no prefab string was generated, then return a failure.
        if (prefabJson.empty())
        {
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // Save the prefab to our output directory.
        rapidjson::Document doc;
        doc.Parse(prefabJson.c_str());
        AZ::JsonSerializationUtils::WriteJsonFile(doc, tempAssetOutputPath.c_str());

        // Mark the resulting prefab as a product asset with the "procedural prefab" asset type.
        AssetBuilderSDK::JobProduct sdfJobProduct;
        sdfJobProduct.m_productFileName = tempAssetOutputPath;
        sdfJobProduct.m_productSubID = 0;
        sdfJobProduct.m_productAssetType = azrtti_typeid<AZ::Prefab::ProceduralPrefabAsset>();

        // Right now, just mark that dependencies are handled because there aren't any to handle.
        // Once the prefab becomes filled with information, we'll need to do more work here to ensure
        // that the prefab dependencies are parsed out and hooked up correctly.
        sdfJobProduct.m_dependenciesHandled = true;

        response.m_outputProducts.push_back(AZStd::move(sdfJobProduct));
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }

    AZStd::string SdfAssetBuilder::CreateDefaultProcPrefab(
        const AssetBuilderSDK::ProcessJobRequest& request,
        [[maybe_unused]] AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // The following code is similar to DefaultProceduralPrefabGroup::CreatePrefabGroupManifestUpdates().
        // It might be possible to refactor the two to have shared logic.
        // It's not worth investigating until the SdfAssetBuilder is fully functional though, because the code
        // is likely to change pretty drastically still.

        // Create a prefab name based on the source file name.
        AZStd::string prefabTemplateName{ request.m_sourceFile };
        AzFramework::StringFunc::Path::ReplaceExtension(prefabTemplateName, "procprefab");
        AZ::StringFunc::Replace(prefabTemplateName, "\\", "/"); // the source folder uses forward slash

        // clear out any previously created prefab template for this path
        auto* prefabSystemComponentInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabSystemComponentInterface>::Get();
        AzToolsFramework::Prefab::TemplateId prefabTemplateId =
            prefabSystemComponentInterface->GetTemplateIdFromFilePath({ prefabTemplateName.c_str() });
        if (prefabTemplateId != AzToolsFramework::Prefab::InvalidTemplateId)
        {
            prefabSystemComponentInterface->RemoveTemplate(prefabTemplateId);
            prefabTemplateId = AzToolsFramework::Prefab::InvalidTemplateId;
        }

        // create a default entity in the prefab to verify functionality.
        AZ::EntityId entityId;
        AzToolsFramework::EntityUtilityBus::BroadcastResult(
            entityId, &AzToolsFramework::EntityUtilityBus::Events::CreateEditorReadyEntity, "Test");

        AZStd::vector<AZ::EntityId> entities = { entityId };

        // create prefab from the "set" of entities (currently just the single default entity)
        AzToolsFramework::Prefab::PrefabSystemScriptingBus::BroadcastResult(
            prefabTemplateId,
            &AzToolsFramework::Prefab::PrefabSystemScriptingBus::Events::CreatePrefabTemplate,
            entities,
            prefabTemplateName);

        if (prefabTemplateId == AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_Error("prefab", false, "Could not create a prefab template for entities.");
            return {};
        }

        // Convert the prefab to a JSON string
        AZ::Outcome<AZStd::string, void> outcome;
        AzToolsFramework::Prefab::PrefabLoaderScriptingBus::BroadcastResult(
            outcome,
            &AzToolsFramework::Prefab::PrefabLoaderScriptingBus::Events::SaveTemplateToString,
            prefabTemplateId);

        if (outcome.IsSuccess() == false)
        {
            AZ_Error("prefab", false, "Could not create JSON string for template; maybe NaN values in the template?");
            return {};
        }

        return outcome.GetValue();
    }

} // ROS2