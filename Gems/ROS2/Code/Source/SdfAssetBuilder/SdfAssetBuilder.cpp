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
#include <AzCore/Settings/SettingsRegistryVisitorUtils.h>
#include <AzToolsFramework/Entity/EntityUtilityComponent.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>

#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include <RobotImporter/URDF/URDFPrefabMaker.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <Utils/RobotImporterUtils.h>

namespace ROS2
{
        namespace
        {
            [[maybe_unused]] constexpr const char* SdfAssetBuilderName = "SdfAssetBuilder";
            constexpr const char* SdfAssetBuilderJobKey = "Sdf Asset Builder";
            constexpr const char* SdfAssetBuilderSupportedFileExtensionsRegistryKey = "/O3DE/ROS2/SdfAssetBuilder/SupportedFileTypeExtensions";
        }

    SdfAssetBuilder::SdfAssetBuilder()
    {
        AssetBuilderSDK::AssetBuilderDesc sdfAssetBuilderDescriptor;

        sdfAssetBuilderDescriptor.m_name = SdfAssetBuilderJobKey;
        sdfAssetBuilderDescriptor.m_version = 1; // bump this to rebuild all sdf files
        sdfAssetBuilderDescriptor.m_busId = azrtti_typeid<SdfAssetBuilder>();
        sdfAssetBuilderDescriptor.m_patterns = GetSupportedBuilderPatterns();

        sdfAssetBuilderDescriptor.m_createJobFunction = [this](auto && request, auto && response) 
            { 
                return CreateJobs(request, response); 
            };

        sdfAssetBuilderDescriptor.m_processJobFunction = [this](auto && request, auto && response) 
            { 
                return ProcessJob(request, response); 
            };

        // Listen for asset builder notifications requesting jobs for any of the sdf source file types.
        BusConnect(sdfAssetBuilderDescriptor.m_busId);

        // Register this builder with the AssetBuilderSDK.
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, sdfAssetBuilderDescriptor);
    }

    SdfAssetBuilder::~SdfAssetBuilder()
    {
        // Stop listening for asset builder notifications.
        BusDisconnect();

        // The AssetBuilderSDK doesn't support deregistration, so there's nothing more to do here.
    }

    AZStd::vector<AssetBuilderSDK::AssetBuilderPattern> SdfAssetBuilder::GetSupportedBuilderPatterns()
    {
        AZStd::vector<AssetBuilderSDK::AssetBuilderPattern> patterns;

        auto settingsRegistry = AZ::SettingsRegistry::Get();
        if (settingsRegistry == nullptr)
        {
            AZ_Error(SdfAssetBuilderName, false, "Settings Registry not found, no sdf file type extensions enabled.");
            return {};
        }

        // Visit each supported file type extension and create an asset builder wildcard pattern for it.
        auto VisitFileTypeExtensions = [&settingsRegistry, &patterns]
            (const AZ::SettingsRegistryInterface::VisitArgs& visitArgs)
            {
                if (AZ::SettingsRegistryInterface::FixedValueString value;
                    settingsRegistry->Get(value, visitArgs.m_jsonKeyPath))
                {
                    // Ignore any entries that are either completely empty or *only* contain a '.'.
                    // These will produce excessive (and presumably incorrect) wildcard matches.
                    if (value.empty() ||
                        ((value.size() == 1) && value.starts_with('.')))
                    {
                        return AZ::SettingsRegistryInterface::VisitResponse::Continue;
                    }

                    // Support both 'sdf' and '.sdf' style entries in the setreg file for robustness.
                    // Either one will get turned into a '*.sdf' pattern.
                    AZStd::string wildcardPattern = value.starts_with('.')
                        ? AZStd::string::format("*%s", value.c_str())
                        : AZStd::string::format("*.%s", value.c_str());

                    patterns.push_back(
                            AssetBuilderSDK::AssetBuilderPattern(
                                wildcardPattern, AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));
                }
                return AZ::SettingsRegistryInterface::VisitResponse::Continue;
            };
        AZ::SettingsRegistryVisitorUtils::VisitArray(*settingsRegistry, VisitFileTypeExtensions, SdfAssetBuilderSupportedFileExtensionsRegistryKey);

        AZ_Warning(SdfAssetBuilderName, !patterns.empty(), "SdfAssetBuilder disabled, no supported file type extensions found.");
        return patterns;
    }

    void SdfAssetBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request,
        AssetBuilderSDK::CreateJobsResponse& response) const
    {
        // To be able to successfully process the SDF job, we need job dependencies on every asset
        // referenced by the SDF file. Otherwise we won't be able to connect the references to the 
        // correct product assets. Unfortunately, this means that we need to redundantly parse the
        // source file once here to set up the job dependencies, and then we'll parse it a second
        // time when actually running ProcessJob().

        // Eventually, we may need to extend the logic here even further to create more asset
        // generation jobs for exporting any embedded model / material / collider assets that only
        // exist inside the SDF file and not as an external reference.

        const auto fullSourcePath = AZ::IO::Path(request.m_watchFolder) / AZ::IO::Path(request.m_sourceFile);

        AZ_Info(SdfAssetBuilderName, "Parsing source file: %s", fullSourcePath.c_str());
        auto parsedSourceFile = UrdfParser::ParseFromFile(fullSourcePath.String());
        if (!parsedSourceFile)
        {
            AZ_Error(SdfAssetBuilderName, false, "Failed to parse source file '%s'.", fullSourcePath.c_str());
            return;
        }

        AZ_Info(SdfAssetBuilderName, "Finding asset IDs for all mesh and collider assets.");
        auto sourceAssetMap = AZStd::make_shared<Utils::UrdfAssetMap>(FindAssets(parsedSourceFile->getRoot(), fullSourcePath.String()));

        // Create an output job for each platform
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "SDF (Simulation Description Format) Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            // Add in all of the job dependencies for this file.
            // The SDF file won't get processed until every asset it relies on has been processed first.
            for (const auto& asset : *sourceAssetMap)
            {
                AssetBuilderSDK::JobDependency jobDependency;
                jobDependency.m_sourceFile.m_sourceFileDependencyUUID = asset.second.m_availableAssetInfo.m_sourceGuid;
                jobDependency.m_platformIdentifier = platformInfo.m_identifier;
                jobDependency.m_type = AssetBuilderSDK::JobDependencyType::Order;
                jobDescriptor.m_jobDependencyList.push_back(AZStd::move(jobDependency));
            }

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    Utils::UrdfAssetMap SdfAssetBuilder::FindAssets(const urdf::LinkConstSharedPtr& rootLink, const AZStd::string& sourceFilename) const
    {
        AZ_Info(SdfAssetBuilderName, "Parsing mesh and collider names");
        auto assetNames = Utils::GetMeshesFilenames(rootLink, true, true);

        Utils::UrdfAssetMap assetMap;

        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;

        // Unlike the RobotImporter, the SDF Asset Builder does not use the AMENT_PREFIX_PATH
        // to resolve file locations. There wouldn't be a way to guarantee identical results across
        // machines or to detect the need to rebuild assets if the environment variable changes.
        const AZStd::string emptyAmentPrefixPath;

        for (const auto& uri : assetNames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = uri;

            // Attempt to find the absolute path for the raw uri reference, which might look something like "model://meshes/model.dae"
            asset.m_resolvedUrdfPath = Utils::ResolveURDFPath(asset.m_urdfPath, sourceFilename, emptyAmentPrefixPath);
            if (asset.m_resolvedUrdfPath.empty())
            {
                AZ_Warning(SdfAssetBuilderName, false, "Failed to resolve file reference '%s' to an absolute path, skipping.", uri.c_str());
                continue;
            }

            // Get a checksum on the resolved file that we'll use to ensure we've matched with the correct relative
            // source asset. Ideally we'll be able to remove this check since it's not a very robust safety check and
            // adds some amount of overhead to the processing.
            asset.m_urdfFileCRC = Utils::GetFileCRC(asset.m_resolvedUrdfPath);

            // Given the absolute path to the asset, try to get the source asset info from the AssetProcessor.
            bool sourceAssetFound{ false };
            AZ::Data::AssetInfo assetInfo;
            AZStd::string watchFolder;
            AssetSysReqBus::BroadcastResult(
                sourceAssetFound, &AssetSysReqBus::Events::GetSourceInfoBySourcePath, 
                asset.m_resolvedUrdfPath.c_str(), assetInfo, watchFolder);

            if (!sourceAssetFound)
            {
                AZ_Warning(SdfAssetBuilderName, false, "Cannot find source asset info for '%s', skipping.", asset.m_resolvedUrdfPath.c_str());
                continue;
            }

            // If the source asset has been found by the Asset Processor, save the mapping between raw uri
            // reference and Asset Processor source asset information.
            const auto fullSourcePath = AZ::IO::Path(watchFolder) / AZ::IO::Path(assetInfo.m_relativePath);
            asset.m_availableAssetInfo.m_sourceAssetRelativePath = assetInfo.m_relativePath;
            asset.m_availableAssetInfo.m_sourceAssetGlobalPath = fullSourcePath.String();
            asset.m_availableAssetInfo.m_sourceGuid = assetInfo.m_assetId.m_guid;

            // We should determine if this CRC check is actually necessary for resolving URI references.
            // Ideally, the additional overhead should be removed and the asset reference result should be trusted.
            AZ::Crc32 crc = Utils::GetFileCRC(asset.m_availableAssetInfo.m_sourceAssetGlobalPath);
            if (crc == asset.m_urdfFileCRC)
            {
                AZ_Info(SdfAssetBuilderName, "Resolved uri '%s' to source asset '%s'.", uri.c_str(), assetInfo.m_relativePath.c_str());
                assetMap.emplace(uri, AZStd::move(asset));
            }
            else
            {
                AZ_Warning(SdfAssetBuilderName, false, "Resolved to source asset '%s' which has incorrect CRC, skipping.", assetInfo.m_relativePath.c_str());
            }
        }

        return assetMap;
    }


    void SdfAssetBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request,
        AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // Set whether or not the outputs should use PhysX articulation components for joints.
        // This should eventually get moved into a global and/or per-file asset builder setting.
        const bool useArticulation = true;

        auto tempAssetOutputPath = AZ::IO::Path(request.m_tempDirPath) / request.m_sourceFile;
        tempAssetOutputPath.ReplaceExtension("procprefab");

        // Read in and parse the source SDF file.
        AZ_Info(SdfAssetBuilderName, "Parsing source file: %s", request.m_fullPath.c_str());
        auto parsedSourceFile = UrdfParser::ParseFromFile(request.m_fullPath);
        if (!parsedSourceFile)
        {
            AZ_Error(SdfAssetBuilderName, false, "Failed to parse source file '%s'.", request.m_fullPath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // Resolve all the URI references into source asset GUIDs.
        AZ_Info(SdfAssetBuilderName, "Finding asset IDs for all mesh and collider assets.");
        auto assetMap = AZStd::make_shared<Utils::UrdfAssetMap>(FindAssets(parsedSourceFile->getRoot(), request.m_fullPath));

        // Given the parsed source file and asset mappings, generate an in-memory prefab.
        AZ_Info(SdfAssetBuilderName, "Creating prefab from source file.");
        auto prefabMaker = AZStd::make_unique<URDFPrefabMaker>(
            request.m_fullPath, parsedSourceFile, tempAssetOutputPath.String(), assetMap, useArticulation);
        AZStd::string outputPrefab;
        auto prefabResult = prefabMaker->CreatePrefabStringFromURDF(outputPrefab);
        if (!prefabResult.IsSuccess())
        {
            AZ_Error(
                SdfAssetBuilderName,
                false,
                "Failed to create proc prefab file '%s'. Error message: %s",
                tempAssetOutputPath.c_str(),
                prefabResult.GetError().c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // If no prefab string was generated, then return a failure.
        if (outputPrefab.empty())
        {
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // Save the prefab to our output directory.
        rapidjson::Document doc;
        doc.Parse(outputPrefab.c_str());
        auto outcome = AZ::JsonSerializationUtils::WriteJsonFile(doc, tempAssetOutputPath.c_str());
        if (!outcome.IsSuccess())
        {
            AZ_Error(SdfAssetBuilderName, false, "Failed to write out temp asset file '%s'. Error message: %s", 
                tempAssetOutputPath.c_str(), outcome.GetError().c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        AZ_Info(SdfAssetBuilderName, "Prefab creation completed successfully.");
        
        // Mark the resulting prefab as a product asset with the "procedural prefab" asset type.
        AssetBuilderSDK::JobProduct sdfJobProduct;
        sdfJobProduct.m_productFileName = tempAssetOutputPath.String();
        sdfJobProduct.m_productSubID = 0;
        sdfJobProduct.m_productAssetType = azrtti_typeid<AZ::Prefab::ProceduralPrefabAsset>();

        // Right now, just mark that dependencies are handled because there aren't any to handle.
        // It seems that procedural prefabs don't declare product asset dependencies, presumably because
        // they're still a source asset themselves and not a product asset.
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
        AZ::IO::FixedMaxPath prefabTemplateName{ AZ::IO::PathView(request.m_sourceFile).FixedMaxPathStringAsPosix() };
        prefabTemplateName.ReplaceExtension("procprefab");

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
            prefabTemplateName.String());

        if (prefabTemplateId == AzToolsFramework::Prefab::InvalidTemplateId)
        {
            AZ_Error(SdfAssetBuilderName, false, "Could not create a prefab template for entities.");
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
            AZ_Error(SdfAssetBuilderName, false, "Could not serialize prefab template as a JSON string");
            return {};
        }

        return outcome.GetValue();
    }

} // ROS2
