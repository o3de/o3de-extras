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
#include <AzToolsFramework/Prefab/PrefabLoaderInterface.h>
#include <AzToolsFramework/Prefab/PrefabLoaderScriptingBus.h>
#include <AzToolsFramework/Prefab/PrefabSystemComponentInterface.h>
#include <AzToolsFramework/Prefab/PrefabSystemScriptingBus.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>

#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include <RobotImporter/URDF/URDFPrefabMaker.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <Utils/RobotImporterUtils.h>

namespace ROS2
{
    inline namespace SDFAssetBuilderInternal
    {
        constexpr const char* SdfAssetBuilderJobKey = "Sdf Asset Builder";
    }

    SdfAssetBuilder::SdfAssetBuilder()
    {
        // Read in all of the global settings from the settings registry.
        m_globalSettings.LoadSettings();

        // Turn our global settings into a cached fingerprint that we'll use on every job
        // so that we can detect when to rebuild assets on global setting changes.
        // The fingerprint should only use the global builder settings, not the per-file settings.
        // The analysis fingerprint is set at the builder level, outside of any individual files,
        // so it should only include data that's invariant across files.
        // Per-file settings changes will cause rebuilds of individual files through a separate
        // mechanism in the Asset Processor that detects when an associated metadata settings file changes.
        m_fingerprint = GetFingerprint();

        AssetBuilderSDK::AssetBuilderDesc sdfAssetBuilderDescriptor;

        sdfAssetBuilderDescriptor.m_name = SdfAssetBuilderJobKey;
        sdfAssetBuilderDescriptor.m_version = 1; // bump this to rebuild all sdf files
        sdfAssetBuilderDescriptor.m_busId = azrtti_typeid<SdfAssetBuilder>();
        sdfAssetBuilderDescriptor.m_patterns = m_globalSettings.m_builderPatterns;
        sdfAssetBuilderDescriptor.m_analysisFingerprint = m_fingerprint; // set the fingerprint to the global settings

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

    Utils::UrdfAssetMap SdfAssetBuilder::FindAssets(const sdf::Root& root, const AZStd::string& sourceFilename) const
    {
        AZ_Info(SdfAssetBuilderName, "Parsing mesh and collider names");
        auto assetNames = Utils::GetReferencedAssetFilenames(root);

        Utils::UrdfAssetMap assetMap;

        using AssetSysReqBus = AzToolsFramework::AssetSystemRequestBus;

        auto amentPrefixPath = Utils::GetAmentPrefixPath();

        for (const auto& [uri, assetReferenceType] : assetNames)
        {
            Utils::UrdfAsset asset;
            asset.m_urdfPath = uri;

            // Attempt to find the absolute path for the raw uri reference, which might look something like "model://meshes/model.dae"
            asset.m_resolvedUrdfPath = Utils::ResolveAssetPath(asset.m_urdfPath, AZ::IO::PathView(sourceFilename), amentPrefixPath,
                m_globalSettings);
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

    AZStd::string SdfAssetBuilder::GetFingerprint() const
    {
        AZStd::string settingsString;
        AZ::IO::ByteContainerStream<AZStd::string> stream { &settingsString };

        // Always write all of the settings, even when they match the defaults.
        // This isn't strictly necessary, but this way if the default values ever get changed
        // we'll still be able to detect that the settings don't match what they used to be.
        AZ::JsonSerializerSettings jsonSettings;
        jsonSettings.m_keepDefaults = true;

        [[maybe_unused]] AZ::Outcome<void, AZStd::string> saveObjectResult =
            AZ::JsonSerializationUtils::SaveObjectToStream(&m_globalSettings, stream, {}, &jsonSettings);
        AZ_Assert(saveObjectResult.IsSuccess(), "Failed to save settings to fingerprint string: %s",
            saveObjectResult.GetError().c_str());

        return settingsString;
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

        // Set the parser config settings for parsing URDF content through the libsdformat parser
        sdf::ParserConfig parserConfig = Utils::SDFormat::CreateSdfParserConfigFromSettings(m_globalSettings, fullSourcePath);

        AZ_Info(SdfAssetBuilderName, "Parsing source file: %s", fullSourcePath.c_str());
        auto parsedSdfRootOutcome = UrdfParser::ParseFromFile(fullSourcePath, parserConfig, m_globalSettings);
        if (!parsedSdfRootOutcome)
        {
            const AZStd::string sdfParseErrors = Utils::JoinSdfErrorsToString(parsedSdfRootOutcome.GetSdfErrors());
            AZ_Error(SdfAssetBuilderName, false, R"(Failed to parse source file "%s". Errors: "%s")",
                fullSourcePath.c_str(), sdfParseErrors.c_str());
            return;
        }

        const sdf::Root& sdfRoot = parsedSdfRootOutcome.GetRoot();

        AZ_Info(SdfAssetBuilderName, "Finding asset IDs for all mesh and collider assets.");
        auto sourceAssetMap = AZStd::make_shared<Utils::UrdfAssetMap>(FindAssets(sdfRoot, fullSourcePath.String()));

        // Create an output job for each platform
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "SDF (Simulation Description Format) Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            // This fingerprint should only include the global builder settings, not the individual file settings.
            // The Asset Processor will detect when the per-file settings change and will trigger a rebuild without
            // requiring the settings to be in the fingerprint.
            jobDescriptor.m_additionalFingerprintInfo = m_fingerprint;

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

            // Eventually this should also add a job dependency on 'wheel_material.physicsmaterial'
            // if that material gets used by the colliders.

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void SdfAssetBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request,
        AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // Set whether or not the outputs should use PhysX articulation components for joints.
        const bool useArticulation = m_globalSettings.m_useArticulations;

        auto tempAssetOutputPath = AZ::IO::Path(request.m_tempDirPath) / request.m_sourceFile;
        tempAssetOutputPath.ReplaceExtension("procprefab");

        // Set the parser config settings for parsing URDF content through the libsdformat parser
        sdf::ParserConfig parserConfig = Utils::SDFormat::CreateSdfParserConfigFromSettings(m_globalSettings, AZ::IO::PathView(request.m_sourceFile));

        // Read in and parse the source SDF file.
        AZ_Info(SdfAssetBuilderName, "Parsing source file: %s", request.m_fullPath.c_str());
        auto parsedSdfRootOutcome = UrdfParser::ParseFromFile(AZ::IO::PathView(request.m_fullPath), parserConfig, m_globalSettings);
        if (!parsedSdfRootOutcome)
        {
            const AZStd::string sdfParseErrors = Utils::JoinSdfErrorsToString(parsedSdfRootOutcome.GetSdfErrors());
            AZ_Error(SdfAssetBuilderName, false, R"(Failed to parse source file "%s". Errors: "%s")",
                request.m_fullPath.c_str(), sdfParseErrors.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        const sdf::Root& sdfRoot = parsedSdfRootOutcome.GetRoot();

        // Resolve all the URI references into source asset GUIDs.
        AZ_Info(SdfAssetBuilderName, "Finding asset IDs for all mesh and collider assets.");
        auto assetMap = AZStd::make_shared<Utils::UrdfAssetMap>(FindAssets(sdfRoot, request.m_fullPath));

        // Given the parsed source file and asset mappings, generate an in-memory prefab.
        AZ_Info(SdfAssetBuilderName, "Creating prefab from source file.");
        auto prefabMaker = AZStd::make_unique<URDFPrefabMaker>(
            request.m_fullPath, &sdfRoot, tempAssetOutputPath.String(), assetMap, useArticulation);
        auto prefabResult = prefabMaker->CreatePrefabTemplateFromUrdfOrSdf();
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

        // Save Template to file
        auto prefabLoaderInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabLoaderInterface>::Get();
        bool saveResult = prefabLoaderInterface->SaveTemplateToFile(prefabResult.GetValue(), tempAssetOutputPath.c_str());

        if (!saveResult)
        {
            AZ_Error(SdfAssetBuilderName, false, "Failed to write out temp asset file '%s'.",
                tempAssetOutputPath.c_str());
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

} // ROS2
