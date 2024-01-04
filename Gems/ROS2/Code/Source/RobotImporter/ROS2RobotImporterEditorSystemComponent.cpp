/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterEditorSystemComponent.h"
#include "RobotImporterWidget.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/utility/move.h>
#include <AzToolsFramework/API/ViewPaneOptions.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include <RobotImporter/Utils/FilePath.h>
#include <SDFormat/ROS2ModelPluginHooks.h>
#include <SDFormat/ROS2SensorHooks.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <sdf/sdf.hh>

#if !defined(Q_MOC_RUN)
#include <QWindow>
#endif

namespace ROS2
{
    void ROS2RobotImporterEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2RobotImporterEditorSystemComponent, ROS2RobotImporterSystemComponent>()->Version(1);
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<RobotImporterRequestBus>("RobotImporterBus")
                ->Attribute(AZ::Script::Attributes::Category, "Robotics")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Automation)
                ->Attribute(AZ::Script::Attributes::Module, "ROS2")
                ->Event("ImportURDF", &RobotImporterRequestBus::Events::GeneratePrefabFromFile);
        }
    }

    void ROS2RobotImporterEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        ROS2RobotImporterSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2RobotImporterEditorService"));
    }

    void ROS2RobotImporterEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        ROS2RobotImporterSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2RobotImporterEditorService"));
    }

    void ROS2RobotImporterEditorSystemComponent::Activate()
    {
        ROS2RobotImporterSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        RobotImporterRequestBus::Handler::BusConnect();

        // Register default sensor and plugin hooks
        m_sensorHooks.emplace_back(SDFormat::ROS2SensorHooks::ROS2CameraSensor());
        m_sensorHooks.emplace_back(SDFormat::ROS2SensorHooks::ROS2GNSSSensor());
        m_sensorHooks.emplace_back(SDFormat::ROS2SensorHooks::ROS2ImuSensor());
        m_sensorHooks.emplace_back(SDFormat::ROS2SensorHooks::ROS2LidarSensor());
	m_modelPluginHooks.emplace_back(SDFormat::ROS2ModelPluginHooks::ROS2AckermannModel());
        m_modelPluginHooks.emplace_back(SDFormat::ROS2ModelPluginHooks::ROS2SkidSteeringModel());

        // Query user-defined sensor and plugin hooks
        auto serializeContext = AZ::Interface<AZ::ComponentApplicationRequests>::Get()->GetSerializeContext();
        serializeContext->EnumerateAll(
            [&](const AZ::SerializeContext::ClassData* classData, const AZ::Uuid& typeId) -> bool
            {
                return CopyHooksCallback<SDFormat::SensorImporterHooksStorage>(m_sensorHooks, classData, "SensorImporterHooks");
            });

        serializeContext->EnumerateAll(
            [&](const AZ::SerializeContext::ClassData* classData, const AZ::Uuid& typeId) -> bool
            {
                return CopyHooksCallback<SDFormat::ModelPluginImporterHooksStorage>(
                    m_modelPluginHooks, classData, "ModelPluginImporterHooks");
            });
    }

    void ROS2RobotImporterEditorSystemComponent::Deactivate()
    {
        RobotImporterRequestBus::Handler::BusDisconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        ROS2RobotImporterSystemComponent::Deactivate();
    }

    void ROS2RobotImporterEditorSystemComponent::NotifyRegisterViews()
    {
        AzToolsFramework::ViewPaneOptions options;
        options.showOnToolsToolbar = true;
        options.isDockable = false;
        options.detachedWindow = true;
        options.canHaveMultipleInstances = false;
        options.isDisabledInSimMode = true;
        options.isDeletable = true;

        options.toolbarIcon = ":/ROS2/ROS_import_icon.svg";
        AzToolsFramework::RegisterViewPane<RobotImporterWidget>("Robot Importer", "ROS2", options);
    }

    bool ROS2RobotImporterEditorSystemComponent::GeneratePrefabFromFile(
        const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation)
    {
        if (filePath.empty())
        {
            AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "Path provided for prefab is empty");
            return false;
        }
        if (Utils::IsFileXacro(filePath))
        {
            AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "XACRO formatted files are not supported");
            return false;
        }

        // Read the SDF Settings from the Settings Registry into a local struct
        SdfAssetBuilderSettings sdfBuilderSettings;
        sdfBuilderSettings.LoadSettings();
        // Set the parser config settings for URDF content
        sdf::ParserConfig parserConfig = Utils::SDFormat::CreateSdfParserConfigFromSettings(sdfBuilderSettings, filePath);

        auto parsedSdfOutcome = UrdfParser::ParseFromFile(filePath, parserConfig, sdfBuilderSettings);
        if (!parsedSdfOutcome)
        {
            const AZStd::string log = Utils::JoinSdfErrorsToString(parsedSdfOutcome.GetSdfErrors());

            AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "URDF/SDF parsing failed with errors:\nRefer to %s", log.c_str());
            return false;
        }

        // Urdf Root has been parsed successfully retrieve it from the Outcome
        const sdf::Root& parsedSdfRoot = parsedSdfOutcome.GetRoot();

        auto assetNames = Utils::GetReferencedAssetFilenames(parsedSdfRoot);
        AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>();
        if (importAssetWithUrdf)
        {
            urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(
                Utils::CopyReferencedAssetsAndCreateAssetMap(assetNames, filePath, sdfBuilderSettings));
        }
        bool allAssetProcessed = false;
        bool assetProcessorFailed = false;

        auto loopStartTime = AZStd::chrono::system_clock::now();

        /* This loop waits until all of the assets are processed.
           The urdf prefab cannot be created before all assets are processed.
           There are three stop conditions: allAssetProcessed, assetProcessorFailed and a timeout.
           After all asset are processed the allAssetProcessed will be set to true.
           assetProcessorFailed will be set to true if the asset processor does not respond.
           The time out will break the loop if assetLoopTimeout is exceed. */
        while (!allAssetProcessed && !assetProcessorFailed)
        {
            auto loopTime = AZStd::chrono::system_clock::now();

            if (loopTime - loopStartTime > assetLoopTimeout)
            {
                AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "Loop waiting for assets timed out");
                break;
            }

            allAssetProcessed = true;
            for (const auto& [name, asset] : *urdfAssetsMapping)
            {
                auto sourceAssetFullPath = asset.m_availableAssetInfo.m_sourceAssetGlobalPath;
                if (sourceAssetFullPath.empty())
                {
                    AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "Asset %s missing `sourceAssetFullPath`", name.c_str());
                    continue;
                }
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;
                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath.Native(), true);

                if (!result.IsSuccess())
                {
                    assetProcessorFailed = true;
                    AZ_Error("ROS2RobotImporterEditorSystemComponent", false, "Asset System failed to reply with jobs infos");
                    break;
                }

                JobInfoContainer& allJobs = result.GetValue();
                for (const JobInfo& job : allJobs)
                {
                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                    {
                        AZ_Printf("ROS2RobotImporterEditorSystemComponent", "asset %s is being processed\n", sourceAssetFullPath.c_str());
                        allAssetProcessed = false;
                    }
                    else
                    {
                        AZ_Printf("ROS2RobotImporterEditorSystemComponent", "asset %s is done\n", sourceAssetFullPath.c_str());
                    }
                }
            }

            if (allAssetProcessed && !assetProcessorFailed)
            {
                AZ_Printf("ROS2RobotImporterEditorSystemComponent", "All assets processed\n");
            }
        };

        // Use the URDF/SDF file name stem the prefab name
        auto fileStem = AZ::IO::PathView(filePath).Stem();
        AZStd::string prefabName = AZStd::string::format("%.*s.prefab", AZ_PATH_ARG(fileStem));

        if (prefabName.empty())
        {
            AZ_Error(
                "ROS2RobotImporterEditorSystemComponent",
                false,
                R"(URDF/SDF doesn't filename doesn't contain a stem "%.*s".)"
                " O3DE Prefab cannot be created",
                AZ_STRING_ARG(filePath));
            return false;
        }

        const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
        AZStd::unique_ptr<URDFPrefabMaker> prefabMaker =
            AZStd::make_unique<URDFPrefabMaker>(filePath, &parsedSdfRoot, prefabPath.String(), urdfAssetsMapping, useArticulation);

        auto prefabOutcome = prefabMaker->CreatePrefabFromUrdfOrSdf();

        if (!prefabOutcome.IsSuccess())
        {
            AZ_Error(
                "ROS2RobotImporterEditorSystemComponent",
                false,
                "Unable to create Prefab from URDF/SDF file %.*s",
                AZ_STRING_ARG(filePath));
            return false;
        }

        return true;
    }

    const SDFormat::SensorImporterHooksStorage& ROS2RobotImporterEditorSystemComponent::GetSensorHooks() const
    {
        return m_sensorHooks;
    }

    const SDFormat::ModelPluginImporterHooksStorage& ROS2RobotImporterEditorSystemComponent::GetModelPluginHooks() const
    {
        return m_modelPluginHooks;
    }

} // namespace ROS2
