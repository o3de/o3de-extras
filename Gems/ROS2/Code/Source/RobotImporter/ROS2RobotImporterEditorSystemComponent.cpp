/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterEditorSystemComponent.h"
#include <RobotImporter/URDF/UrdfParser.h>
#include <RobotImporter/Utils/FilePath.h>
#include <RobotImporter/Utils/ErrorUtils.h>
#include "RobotImporterWidget.h"
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/string/string.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzToolsFramework/API/ViewPaneOptions.h>


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
            serializeContext->Class<ROS2RobotImporterEditorSystemComponent, ROS2RobotImporterSystemComponent>()->Version(0);
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
        sdf::ParserConfig parserConfig;
        parserConfig.URDFSetPreserveFixedJoint(sdfBuilderSettings.m_urdfPreserveFixedJoints);

        auto parsedUrdfOutcome = UrdfParser::ParseFromFile(filePath, parserConfig);
        if (!parsedUrdfOutcome)
        {
            const AZStd::string log = Utils::JoinSdfErrorsToString(parsedUrdfOutcome.error());

            AZ_Warning("ROS2RobotImporterEditorSystemComponent", false, "URDF parsing failed with errors:\nRefer to %s",
                log.c_str());
            return false;
        }

        // Urdf Root has been parsed successfully retrieve it from the Outcome
        const sdf::Root& parsedUrdfRoot = parsedUrdfOutcome.value();

        auto collidersNames = Utils::GetMeshesFilenames(&parsedUrdfRoot, false, true);
        auto visualNames = Utils::GetMeshesFilenames(&parsedUrdfRoot, true, false);
        auto meshNames = Utils::GetMeshesFilenames(&parsedUrdfRoot, true, true);
        AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>();
        if (importAssetWithUrdf)
        {
            urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(
                Utils::CopyAssetForURDFAndCreateAssetMap(meshNames, filePath, collidersNames, visualNames));
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
                        AZ_Printf("ROS2RobotImporterEditorSystemComponent", "asset %s is being processed", sourceAssetFullPath.c_str());
                        allAssetProcessed = false;
                    }
                    else
                    {
                        AZ_Printf("ROS2RobotImporterEditorSystemComponent", "asset %s is done", sourceAssetFullPath.c_str());
                    }
                }
            }

            if (allAssetProcessed && !assetProcessorFailed)
            {
                AZ_Printf("ROS2RobotImporterEditorSystemComponent", "All assets processed");
            }
        };

        // Use the name of the first model tag in the SDF for the prefab
        // Otherwise use the name of the first world tag in the SDF
        AZStd::string prefabName;
        if (const sdf::Model* model = parsedUrdfRoot.Model();
            model != nullptr)
        {
            prefabName = AZStd::string(model->Name().c_str(), model->Name().size()) + ".prefab";
        }

        if (uint64_t urdfWorldCount = parsedUrdfRoot.WorldCount();
            prefabName.empty() && urdfWorldCount > 0)
        {
            const sdf::World* parsedUrdfWorld = parsedUrdfRoot.WorldByIndex(0);
            prefabName = AZStd::string(parsedUrdfWorld->Name().c_str(), parsedUrdfWorld->Name().size()) + ".prefab";
        }

        if (prefabName.empty())
        {
            AZ_Error("ROS2RobotImporterEditorSystemComponent", false, "URDF file converted to SDF %.*s contains no worlds."
                " O3DE Prefab cannot be created", AZ_STRING_ARG(filePath));
            return false;
        }


        const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
        AZStd::unique_ptr<URDFPrefabMaker> prefabMaker = AZStd::make_unique<URDFPrefabMaker>(filePath, &parsedUrdfRoot, prefabPath.String(), urdfAssetsMapping, useArticulation);

        auto prefabOutcome = prefabMaker->CreatePrefabFromURDF();

        if (!prefabOutcome.IsSuccess())
        {
            AZ_Error("ROS2RobotImporterEditorSystemComponent", false, "Unable to create Prefab from URDF file %s", filePath.data());
            return false;
        }

        return true;
    }

} // namespace ROS2
