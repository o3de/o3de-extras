/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterEditorSystemComponent.h"
#include "RobotImporter/URDF/UrdfParser.h"
#include "RobotImporter/Utils/FilePath.h"
#include "RobotImporterWidget.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/std/chrono/chrono.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/API/ViewPaneOptions.h>
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
            AZ_Warning("ROS2EditorSystemComponent", false, "Path provided for prefab is empty");
            return false;
        }
        if (Utils::IsFileXacro(filePath))
        {
            AZ_Warning("ROS2EditorSystemComponent", false, "XACRO formatted files are not supported");
            return false;
        }

        urdf::ModelInterfaceSharedPtr parsedUrdf = UrdfParser::ParseFromFile(filePath);
        if (!parsedUrdf)
        {
            const auto log = UrdfParser::GetUrdfParsingLog();
            AZ_Warning("ROS2EditorSystemComponent", false, "URDF parsing failed. Refer to %s", log.c_str());
            return false;
        }

        auto collidersNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), false, true);
        auto visualNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), true, false);
        auto meshNames = Utils::GetMeshesFilenames(parsedUrdf->getRoot(), true, true);
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
                AZ_Warning("ROS2EditorSystemComponent", false, "Loop waiting for assets timed out");
                break;
            }

            allAssetProcessed = true;
            for (const auto& [name, asset] : *urdfAssetsMapping)
            {
                auto sourceAssetFullPath = asset.m_availableAssetInfo.m_sourceAssetGlobalPath;
                if (sourceAssetFullPath.empty())
                {
                    AZ_Warning("ROS2EditorSystemComponent", false, "Asset %s missing `sourceAssetFullPath`", name.c_str());
                    continue;
                }
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;
                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);

                if (!result.IsSuccess())
                {
                    assetProcessorFailed = true;
                    AZ_Error("ROS2EditorSystemComponent", false, "Asset System failed to reply with jobs infos");
                    break;
                }

                JobInfoContainer& allJobs = result.GetValue();
                for (const JobInfo& job : allJobs)
                {
                    if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                    {
                        AZ_Printf("ROS2EditorSystemComponent", "asset %s is being processed", sourceAssetFullPath.c_str());
                        allAssetProcessed = false;
                    }
                    else
                    {
                        AZ_Printf("ROS2EditorSystemComponent", "asset %s is done", sourceAssetFullPath.c_str());
                    }
                }
            }

            if (allAssetProcessed && !assetProcessorFailed)
            {
                AZ_Printf("ROS2EditorSystemComponent", "All assets processed");
            }
        };

        AZStd::string prefabName = AZStd::string(parsedUrdf->getName().c_str(), parsedUrdf->getName().size()) + ".prefab";

        const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
        AZStd::unique_ptr<URDFPrefabMaker> prefabMaker;
        prefabMaker = AZStd::make_unique<URDFPrefabMaker>(filePath, parsedUrdf, prefabPath.String(), urdfAssetsMapping, useArticulation);

        auto prefabOutcome = prefabMaker->CreatePrefabFromURDF();

        if (!prefabOutcome.IsSuccess())
        {
            AZ_Error("ROS2EditorSystemComponent", false, "Unable to create Prefab from URDF file %s", filePath.data());
            return false;
        }

        return true;
    }

} // namespace ROS2
