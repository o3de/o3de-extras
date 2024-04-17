/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerComponentController.h"
#include "Spawner/ROS2SpawnPointComponent.h"
#include "Spawner/ROS2SpawnerComponent.h"
#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{

    void ServiceNames::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ServiceNames>()
                ->Version(1)
                ->Field("Get available spawnable names service name", &ServiceNames::availableSpawnableNamesServiceName)
                ->Field("Spawn entity service name", &ServiceNames::spawnEntityServiceName)
                ->Field("Get spawn point info service name", &ServiceNames::spawnPointInfoServiceName)
                ->Field("Get spawn points names service name", &ServiceNames::spawnPointsNamesServiceName);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ServiceNames>("ServiceNames", "Service names for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ServiceNames::availableSpawnableNamesServiceName,
                        "Get available spawnable names service name",
                        "Service name for getting available spawnable names")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ServiceNames::spawnEntityServiceName,
                        "Spawn entity service name",
                        "Service name for spawning entity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ServiceNames::spawnPointInfoServiceName,
                        "Get spawn point info service name",
                        "Service name for getting spawn point info")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ServiceNames::spawnPointsNamesServiceName,
                        "Get spawn points names service name",
                        "Service name for getting spawn points names");
            }
        }
    }

    void ROS2SpawnerComponentConfig::Reflect(AZ::ReflectContext* context)
    {

        ServiceNames::Reflect(context); 
        
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnerComponentConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("Editor entity id", &ROS2SpawnerComponentConfig::m_editorEntityId)
                ->Field("Spawnables", &ROS2SpawnerComponentConfig::m_spawnables)
                ->Field("Default spawn pose", &ROS2SpawnerComponentConfig::m_defaultSpawnPose)
                ->Field("Service names", &ROS2SpawnerComponentConfig::m_serviceNames);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ROS2SpawnerComponentConfig>("ROS2SpawnerComponentConfig", "Config for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentConfig::m_spawnables, "Spawnables", "Spawnables")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SpawnerComponentConfig::m_defaultSpawnPose,
                        "Default spawn pose",
                        "Default spawn pose")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentConfig::m_serviceNames, "Service names", "Service names");
            }
        }
    }

    AZ::EntityId ROS2SpawnerComponentController::GetEditorEntityId() const
    {
        return m_config.m_editorEntityId;
    }

    AZStd::unordered_map<AZStd::string, AZ::Data::Asset<AzFramework::Spawnable>> ROS2SpawnerComponentController::GetSpawnables() const
    {
        return m_config.m_spawnables;
    }

    const AZ::Transform& ROS2SpawnerComponentController::GetDefaultSpawnPose() const
    {
        return m_config.m_defaultSpawnPose;
    }

    const ServiceNames ROS2SpawnerComponentController::GetServiceNames() const
    {
        return m_config.m_serviceNames;
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponentController::GetAllSpawnPointInfos() const
    {
        return GetSpawnPoints();
    }

    void ROS2SpawnerComponentController::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerComponentConfig::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnerComponentController>()->Version(1)->Field(
                "Configuration", &ROS2SpawnerComponentController::m_config);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<ROS2SpawnerComponentController>("ROS2SpawnerComponentController", "Controller for ROS2SpawnerComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages spawning of robots in configurable locations")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnerComponentController::m_config)
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerComponentController::GetSpawnPoints() const
    {
        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, m_config.m_editorEntityId, &AZ::TransformBus::Events::GetChildren);

        AZStd::unordered_map<AZStd::string, SpawnPointInfo> result;

        for (const AZ::EntityId& child : children)
        {
            AZ::Entity* childEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(childEntity, &AZ::ComponentApplicationRequests::FindEntity, child);
            AZ_Assert(childEntity, "No child entity found for entity %s", child.ToString().c_str());

            if (const auto* spawnPoint = childEntity->FindComponent<ROS2SpawnPointComponent>(); spawnPoint != nullptr)
            {
                result.insert(spawnPoint->GetInfo());
            }
        }

        // setting name of spawn point component "default" in a child entity will have no effect since it is overwritten here with the
        // default spawn pose of spawner
        result["default"] = SpawnPointInfo{ "Default spawn pose defined in the Editor", m_config.m_defaultSpawnPose };
        return result;
    }

    void ROS2SpawnerComponentController::Init()
    {
    }

    void ROS2SpawnerComponentController::Activate(AZ::EntityId entityId)
    {
        m_config.m_editorEntityId = entityId;
        SpawnerRequestsBus::Handler::BusConnect(entityId);
    }

    void ROS2SpawnerComponentController::Deactivate()
    {
        SpawnerRequestsBus::Handler::BusDisconnect();
    }

    ROS2SpawnerComponentController::ROS2SpawnerComponentController(const ROS2SpawnerComponentConfig& config)
    {
        SetConfiguration(config);
    }

    void ROS2SpawnerComponentController::SetConfiguration(const ROS2SpawnerComponentConfig& config)
    {
        m_config = config;
    }

    const ROS2SpawnerComponentConfig& ROS2SpawnerComponentController::GetConfiguration() const
    {
        return m_config;
    }

} // namespace ROS2
