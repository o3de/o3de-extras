/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnerEditorComponent.h"
#include "AzCore/Debug/Trace.h"
#include "ROS2SpawnPointEditorComponent.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Spawner/SpawnerBus.h>

namespace ROS2
{
    ROS2SpawnerEditorComponent::ROS2SpawnerEditorComponent(const ROS2SpawnerComponentConfig& configuration)
        : ROS2SpawnerEditorComponentBase(configuration)
    {
    }

    void ROS2SpawnerEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnerEditorComponentBase::Reflect(context);

        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<ROS2SpawnerEditorComponent, ROS2SpawnerEditorComponentBase>()->Version(1);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<ROS2SpawnerEditorComponent>("ROS2 Spawner", "Spawner component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages spawning of robots in configurable locations")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2Spawner.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2Spawner.svg");
            }
        }
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerEditorComponent::GetSpawnPoints() const
    {
        AZStd::unordered_map<AZStd::string, SpawnPointInfo> result;
        result[GetEntity()->GetName() + " - default"] =
            SpawnPointInfo{ "Default spawn pose defined in the Editor", m_controller.GetDefaultSpawnPose() };

        AZStd::vector<AZ::EntityId> children;
        AZ::TransformBus::EventResult(children, m_controller.GetEditorEntityId(), &AZ::TransformBus::Events::GetChildren);

        for (const AZ::EntityId& child : children)
        {
            AZ::Entity* childEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(childEntity, &AZ::ComponentApplicationRequests::FindEntity, child);
            AZ_Assert(childEntity, "No child entity found for entity %s", child.ToString().c_str());

            const auto* editorSpawnPoint = childEntity->FindComponent<ROS2SpawnPointEditorComponent>();

            if (editorSpawnPoint != nullptr)
            {
                result.insert({ GetEntity()->GetName() + " - " + editorSpawnPoint->GetInfo().first, editorSpawnPoint->GetInfo().second });
            }
        }

        return result;
    }

    const AZ::Transform& ROS2SpawnerEditorComponent::GetDefaultSpawnPose() const
    {
        return m_controller.GetDefaultSpawnPose();
    }

    AZStd::unordered_map<AZStd::string, SpawnPointInfo> ROS2SpawnerEditorComponent::GetAllSpawnPointInfos() const
    {
        return GetSpawnPoints();
    }

    bool ROS2SpawnerEditorComponent::ShouldActivateController() const
    {
        return false;
    }

    void ROS2SpawnerEditorComponent::Activate()
    {
        ROS2SpawnerEditorComponentBase::Activate();
        ROS2SpawnerComponentConfig config = m_controller.GetConfiguration();
        config.m_editorEntityId = GetEntityId();
        AZ_Assert(config.m_editorEntityId.IsValid(), "Spawner component got an invalid entity id");
        m_controller.SetConfiguration(config);
        SpawnerRequestsBus::Handler::BusConnect(config.m_editorEntityId);
    }

    void ROS2SpawnerEditorComponent::Deactivate()
    {
        SpawnerRequestsBus::Handler::BusDisconnect();
        ROS2SpawnerEditorComponentBase::Deactivate();
    }

} // namespace ROS2
