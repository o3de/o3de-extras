/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Spawner/ROS2SpawnPointComponentController.h"
#include "Spawner/ROS2SpawnerComponentController.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2SpawnPointComponentConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnPointComponentConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("Name", &ROS2SpawnPointComponentConfig::m_name)
                ->Field("Info", &ROS2SpawnPointComponentConfig::m_info);

            if (auto editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<ROS2SpawnPointComponentConfig>("ROS2SpawnPointComponentConfig", "Config for the ROS2 Spawn Point Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Stores information about available spawn point")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnPointComponentConfig::m_name, "Name", "Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ROS2SpawnPointComponentConfig::m_info, "Info", "Spawn point detailed description");
            }
        }
    }

    void ROS2SpawnPointComponentController::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnPointComponentConfig::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SpawnPointComponentController>()->Version(1)->Field(
                "Configuration", &ROS2SpawnPointComponentController::m_config);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext
                    ->Class<ROS2SpawnPointComponentController>("ROS2SpawnPointController", "Controller for the ROS2 Spawn Point Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2SpawnPointComponentController::m_config)
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    ROS2SpawnPointComponentController::ROS2SpawnPointComponentController(const ROS2SpawnPointComponentConfig& config)
    {
        SetConfiguration(config);
    }

    void ROS2SpawnPointComponentController::SetConfiguration(const ROS2SpawnPointComponentConfig& config)
    {
        m_config = config;
    }

    const ROS2SpawnPointComponentConfig& ROS2SpawnPointComponentController::GetConfiguration() const
    {
        return m_config;
    }

    void ROS2SpawnPointComponentController::Activate(AZ::EntityId entityId)
    {
        m_config.m_editorEntityId = entityId;
    }

    void ROS2SpawnPointComponentController::Deactivate()
    {
    }

    AZStd::pair<AZStd::string, SpawnPointInfo> ROS2SpawnPointComponentController::GetInfo() const
    {
        AZ::Transform transform = { AZ::Vector3{ 0, 0, 0 }, AZ::Quaternion{ 0, 0, 0, 1.0 }, 1.0 };
        AZ::TransformBus::EventResult(transform, m_config.m_editorEntityId, &AZ::TransformBus::Events::GetWorldTM);

        return { m_config.m_name, SpawnPointInfo{ m_config.m_info, transform } };
    }

} // namespace ROS2
