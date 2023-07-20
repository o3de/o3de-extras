/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnPointComponent.h"
#include <AzCore/Component/Entity.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    void ROS2SpawnPointComponent::Activate()
    {
    }

    void ROS2SpawnPointComponent::Deactivate()
    {
    }

    void ROS2SpawnPointComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnPointComponent, AZ::Component>()
                ->Version(1)
                ->Field("Name", &ROS2SpawnPointComponent::m_name)
                ->Field("Info", &ROS2SpawnPointComponent::m_info);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SpawnPointComponent>("ROS2 Spawn Point", "Spawn Point")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Stores information about available spawn point")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &ROS2SpawnPointComponent::m_name, "Name", "Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId, &ROS2SpawnPointComponent::m_info, "Info", "Spawn point detailed description");
            }
        }
    }

    AZStd::pair<AZStd::string, SpawnPointInfo> ROS2SpawnPointComponent::GetInfo() const
    {
        auto transform_component = GetEntity()->FindComponent<AzFramework::TransformComponent>();

        // if SpawnPointComponent entity for some reason does not include TransformComponent - this default pose will be returned
        AZ::Transform transform = AZ::Transform::CreateIdentity();

        if (transform_component != nullptr)
        {
            transform = transform_component->GetWorldTM();
        }
        return { m_name, SpawnPointInfo{ m_info, transform } };
    }
} // namespace ROS2
