/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SpawnPointComponent.h"
#include "Spawner/ROS2SpawnPointComponentController.h"
#include <AzCore/Component/Entity.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    ROS2SpawnPointComponent::ROS2SpawnPointComponent(const ROS2SpawnPointComponentConfig& config)
        : ROS2SpawnPointComponentBase(config)
    {
    }

    void ROS2SpawnPointComponent::Activate()
    {
        ROS2SpawnPointComponentBase::Activate();
    }

    void ROS2SpawnPointComponent::Deactivate()
    {
        ROS2SpawnPointComponentBase::Deactivate();
    }

    void ROS2SpawnPointComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2SpawnPointComponentBase::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SpawnPointComponent, ROS2SpawnPointComponentBase>()->Version(1);
        }
    }

    AZStd::pair<AZStd::string, SpawnPointInfo> ROS2SpawnPointComponent::GetInfo() const
    {
        return m_controller.GetInfo();
    }

} // namespace ROS2
