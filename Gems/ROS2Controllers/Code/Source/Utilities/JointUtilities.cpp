/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointUtilities.h"
#include <ROS2/Frame/ROS2FrameComponent.h>

namespace ROS2::Utils
{
    AZStd::string GetJointName(AZ::EntityId entityId)
    {
        AZ::Entity* entity{ nullptr };
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        AZ_Assert(entity, "Entity %s not found.", entityId.ToString().c_str());
        if (entity)
        {
            ROS2FrameComponent* component = entity->FindComponent<ROS2FrameComponent>();
            if (component)
            {
                return component->GetJointName().GetStringView();
            }
        }
        return AZStd::string();
    }
} // namespace ROS2::Utils