/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointUtilities.h"
#include "AzCore/Name/Name.h"
#include "ROS2/Frame/ROS2FrameBus.h"
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
            bool hasFrameComponent = false;
            ROS2FrameComponentBus::EventResult(hasFrameComponent, entity->GetId(), &ROS2FrameComponentBus::Events::IsFrame);
            if (hasFrameComponent)
            {
                AZ::Name jointName;
                ROS2FrameComponentBus::EventResult(jointName, entity->GetId(), &ROS2FrameComponentBus::Events::GetJointName);
                return jointName.GetStringView();
            }
        }
        return AZStd::string();
    }
} // namespace ROS2::Utils