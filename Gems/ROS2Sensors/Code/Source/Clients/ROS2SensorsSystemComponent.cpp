/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SensorsSystemComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <Lidar/LidarCore.h>

namespace ROS2Sensors
{
    AZ_COMPONENT_IMPL(ROS2SensorsSystemComponent, "ROS2SensorsSystemComponent", ROS2SensorsSystemComponentTypeId);

    void ROS2SensorsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect structs not strictly owned by any single component
        LidarCore::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SensorsSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2SensorsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2SensorsService"));
    }

    void ROS2SensorsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2SensorsService"));
    }

    void ROS2SensorsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2SensorsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

} // namespace ROS2Sensors
