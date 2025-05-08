/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ControllersSystemComponent.h"

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Controllers/Controllers/PidConfiguration.h>
#include <VehicleDynamics/VehicleModelComponent.h>

namespace ROS2Controllers
{
    AZ_COMPONENT_IMPL(ROS2ControllersSystemComponent, "ROS2ControllersSystemComponent", ROS2ControllersSystemComponentTypeId);

    void ROS2ControllersSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect structs not strictly owned by any single component
        PidConfiguration::Reflect(context);
        VehicleDynamics::VehicleModelComponent::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ControllersSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2ControllersSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ControllersService"));
    }

    void ROS2ControllersSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ControllersService"));
    }

    void ROS2ControllersSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2ControllersSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }
} // namespace ROS2Controllers
