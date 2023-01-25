/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include <Lidar/LidarRegistrarEditorSystemComponent.h>

namespace ROS2
{
    void LidarRegistrarEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarRegistrarEditorSystemComponent, LidarRegistrarSystemComponent>()->Version(0);
        }
    }

    void LidarRegistrarEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        LidarRegistrarSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("LidarRegistrarEditorService"));
    }

    void LidarRegistrarEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        LidarRegistrarSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("LidarRegistrarEditorService"));
    }

    void LidarRegistrarEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        LidarRegistrarSystemComponent::GetRequiredServices(required);
    }

    void LidarRegistrarEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        LidarRegistrarSystemComponent::GetDependentServices(dependent);
    }

    void LidarRegistrarEditorSystemComponent::Activate()
    {
        LidarRegistrarSystemComponent::Activate();
    }

    void LidarRegistrarEditorSystemComponent::Deactivate()
    {
        LidarRegistrarSystemComponent::Deactivate();
    }
} // namespace ROS2
