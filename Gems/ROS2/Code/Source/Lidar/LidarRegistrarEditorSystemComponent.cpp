/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LidarRegistrarEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

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
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("LidarRegistrarEditorService"));
    }

    void LidarRegistrarEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("LidarRegistrarEditorService"));
    }

    void LidarRegistrarEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void LidarRegistrarEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
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
