/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseComponent.h"
#include "SimulationInterfaces/NamedPose.h"

namespace SimulationInterfaces
{

    void NamedPoseComponent::Activate()
    {
    }

    void NamedPoseComponent::Deactivate()
    {
    }

    void NamedPoseComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TagService")); // tag component
    }

    void NamedPoseComponent::Reflect(AZ::ReflectContext* context)
    {
        NamedPose::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NamedPoseComponent, AZ::Component>()->Version(0)->Field("NamedPoseConfig", &NamedPoseComponent::m_config);
            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<NamedPoseComponent>("Named Pose Component", "Component used to define names pose in simulation")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "NamedPoseComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "Simulation Interfaces")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamedPoseComponent::m_config, "Named Pose Config", "");
            }
        }
    }
} // namespace SimulationInterfaces
