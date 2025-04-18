/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseComponent.h"
#include "SimulationInterfaces/NamedPose.h"
#include "SimulationInterfaces/NamedPoseManagerRequestBus.h"

namespace SimulationInterfaces
{

    NamedPoseComponent::NamedPoseComponent(NamedPose config)
        : m_config(config)
    {
    }

    void NamedPoseComponent::Activate()
    {
        NamedPoseComponentRequestBus::Handler::BusConnect(GetEntityId());
        NamedPoseManagerRequestBus::Broadcast(&NamedPoseManagerRequests::RegisterNamedPose, GetEntityId());
    }

    void NamedPoseComponent::Deactivate()
    {
        NamedPoseManagerRequestBus::Broadcast(&NamedPoseManagerRequests::UnregisterNamedPose, GetEntityId());
        NamedPoseComponentRequestBus::Handler::BusDisconnect();
    }

    void NamedPoseComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TagService")); // tag component
    }

    void NamedPoseComponent::Reflect(AZ::ReflectContext* context)
    {
        if (!context->IsTypeReflected(azrtti_typeid<NamedPose>()))
        {
            NamedPose::Reflect(context);
        }
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NamedPoseComponent, AZ::Component>()->Version(0)->Field("NamedPoseConfig", &NamedPoseComponent::m_config);
        }
    }
} // namespace SimulationInterfaces
