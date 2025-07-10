/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "NamedPoseComponent.h"

namespace SimulationInterfaces
{

    NamedPoseComponent::NamedPoseComponent(const NamedPose& config)
        : m_config(config)
    {
    }

    void NamedPoseComponent::Activate()
    {
        auto entityId = GetEntityId();
        NamedPoseComponentRequestBus::Handler::BusConnect(entityId);
        NamedPoseManagerRequestBus::Broadcast(&NamedPoseManagerRequests::RegisterNamedPose, entityId);
    }

    void NamedPoseComponent::Deactivate()
    {
        auto entityId = GetEntityId();
        NamedPoseManagerRequestBus::Broadcast(&NamedPoseManagerRequests::UnregisterNamedPose, entityId);
        NamedPoseComponentRequestBus::Handler::BusDisconnect();
    }

    void NamedPoseComponent::Reflect(AZ::ReflectContext* context)
    {
        NamedPose::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<NamedPoseComponent, AZ::Component>()->Version(0)->Field("NamedPoseConfig", &NamedPoseComponent::m_config);
        }
    }
} // namespace SimulationInterfaces
