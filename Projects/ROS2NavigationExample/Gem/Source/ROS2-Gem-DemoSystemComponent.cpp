/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "ROS2-Gem-DemoSystemComponent.h"

namespace RobotVacuumSample
{
    void RobotVacuumSampleSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<RobotVacuumSampleSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<RobotVacuumSampleSystemComponent>("RobotVacuumSample", "The base Robot Vacuum Sample component.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void RobotVacuumSampleSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("RobotVacuumSampleService"));
    }

    void RobotVacuumSampleSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("RobotVacuumSampleService"));
    }

    void RobotVacuumSampleSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RobotVacuumSampleSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RobotVacuumSampleSystemComponent::RobotVacuumSampleSystemComponent()
    {
        if (RobotVacuumSampleInterface::Get() == nullptr)
        {
            RobotVacuumSampleInterface::Register(this);
        }
    }

    RobotVacuumSampleSystemComponent::~RobotVacuumSampleSystemComponent()
    {
        if (RobotVacuumSampleInterface::Get() == this)
        {
            RobotVacuumSampleInterface::Unregister(this);
        }
    }

    void RobotVacuumSampleSystemComponent::Init()
    {
    }

    void RobotVacuumSampleSystemComponent::Activate()
    {
        RobotVacuumSampleRequestBus::Handler::BusConnect();
    }

    void RobotVacuumSampleSystemComponent::Deactivate()
    {
        RobotVacuumSampleRequestBus::Handler::BusDisconnect();
    }
}
