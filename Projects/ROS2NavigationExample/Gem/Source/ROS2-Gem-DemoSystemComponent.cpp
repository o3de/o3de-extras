// {BEGIN_LICENSE}
/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
 // {END_LICENSE}

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "ROS2-Gem-DemoSystemComponent.h"

namespace ROS2_Gem_Demo
{
    void ROS2_Gem_DemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2_Gem_DemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2_Gem_DemoSystemComponent>("ROS2_Gem_Demo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void ROS2_Gem_DemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("ROS2_Gem_DemoService"));
    }

    void ROS2_Gem_DemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("ROS2_Gem_DemoService"));
    }

    void ROS2_Gem_DemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2_Gem_DemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2_Gem_DemoSystemComponent::ROS2_Gem_DemoSystemComponent()
    {
        if (ROS2_Gem_DemoInterface::Get() == nullptr)
        {
            ROS2_Gem_DemoInterface::Register(this);
        }
    }

    ROS2_Gem_DemoSystemComponent::~ROS2_Gem_DemoSystemComponent()
    {
        if (ROS2_Gem_DemoInterface::Get() == this)
        {
            ROS2_Gem_DemoInterface::Unregister(this);
        }
    }

    void ROS2_Gem_DemoSystemComponent::Init()
    {
    }

    void ROS2_Gem_DemoSystemComponent::Activate()
    {
        ROS2_Gem_DemoRequestBus::Handler::BusConnect();
    }

    void ROS2_Gem_DemoSystemComponent::Deactivate()
    {
        ROS2_Gem_DemoRequestBus::Handler::BusDisconnect();
    }
}
