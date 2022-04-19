/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ROS2/ROS2Bus.h"
#include "Transform/ROS2FrameComponent.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    void ROS2FrameComponent::Activate()
    {
        m_transformPublisher = AZStd::make_unique<TransformPublisher>(GetParentFrameID(), GetFrameID(), GetFrameTransform());
    }

    void ROS2FrameComponent::Deactivate()
    {
        m_transformPublisher.reset();
    }

    const ROS2FrameComponent* ROS2FrameComponent::GetParentROS2FrameComponent() const
    {
        if (const AZ::EntityId parentEntityId = GetEntityTransformInterface()->GetParentId(); parentEntityId.IsValid())
        {
            // TODO - is there really no better way???
            AZ::Entity* parentEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);
            return parentEntity->FindComponent<ROS2FrameComponent>();
        }
        return nullptr;
    }

    const AZ::Transform& ROS2FrameComponent::GetFrameTransform() const
    {
        auto transformInterface = GetEntityTransformInterface();
        if (GetParentROS2FrameComponent() != nullptr)
        {
            return transformInterface->GetLocalTM();
        }
        return transformInterface->GetWorldTM();
    }

    AZ::TransformInterface* ROS2FrameComponent::GetEntityTransformInterface() const
    {
        return GetEntity()->FindComponent<AzFramework::TransformComponent>();
    }

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        if (auto parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
        return "world"; // TODO - parametrize this, sometimes it is "world" and sometimes "map"
    }

    AZStd::string ROS2FrameComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(m_namespace, m_frameName);
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        return m_namespace;
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()
                ->Version(1)
                ->Field("Namespace", &ROS2FrameComponent::m_namespace)
                ->Field("Frame Name", &ROS2FrameComponent::m_frameName)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameComponent>("ROS2 Frame Component", "[ROS2 Frame component]")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                            ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_namespace, "Namespace", "Namespace")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_frameName, "Frame Name", "Frame Name")
                        ;
            }
        }
    }

    void ROS2FrameComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
    }
} // namespace ROS2
