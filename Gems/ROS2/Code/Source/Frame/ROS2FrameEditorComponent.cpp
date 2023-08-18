/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{

    void ROS2FrameEditorComponent::Activate()
    {
        ROS2FrameEditorComponentBase::Activate();
        m_controller.PopulateNamespace(m_controller.IsTopLevel(), GetEntity()->GetName());
    }

    void ROS2FrameEditorComponent::Deactivate()
    {
        ROS2FrameEditorComponentBase::Deactivate();
    }

    AZStd::string ROS2FrameEditorComponent::GetGlobalFrameName() const
    {
        return m_controller.GetGlobalFrameName();
    }

    bool ROS2FrameEditorComponent::IsTopLevel() const
    {
        return m_controller.IsTopLevel();
    }

    bool ROS2FrameEditorComponent::IsDynamic() const
    {
        return m_controller.IsDynamic();
    }

    AZ::Transform ROS2FrameEditorComponent::GetFrameTransform() const
    {
        return m_controller.GetFrameTransform();
    }

    AZStd::string ROS2FrameEditorComponent::GetParentFrameID() const
    {
        return m_controller.GetParentFrameID();
    }

    AZStd::string ROS2FrameEditorComponent::GetFrameID() const
    {
        return m_controller.GetFrameID();
    }

    void ROS2FrameEditorComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_controller.SetFrameID(frameId);
    }

    AZStd::string ROS2FrameEditorComponent::GetNamespace() const
    {
        return m_controller.GetNamespace();
    }

    AZ::Name ROS2FrameEditorComponent::GetJointName() const
    {
        return m_controller.GetJointName();
    }

    void ROS2FrameEditorComponent::SetJointName(const AZStd::string& jointNameString)
    {
        m_controller.SetJointName(jointNameString);
    }

    void ROS2FrameEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameEditorComponentBase::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameEditorComponent, ROS2FrameEditorComponentBase>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameEditorComponent>("ROS2 Frame", "[ROS2 Frame component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void ROS2FrameEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    ROS2FrameEditorComponent::ROS2FrameEditorComponent() = default;

    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const AZStd::string& frameId)
    {
        m_controller.SetFrameID(frameId);
    }

    ROS2FrameEditorComponent::ROS2FrameEditorComponent(const ROS2FrameConfiguration& config)
    {
        SetConfiguration(config);
    }

    bool ROS2FrameEditorComponent::ShouldActivateController() const
    {
        return true;
    }

} // namespace ROS2
