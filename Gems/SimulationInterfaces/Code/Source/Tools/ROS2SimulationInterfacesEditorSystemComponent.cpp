/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SimulationInterfacesEditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/ROS2SimulationInterfacesTypeIds.h>

namespace ROS2SimulationInterfaces
{
    AZ_COMPONENT_IMPL(
        ROS2SimulationInterfacesEditorSystemComponent,
        "ROS2SimulationInterfacesEditorSystemComponent",
        ROS2SimulationInterfacesEditorSystemComponentTypeId,
        BaseSystemComponent);

    void ROS2SimulationInterfacesEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SimulationInterfacesEditorSystemComponent, ROS2SimulationInterfacesSystemComponent>()->Version(0);
        }
    }

    ROS2SimulationInterfacesEditorSystemComponent::ROS2SimulationInterfacesEditorSystemComponent()
        : m_nodeHandler(
              [this](std::shared_ptr<rclcpp::Node> node)
              {
                  if (!m_systemComponentActivated)
                  {
                      ROS2SimulationInterfacesSystemComponent::Activate();
                      m_systemComponentActivated = true;
                  }
                  else
                  {
                      ROS2SimulationInterfacesSystemComponent::Deactivate();
                      m_systemComponentActivated = false;
                  }
              })
    {
    }

    ROS2SimulationInterfacesEditorSystemComponent::~ROS2SimulationInterfacesEditorSystemComponent() = default;

    void ROS2SimulationInterfacesEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("ROS2SimulationInterfacesEditorService"));
    }

    void ROS2SimulationInterfacesEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("ROS2SimulationInterfacesEditorService"));
    }

    void ROS2SimulationInterfacesEditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("ROS2EditorService"));
    }

    void ROS2SimulationInterfacesEditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void ROS2SimulationInterfacesEditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        ROS2::ROS2Interface::Get()->ConnectOnNodeChanged(m_nodeHandler);
    }

    void ROS2SimulationInterfacesEditorSystemComponent::Deactivate()
    {
        m_nodeHandler.Disconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

} // namespace ROS2SimulationInterfaces
