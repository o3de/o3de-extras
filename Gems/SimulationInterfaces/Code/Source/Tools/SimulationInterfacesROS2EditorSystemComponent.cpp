/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2EditorSystemComponent.h"
#include <AzCore/Serialization/SerializeContext.h>

#include <SimulationInterfaces/SimulationInterfacesROS2TypeIds.h>

namespace SimulationInterfacesROS2
{
    AZ_COMPONENT_IMPL(
        SimulationInterfacesROS2EditorSystemComponent,
        "SimulationInterfacesROS2EditorSystemComponent",
        SimulationInterfacesROS2EditorSystemComponentTypeId,
        BaseSystemComponent);

    void SimulationInterfacesROS2EditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationInterfacesROS2EditorSystemComponent, SimulationInterfacesROS2SystemComponent>()->Version(0);
        }
    }

    SimulationInterfacesROS2EditorSystemComponent::SimulationInterfacesROS2EditorSystemComponent()
        : m_nodeHandler(
              [this](std::shared_ptr<rclcpp::Node> node)
              {
                  if (!m_systemComponentActivated)
                  {
                      SimulationInterfacesROS2SystemComponent::Activate();
                      m_systemComponentActivated = true;
                  }
                  else
                  {
                      SimulationInterfacesROS2SystemComponent::Deactivate();
                      m_systemComponentActivated = false;
                  }
              })
    {
    }

    SimulationInterfacesROS2EditorSystemComponent::~SimulationInterfacesROS2EditorSystemComponent() = default;

    void SimulationInterfacesROS2EditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("SimulationInterfacesROS2EditorService"));
    }

    void SimulationInterfacesROS2EditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesROS2EditorService"));
    }

    void SimulationInterfacesROS2EditorSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
        required.push_back(AZ_CRC_CE("ROS2EditorService"));
    }

    void SimulationInterfacesROS2EditorSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void SimulationInterfacesROS2EditorSystemComponent::Activate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
        ROS2::ROS2Interface::Get()->ConnectOnNodeChanged(m_nodeHandler);
    }

    void SimulationInterfacesROS2EditorSystemComponent::Deactivate()
    {
        m_nodeHandler.Disconnect();
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
    }

} // namespace SimulationInterfacesROS2
