/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2SystemComponent.h"

#include <ROS2/ROS2Bus.h>
#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace SimulationInterfacesROS2
{
    AZ_COMPONENT_IMPL(
        SimulationInterfacesROS2SystemComponent, "SimulationInterfacesROS2SystemComponent", SimulationInterfacesROS2SystemComponentTypeId);

    void SimulationInterfacesROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationInterfacesROS2SystemComponent, AZ::Component>()->Version(0);
        }
    }

    void SimulationInterfacesROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationInterfacesROS2Service"));
    }

    void SimulationInterfacesROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationInterfacesROS2Service"));
    }

    void SimulationInterfacesROS2SystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SimulationInterfacesROS2SystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void SimulationInterfacesROS2SystemComponent::Init()
    {
    }

    void SimulationInterfacesROS2SystemComponent::Activate()
    {
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusConnect();
    }

    void SimulationInterfacesROS2SystemComponent::Deactivate()
    {
        AzFramework::LevelSystemLifecycleNotificationBus::Handler::BusDisconnect();
        OnUnloadComplete(nullptr);
    }

    void SimulationInterfacesROS2SystemComponent::OnLoadingComplete([[maybe_unused]] const char* levelName)
    {
        rclcpp::Node::SharedPtr ros2Node = rclcpp::Node::SharedPtr(ROS2::ROS2Interface::Get()->GetNode());
        AZ_Assert(ros2Node, "ROS2 node is not available.");

        m_deleteEntityServiceHandler.emplace(ros2Node, DeleteEntityServiceName);
        m_getEntitiesServiceHandler.emplace(ros2Node, GetEntitiesServiceName);
        m_getSpawnablesServiceHandler.emplace(ros2Node, GetSpawnablesServiceName);
        m_spawnEntityServiceHandler.emplace(ros2Node, SpawnEntityServiceName);
        m_getEntitiesStatesServiceHandler.emplace(ros2Node, GetEntitiesStatesServiceName);
        m_getEntityStateServiceHandler.emplace(ros2Node, GetEntityStateServiceName);
        m_setEntityStateServiceHandler.emplace(ros2Node, SetEntityStateServiceName);
    }

    void SimulationInterfacesROS2SystemComponent::OnUnloadComplete([[maybe_unused]] const char* levelName)
    {
        m_deleteEntityServiceHandler.reset();
        m_getEntitiesServiceHandler.reset();
        m_getSpawnablesServiceHandler.reset();
        m_spawnEntityServiceHandler.reset();
        m_getEntitiesStatesServiceHandler.reset();
        m_getEntityStateServiceHandler.reset();
        m_setEntityStateServiceHandler.reset();
    }

} // namespace SimulationInterfacesROS2
