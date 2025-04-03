/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationInterfacesROS2SystemComponent.h"
#include "SimulationInterfacesROS2/SimulationInterfacesROS2RequestBus.h"
#include "Utils/ServicesConfig.h"

#include <ROS2/ROS2Bus.h>
#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

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
        SimulationInterfacesROS2RequestBus::Handler::BusConnect();

        rclcpp::Node::SharedPtr ros2Node = rclcpp::Node::SharedPtr(ROS2::ROS2Interface::Get()->GetNode());
        AZ_Assert(ros2Node, "ROS2 node is not available.");
        // add all known/implemented interfaces
        m_availableRos2Interface[DeleteEntityService] =
            AZStd::make_shared<DeleteEntityServiceHandler>(ros2Node, DeleteEntityServiceDefaultName);

        m_availableRos2Interface[GetEntitiesService] =
            AZStd::make_shared<GetEntitiesServiceHandler>(ros2Node, GetEntitiesServiceDefaultName);

        m_availableRos2Interface[GetEntitiesStatesService] =
            AZStd::make_shared<GetEntitiesStatesServiceHandler>(ros2Node, GetEntitiesStatesServiceDefaultName);

        m_availableRos2Interface[GetEntityStateService] =
            AZStd::make_shared<GetEntityStateServiceHandler>(ros2Node, GetEntityStateServiceDefaultName);

        m_availableRos2Interface[GetSpawnablesService] =
            AZStd::make_shared<GetSpawnablesServiceHandler>(ros2Node, GetSpawnablesServiceDefaultName);

        m_availableRos2Interface[SetEntityStateService] =
            AZStd::make_shared<SetEntityStateServiceHandler>(ros2Node, SetEntityStateServiceDefaultName);

        m_availableRos2Interface[SpawnEntityService] =
            AZStd::make_shared<SpawnEntityServiceHandler>(ros2Node, SpawnEntityServiceDefaultName);

        m_availableRos2Interface[GetSimulationFeaturesService] =
            AZStd::make_shared<GetSimulationFeaturesServiceHandler>(ros2Node, GetSimulationFeaturesServiceDefaultName);
    }

    void SimulationInterfacesROS2SystemComponent::Deactivate()
    {
        SimulationInterfacesROS2RequestBus::Handler::BusDisconnect();

        for (auto& [serviceType, service] : m_availableRos2Interface)
        {
            service.reset();
        }
    }

    AZStd::unordered_set<AZ::u8> SimulationInterfacesROS2SystemComponent::GetSimulationFeatures()
    {
        AZStd::unordered_set<AZ::u8> result;
        for (auto& [serviceType, serviceHandler] : m_availableRos2Interface)
        {
            auto features = serviceHandler->GetProvidedFeatures();
            result.insert(features.begin(), features.end());
        }
        return result;
    }

} // namespace SimulationInterfacesROS2
