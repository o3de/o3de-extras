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
#include <AzCore/std/string/string.h>

#include <ROS2/ROS2Bus.h>
#include <SimulationInterfacesROS2/SimulationInterfacesROS2TypeIds.h>

#include "Utils/RegistryUtils.h"
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>

namespace SimulationInterfacesROS2
{

    namespace
    {
        template<typename T>
        void RegisterInterface(
            AZStd::unordered_map<AZStd::string, AZStd::shared_ptr<ROS2HandlerBase>>& interfacesMap,
            rclcpp::Node::SharedPtr ros2Node,
            AZStd::string serviceType,
            AZStd::string defaultName)
        {
            // add all known/implemented interfaces
            auto serviceName = RegistryUtilities::GetServiceName(serviceType);
            interfacesMap[serviceType] = AZStd::make_shared<T>(ros2Node, serviceName.empty() ? defaultName : serviceName);
        };
    } // namespace

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
        required.push_back(AZ_CRC_CE("ROS2Service"));
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

        RegisterInterface<DeleteEntityServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(DeleteEntityService), AZStd::string(DeleteEntityServiceDefaultName));

        RegisterInterface<GetEntitiesServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(GetEntitiesService), AZStd::string(GetEntitiesServiceDefaultName));

        RegisterInterface<GetEntitiesStatesServiceHandler>(
            m_availableRos2Interface,
            ros2Node,
            AZStd::string(GetEntitiesStatesService),
            AZStd::string(GetEntitiesStatesServiceDefaultName));

        RegisterInterface<GetEntityStateServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(GetEntityStateService), AZStd::string(GetEntityStateServiceDefaultName));

        RegisterInterface<GetSpawnablesServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(GetSpawnablesService), AZStd::string(GetSpawnablesServiceDefaultName));

        RegisterInterface<SetEntityStateServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(SetEntityStateService), AZStd::string(SetEntityStateServiceDefaultName));

        RegisterInterface<SpawnEntityServiceHandler>(
            m_availableRos2Interface, ros2Node, AZStd::string(SpawnEntityService), AZStd::string(SpawnEntityServiceDefaultName));

        RegisterInterface<GetSimulationFeaturesServiceHandler>(
            m_availableRos2Interface,
            ros2Node,
            AZStd::string(GetSimulationFeaturesService),
            AZStd::string(GetSimulationFeaturesServiceDefaultName));
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
