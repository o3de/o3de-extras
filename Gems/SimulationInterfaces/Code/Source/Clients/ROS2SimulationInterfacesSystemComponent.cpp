/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SimulationInterfacesSystemComponent.h"

#include <Actions/SimulateStepsActionServerHandler.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/API/ApplicationAPI.h>

#include <ROS2/ROS2Bus.h>
#include <SimulationInterfaces/ROS2SimulationInterfacesRequestBus.h>
#include <SimulationInterfaces/ROS2SimulationInterfacesTypeIds.h>

#include <Services/DeleteEntityServiceHandler.h>
#include <Services/GetEntitiesServiceHandler.h>
#include <Services/GetEntitiesStatesServiceHandler.h>
#include <Services/GetEntityStateServiceHandler.h>
#include <Services/GetSimulationFeaturesServiceHandler.h>
#include <Services/GetSimulationStateServiceHandler.h>
#include <Services/GetSpawnablesServiceHandler.h>
#include <Services/ROS2ServiceBase.h>
#include <Services/ResetSimulationServiceHandler.h>
#include <Services/SetEntityStateServiceHandler.h>
#include <Services/SetSimulationStateServiceHandler.h>
#include <Services/SpawnEntityServiceHandler.h>
#include <Services/StepSimulationServiceHandler.h>

namespace ROS2SimulationInterfaces
{

    namespace
    {
        template<typename T>
        void RegisterInterface(
            AZStd::unordered_map<AZStd::string, AZStd::shared_ptr<IROS2HandlerBase>>& interfacesMap, rclcpp::Node::SharedPtr ros2Node)
        {
            AZStd::shared_ptr handler = AZStd::make_shared<T>();
            handler->Initialize(ros2Node);
            interfacesMap[handler->GetTypeName()] = AZStd::move(handler);
            handler.reset();
        };
    } // namespace

    AZ_COMPONENT_IMPL(
        ROS2SimulationInterfacesSystemComponent, "ROS2SimulationInterfacesSystemComponent", ROS2SimulationInterfacesSystemComponentTypeId);

    void ROS2SimulationInterfacesSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SimulationInterfacesSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2SimulationInterfacesSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2SimulationInterfacesService"));
    }

    void ROS2SimulationInterfacesSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2SimulationInterfacesService"));
    }

    void ROS2SimulationInterfacesSystemComponent::GetRequiredServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2SimulationInterfacesSystemComponent::GetDependentServices(
        [[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void ROS2SimulationInterfacesSystemComponent::Activate()
    {
        ROS2SimulationInterfacesRequestBus::Handler::BusConnect();

        rclcpp::Node::SharedPtr ros2Node = rclcpp::Node::SharedPtr(ROS2::ROS2Interface::Get()->GetNode());
        AZ_Assert(ros2Node, "ROS2 node is not available.");

        RegisterInterface<DeleteEntityServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetEntitiesServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetEntitiesStatesServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetEntityStateServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetSpawnablesServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<SetEntityStateServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<SpawnEntityServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetSimulationFeaturesServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<ResetSimulationServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<SimulateStepsActionServerHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<SetSimulationStateServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<GetSimulationStateServiceHandler>(m_availableRos2Interface, ros2Node);
        RegisterInterface<StepSimulationServiceHandler>(m_availableRos2Interface, ros2Node);
    }

    void ROS2SimulationInterfacesSystemComponent::Deactivate()
    {
        ROS2SimulationInterfacesRequestBus::Handler::BusDisconnect();

        for (auto& [handlerType, handler] : m_availableRos2Interface)
        {
            handler.reset();
        }
    }

    AZStd::unordered_set<SimulationFeatureType> ROS2SimulationInterfacesSystemComponent::GetSimulationFeatures()
    {
        AZStd::unordered_set<SimulationFeatureType> result;
        for (auto& [handlerType, handler] : m_availableRos2Interface)
        {
            auto features = handler->GetProvidedFeatures();
            result.insert(features.begin(), features.end());
        }
        return result;
    }

} // namespace ROS2SimulationInterfaces
