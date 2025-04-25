/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/API/ApplicationAPI.h>

#include <SimulationInterfaces/ROS2SimulationInterfacesRequestBus.h>

#include <Interfaces/IROS2HandlerBase.h>

namespace ROS2SimulationInterfaces
{
    class ROS2SimulationInterfacesSystemComponent
        : public AZ::Component
        , public ROS2SimulationInterfacesRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(ROS2SimulationInterfacesSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2SimulationInterfacesSystemComponent() = default;
        ~ROS2SimulationInterfacesSystemComponent() = default;

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        // ROS2SimulationInterfacesRequestBus override
        AZStd::unordered_set<SimulationFeatureType> GetSimulationFeatures() override;

    private:
        AZStd::unordered_map<AZStd::string, AZStd::shared_ptr<IROS2HandlerBase>> m_availableRos2Interface;

        template<typename T>
        void RegisterInterface(rclcpp::Node::SharedPtr ros2Node)
        {
            AZStd::shared_ptr handler = AZStd::make_shared<T>();
            handler->Initialize(ros2Node);
            m_availableRos2Interface[handler->GetTypeName()] = AZStd::move(handler);
            handler.reset();
        };
    };

} // namespace ROS2SimulationInterfaces
