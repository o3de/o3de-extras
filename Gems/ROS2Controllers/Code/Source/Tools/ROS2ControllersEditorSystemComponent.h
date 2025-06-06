/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/ROS2ControllersSystemComponent.h>
#include <ROS2Controllers/ROS2ControllersEditorBus.h>

namespace ROS2Controllers
{
    /// System component for ROS2Controllers editor
    class ROS2ControllersEditorSystemComponent
        : public ROS2ControllersSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
        , protected ROS2ControllersEditorRequestBus::Handler
    {
        using BaseSystemComponent = ROS2ControllersSystemComponent;

    public:
        AZ_COMPONENT_DECL(ROS2ControllersEditorSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        ROS2ControllersEditorSystemComponent();
        ~ROS2ControllersEditorSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ROS2ControllersEditorRequestBus interface implementation
        AZ::Component* CreateWheelControllerComponent(
            AZ::Entity& entity, const AZ::EntityId& steeringEntity, const float steeringScale) override;
        AZ::Component* CreateROS2RobotControlComponent(AZ::Entity& entity, const ControlConfiguration& configuration) override;
        AZ::Component* CreateAckermannVehicleModelComponent(
            AZ::Entity& entity,
            const VehicleDynamics::VehicleConfiguration& configuration,
            const float speedLimit,
            const float steeringLimit,
            const float acceleration,
            const PidConfiguration& steeringPid) override;
        AZ::Component* CreateAckermannControlComponent(AZ::Entity& entity) override;
        ////////////////////////////////////////////////////////////////////////

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2Controllers
