/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2Controllers/Controllers/PidConfiguration.h>
#include <ROS2Controllers/ROS2ControllersTypeIds.h>
#include <ROS2Controllers/RobotControl/ControlConfiguration.h>
#include <ROS2Controllers/VehicleDynamics/VehicleConfiguration.h>

#include <AzCore/Component/Component.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2Controllers
{
    //! Interface for creating ROS2 sensor components.
    //! This interface provides methods to create various types of ROS2 sensor components, such as GNSS, Camera, IMU, Lidar, and Odometry
    //! sensors. It is intended to be used by external tools that create entities, e.g., when importing robots from SDF, URDF, or OpenUSD.
    class ROS2ControllersEditorRequests
    {
    public:
        AZ_RTTI(ROS2ControllersEditorRequests, ROS2ControllersEditorRequestsTypeId);
        virtual ~ROS2ControllersEditorRequests() = default;

        //! Create a new Wheel Controller component.
        //! @param entity The entity to which the wheel controller component will be added.
        //! @param steeringEntity The entity that will be used for steering control (WheelControllerComponent parameter).
        //! @param steeringScale The scale factor for the steering control (WheelControllerComponent parameter).
        //! @return A pointer to the newly created AZ::Component representing the Wheel Controller (or nullptr if failed).
        virtual AZ::Component* CreateWheelControllerComponent(
            AZ::Entity& entity, const AZ::EntityId& steeringEntity, const float steeringScale) = 0;

        //! Create a new ROS2 Robot Control component.
        //! @param entity The entity to which the robot control component will be added.
        //! @param configuration The configuration for the robot control.
        //! @return A pointer to the newly created AZ::Component representing the ROS2 Robot Control (or nullptr if failed).
        virtual AZ::Component* CreateROS2RobotControlComponent(AZ::Entity& entity, const ControlConfiguration& configuration) = 0;

        //! Create a new Ackermann Vehicle Model component.
        //! @param entity The entity to which the vehicle model component will be added.
        //! @param configuration The configuration for the Ackermann vehicle model.
        //! @param speedLimit The maximum speed limit for the vehicle model.
        //! @param steeringLimit The maximum steering limit for the vehicle model.
        //! @param acceleration The maximum acceleration for the vehicle model.
        //! @param steeringPid The PID configuration for the steering control.
        //! @return A pointer to the newly created AZ::Component representing the Ackermann Vehicle Model (or nullptr if failed).
        virtual AZ::Component* CreateAckermannVehicleModelComponent(
            AZ::Entity& entity,
            const VehicleDynamics::VehicleConfiguration& configuration,
            const float speedLimit,
            const float steeringLimit,
            const float acceleration,
            const PidConfiguration& steeringPid) = 0;

        //! Create a new Ackermann Control component.
        //! @param entity The entity to which the control component will be added.
        //! @return A pointer to the newly created AZ::Component representing the Ackermann Control (or nullptr if failed).
        virtual AZ::Component* CreateAckermannControlComponent(AZ::Entity& entity) = 0;

        //! Create a new Skid Steering Model component.
        //! @param entity The entity to which the skid steering model component will be added.
        //! @param configuration The configuration for the skid steering vehicle model.
        //! @param linearLimit The maximum linear speed limit for the skid steering model [m/s].
        //! @param angularLimit The maximum angular speed limit for the skid steering model [Rad/s].
        //! @param linearAcceleration The maximum linear acceleration for the skid steering model [m*s^(-2)].
        //! @param angularAcceleration The maximum angular acceleration for the skid steering model [rad*s^(-2)].
        //! @return A pointer to the newly created AZ::Component representing the Skid Steering Model (or nullptr if failed).
        virtual AZ::Component* CreateSkidSteeringModelComponent(
            AZ::Entity& entity,
            const VehicleDynamics::VehicleConfiguration& configuration,
            const float linearLimit,
            const float angularLimit,
            const float linearAcceleration,
            const float angularAcceleration) = 0;

        //! Create a new Skid Steering Control component.
        //! @param entity The entity to which the control component will be added.
        //! @return A pointer to the newly created AZ::Component representing the Skid Steering Control (or nullptr if failed).
        virtual AZ::Component* CreateSkidSteeringControlComponent(AZ::Entity& entity) = 0;
    };

    class ROS2ControllersEditorBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2ControllersEditorRequestBus = AZ::EBus<ROS2ControllersEditorRequests, ROS2ControllersEditorBusTraits>;
    using ROS2ControllersEditorInterface = AZ::Interface<ROS2ControllersEditorRequests>;

} // namespace ROS2Controllers
