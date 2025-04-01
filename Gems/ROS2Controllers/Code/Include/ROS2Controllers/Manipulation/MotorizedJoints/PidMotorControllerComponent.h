/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <ROS2/Manipulation/MotorizedJoints/JointMotorControllerComponent.h>
#include <ROS2/Manipulation/MotorizedJoints/PidMotorControllerBus.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>

namespace ROS2
{
    class PidMotorControllerComponent
        : public JointMotorControllerComponent
        , public PidMotorControllerRequestBus::Handler
    {
    public:
        AZ_COMPONENT(PidMotorControllerComponent, "{ac1d057f-a6ad-4a26-b44f-0ebda2f5f526}", JointMotorControllerComponent);

        PidMotorControllerComponent() = default;
        ~PidMotorControllerComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        void Activate() override;
        void Deactivate() override;

        // PidMotorControllerBus overrides
        void SetSetpoint(float setpoint) override;
        float GetSetpoint() override;
        float GetCurrentMeasurement() override;
        float GetError() override;

    private:
        Controllers::PidConfiguration m_pidPos; //!< PID controller for position.
        float m_zeroOffset{ 0.0f }; //!< Offset added to setpoint.
        float m_setPoint{ 0.0f }; //!< Desired local position.
        float m_error{ 0.0f }; //!< Current error (difference between control value and measurement).

        // JointMotorControllerComponent overrides
        float CalculateMotorSpeed([[maybe_unused]] float deltaTime) override;
        void DisplayControllerParameters() override;
    };
} // namespace ROS2
