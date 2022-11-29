/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/VehicleDynamics/DriveModels/PidConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector2.h>

namespace ROS2
{
    //! A prototype component for simulated joint with a motor.
    //! It works with either TransformBus or RigidBodyBus.
    //! TransformBus mode, called `AnimationMode` changes local transform. In this mode, you cannot have a rigid body
    //! controller enabled. With RigidBodyBus it applies forces and torque according to PID control.
    //! @note This class is already used through ROS2FrameComponent.
    // TODO This is a prototype. Tasks: refactor, add bus interface, rotation, ramps, cascading controllers, tests.
    class MotorizedJointComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(MotorizedJointComponent, "{AE9207DB-5B7E-4F70-A7DD-C4EAD8DD9403}", AZ::Component);

        MotorizedJointComponent() = default;
        ~MotorizedJointComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        //! Set a setpoint (e.g. desired local position). The controller will follow it.
        void SetSetpoint(float setpoint);

        //! Get current control error. It is the difference between control value and measurement.
        //! When the setpoint is reached this should be close to zero.
        //! @returns control error, in meters for linear joints and in radians for angular joints.
        float GetError() const;

        //! Get current position from measurement.
        //! @returns current position, in meters for linear joints and radians for angular joints.
        float GetCurrentPosition() const;

        //! Get a degree of freedom direction.
        //! @returns direction of joint movement in global coordinates.
        AZ::Vector3 GetDir() const
        {
            return m_jointDir;
        };

    private:
        float ComputeMeasurement(AZ::ScriptTimePoint time);
        void SetVelocity(float velocity, float deltaTime);
        void ApplyLinVelAnimation(float velocity, float deltaTime);
        void ApplyLinVelRigidBodyImpulse(float velocity, float deltaTime);
        void ApplyLinVelRigidBody(float velocity, float deltaTime);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        AZ::Vector3 m_jointDir{ 0.f, 0.f, 1.f }; //!< Direction of joint movement in parent frame of reference, used to compute measurement.
        AZ::Vector3 m_effortAxis{ 0.f, 0.f, 1.f }; //!< Direction of force or torque application in owning entity frame of reference.
        AZStd::pair<float, float> m_limits{ -0.5f, 0.5f }; //!< limits of joint, the force is applied only when joint is within limits.
        VehicleDynamics::PidConfiguration m_pidPos; //!< PID controller for position.

        bool m_linear{ true }; //!< Linear mode. The force is applied through RigidBodyBus.
        bool m_animationMode{ true }; //!< Use TransformBus (animation mode, no physics) instead of RigidBodyBus.

        // TODO - remove test signal?
        bool m_testSinusoidal{ true }; //!< Enable sinusoidal signal generator to setpoint (for tuning).
        float m_sinAmplitude{ 0.5 }; //!< Amplitude of test signal generator.
        float m_sinDC{ 0.25 }; //!< DC of test signal generator.
        float m_sinFreq{ 0.1 }; //!< Frequency of test signal generator.

        float m_zeroOffset{ 0.f }; //!< offset added to setpoint.
        float m_setpoint{ 0 }; //!< Desired local position.
        float m_error{ 0 }; //!< Current error (difference between control value and measurement).
        float m_currentPosition{ 0 }; //!< Last measured position.
        float m_currentVelocity{ 0 }; //!< Last measured velocity.
        double m_lastMeasurementTime; //!< Last measurement time in seconds.

        // TODO - remove/replace with proper API use (EntityDebugDisplayEventBus)
        AZ::EntityId m_debugDrawEntity; //!< Optional Entity that allows to visualize desired setpoint value.
        AZ::Transform m_debugDrawEntityInitialTransform; //!< Initial transform of m_debugDrawEntity.
        bool m_debugPrint{ false }; //!< Print debug info to the console.

        AZ::EntityId m_measurementReferenceEntity; //!< Entity used for reference for measurements. Defaults to parent entity.
    };
} // namespace ROS2
