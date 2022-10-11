/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "VehicleDynamics/DriveModels/PidConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{
    //! Helper that allows us to design a joint with a 'motor'
    //! It can affect the scene with two APIS: TransformBus and RigidBodyBus
    //! TransformBus mode, called `AnimationMode` changes local transform. In this mode, you cannot have a rigid body
    //! controller enabled. It is intended to use in a simple scenario.
    //! With RigidBodyBus it applies forces and torque according to PID control.
    //! @note This class is already used through ROS2FrameComponent.
    // TODO Add interface using EBus, test implement also rotation, add ramps, cascading controllers,
    // TODO automatic tests on test scene

    class MotorizedJoint
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(MotorizedJoint, "{AE9207DB-5B7E-4F70-A7DD-C4EAD8DD9403}", AZ::Component);

        // AZ::Component interface implementation.
        MotorizedJoint() = default;
        ~MotorizedJoint() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        //! Set a setpoint (e.g. desired local position). Controller follows it.
        void SetSetpoint(float setpoint);

        //! Get current control error. It is a difference between control value and measurement
        //! For steady state should be close to zero
        //! @returns error in meters for linear joints and radians for angular
        float GetError() const;

        //! Get current position from measurement.
        //! For steady state should be close to setpoint
        //! @returns current position  in meters for linear joints and radians for angular
        float GetCurrentPosition() const;

    private:
        float ComputeMeasurement(AZ::ScriptTimePoint time);

        void SetVelocity(float velocity, float deltaTime);

        // TODO apply RotVel...
        void ApplyLinVelAnimation(float velocity, float deltaTime);

        void ApplyLinVelRigidBodyImpulse(float velocity, float deltaTime);

        void ApplyLinVelRigidBody(float velocity, float deltaTime);

        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        ///! Direction of degree of freedom
        AZ::Vector3 m_jointDir{ 0.f, 0.f, 1.f };

        ///! Limits - on limit controller will apply force only in opposite direction
        AZStd::pair<float, float> m_limits{ -0.5f, 0.5f };

        ///! PID controller for position
        VehicleDynamics::PidConfiguration m_pidPos;

        ///! offset added to setpoint
        float m_zeroOffset{ 0.f };

        ///! Linear mode - apply force
        bool m_linear{ true };

        ///! Use TransformBus instead of RigidBodyBus
        bool m_animationMode{ true };

        ///! Enable sinusoidal signal generator to setpoint (for tuning)
        bool m_testSinusoidal{ true };

        ///! Print To console debug info
        bool m_debugPrint{ false };

        ///! Amplitude of test signal generator
        float m_sinAmplitude{ 0.25 };

        ///! Frequency of test signal generator
        float m_sinFreq{ 0.1 };

        ///! Last measured position
        float m_currentPosition{ 0 };

        ///! Last measured position
        float m_currentVelocity{ 0 };

        float m_setpoint{ 0 };
        float m_error{ 0 };
        double m_lastMeasurementTime;

        ///! Optional Entity that allows to visualize desired setpoint value
        AZ::EntityId m_debugDrawEntity;

        ///! initial transform of m_debugDrawEntity. It will be modified by the desired setpoint value in the set direction.
        AZ::Transform m_debugDrawEntityInitialTransform;
    };
} // namespace ROS2
