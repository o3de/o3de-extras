/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2/Manipulator/MotorizedJoint.h"

#include "AzFramework/Physics/Components/SimulatedBodyComponentBus.h"
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void MotorizedJoint::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_pidPos.InitializePid();
        if (m_debugDrawEntity.IsValid())
        {
            AZ::TransformBus::EventResult(m_debugDrawEntityInitialTransform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        }
    }

    void MotorizedJoint::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void MotorizedJoint::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorizedJoint, AZ::Component>()
                ->Version(1)
                ->Field("JointAxis", &MotorizedJoint::m_jointDir)
                ->Field("Limit", &MotorizedJoint::m_limits)
                ->Field("Linear", &MotorizedJoint::m_linear)
                ->Field("AnimationMode", &MotorizedJoint::m_animationMode)
                ->Field("ZeroOffset", &MotorizedJoint::m_zeroOffset)
                ->Field("PidPosition", &MotorizedJoint::m_pidPos)
                ->Field("DebugDrawEntity", &MotorizedJoint::m_debugDrawEntity)
                ->Field("TestSinActive", &MotorizedJoint::m_testSinusoidal)
                ->Field("TestSinAmplitude", &MotorizedJoint::m_sinAmplitude)
                ->Field("TestSinFreq", &MotorizedJoint::m_sinFreq)
                ->Field("DebugPrint", &MotorizedJoint::m_debugPrint);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MotorizedJoint>("MotorizedJoint", "MotorizedJoint")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "MotorizedJoint")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "MotorizedJoint")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_jointDir, "Dir.", "Direction of joint.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_limits,
                        "ControllerLimits",
                        "When measurement is outside the limits, ")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_debugDrawEntity,
                        "Setpoint",
                        "Allows to apply debug setpoint visualizer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_zeroOffset,
                        "Zero Off.",
                        "Allows to change offset of zero to set point")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_linear, "Linear joint", "Applies linear force instead of torque")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_animationMode,
                        "Animation mode",
                        "In animation mode, the transform API is used instead of Rigid Body. "
                        "If this property is set to true the Rigid Body Component should be disabled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_pidPos, "PidPosition", "PidPosition")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJoint::m_testSinusoidal,
                        "SinusoidalTest",
                        "Allows to apply sinusoidal test signal")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_sinAmplitude, "Amplitude", "Amplitude of sinusoidal test signal.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_sinFreq, "Frequency", "Frequency of sinusoidal test signal.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJoint::m_debugPrint, "Debug", "Print debug to console");
            }
        }
    }
    void MotorizedJoint::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const float measurement = ComputeMeasurement(time);
        if (m_testSinusoidal)
        {
            m_setpoint = m_sinAmplitude * AZ::Sin(m_sinFreq * time.GetSeconds());
        }
        const float control_position_error = (m_setpoint + m_zeroOffset) - measurement;
        m_error = control_position_error; // TODO decide if we want to expose this control error.

        if (m_debugDrawEntity.IsValid())
        {
            if (m_linear)
            {
                AZ::Transform transform = AZ::Transform::Identity();
                transform.SetTranslation(m_jointDir * (m_setpoint + m_zeroOffset));
                AZ::TransformBus::Event(
                    m_debugDrawEntity, &AZ::TransformBus::Events::SetLocalTM, transform * m_debugDrawEntityInitialTransform);
            }
            else
            {
                AZ_Assert(false, "Not implemented");
            }
        }

        const uint64_t deltaTimeNs = deltaTime * 1'000'000'000;
        float speed_control = m_pidPos.ComputeCommand(control_position_error, deltaTimeNs);

        if (measurement <= m_limits.first)
        {
            // allow only positive control
            speed_control = AZStd::max(0.f, speed_control);
        }
        else if (measurement >= m_limits.second)
        {
            // allow only negative control
            speed_control = AZStd::min(0.f, speed_control);
        }

        if (m_debugPrint)
        {
            AZ_Printf(
                "MotorizedJoint",
                " %s | pos: %f | err: %f | cntrl : %f |",
                GetEntity()->GetName().c_str(),
                measurement,
                control_position_error,
                speed_control);
        }
        SetVelocity(speed_control, deltaTime);
    }

    float MotorizedJoint::ComputeMeasurement(AZ::ScriptTimePoint time)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        if (m_linear)
        {
            const float last_position = m_currentPosition;
            m_currentPosition = transform.GetTranslation().Dot(this->m_jointDir);
            if (m_lastMeasurementTime > 0)
            {
                double delta_time = time.GetSeconds() - m_lastMeasurementTime;
                m_currentVelocity = (m_currentPosition - last_position) / delta_time;
            }
            m_lastMeasurementTime = time.GetSeconds();
            return m_currentPosition;
        }
        AZ_Assert(false, "it is not implemented");
        return 0;
    }

    void MotorizedJoint::SetVelocity(float velocity, float deltaTime)
    {
        if (m_animationMode)
        {
            ApplyLinVelAnimation(velocity, deltaTime);
        }
        else
        {
            if (m_linear)
            {
                // TODO decide which approach is better here.
                ApplyLinVelRigidBodyImpulse(velocity, deltaTime);
                // ApplyLinVelRigidBody(velocity, deltaTime);
            }
        }
    }

    void MotorizedJoint::ApplyLinVelAnimation(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        transform.SetTranslation(transform.GetTranslation() + velocity * m_jointDir * deltaTime);
        AZ::TransformBus::Event(this->GetEntityId(), &AZ::TransformBus::Events::SetLocalTM, transform);
    }

    void MotorizedJoint::ApplyLinVelRigidBodyImpulse(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        auto force_impulse = transform.TransformVector(m_jointDir * velocity);
        Physics::RigidBodyRequestBus::Event(
            this->GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, force_impulse * deltaTime);
    }

    void MotorizedJoint::ApplyLinVelRigidBody(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        AZ::Vector3 currentVelocity;
        auto transformed_velocity_increment = transform.TransformVector(m_jointDir * velocity);
        Physics::RigidBodyRequestBus::EventResult(currentVelocity, this->GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
        AZ::Vector3 new_velocity = currentVelocity + transformed_velocity_increment;
        Physics::RigidBodyRequestBus::Event(this->GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, new_velocity);
    }

    void MotorizedJoint::SetSetpoint(float setpoint)
    {
        m_setpoint = setpoint;
    }

    float MotorizedJoint::GetError() const
    {
        return m_error;
    }

    float MotorizedJoint::GetCurrentPosition() const
    {
        return m_currentPosition;
    }

} // namespace ROS2
