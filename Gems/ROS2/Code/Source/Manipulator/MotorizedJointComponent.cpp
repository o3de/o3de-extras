/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2/Manipulator/MotorizedJointComponent.h"
#include "AzFramework/Physics/Components/SimulatedBodyComponentBus.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{
    void MotorizedJointComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_pidPos.InitializePid();
        if (m_debugDrawEntity.IsValid())
        {
            AZ::TransformBus::EventResult(m_debugDrawEntityInitialTransform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        }
        MotorizedJointRequestBus::Handler::BusConnect(m_entity->GetId());
    }

    void MotorizedJointComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        MotorizedJointRequestBus::Handler::BusDisconnect();
    }

    void MotorizedJointComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<MotorizedJointComponent, AZ::Component>()
                ->Version(2)
                ->Field("JointAxis", &MotorizedJointComponent::m_jointDir)
                ->Field("EffortAxis", &MotorizedJointComponent::m_effortAxis)
                ->Field("Limit", &MotorizedJointComponent::m_limits)
                ->Field("Linear", &MotorizedJointComponent::m_linear)
                ->Field("AnimationMode", &MotorizedJointComponent::m_animationMode)
                ->Field("ZeroOffset", &MotorizedJointComponent::m_zeroOffset)
                ->Field("PidPosition", &MotorizedJointComponent::m_pidPos)
                ->Field("DebugDrawEntity", &MotorizedJointComponent::m_debugDrawEntity)
                ->Field("TestSinActive", &MotorizedJointComponent::m_testSinusoidal)
                ->Field("TestSinAmplitude", &MotorizedJointComponent::m_sinAmplitude)
                ->Field("TestSinFreq", &MotorizedJointComponent::m_sinFreq)
                ->Field("TestSinDC", &MotorizedJointComponent::m_sinDC)
                ->Field("DebugPrint", &MotorizedJointComponent::m_debugPrint)
                ->Field("OverrideParent", &MotorizedJointComponent::m_measurementReferenceEntity);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<MotorizedJointComponent>("MotorizedJointComponent", "MotorizedJointComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "MotorizedJointComponent")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "MotorizedJointComponent")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_jointDir, "Joint Dir.", "Direction of joint in parent's reference frame.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_effortAxis, "Effort Dir.", "Desired direction of force/torque vector that is applied to rigid body.")

                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_limits,
                        "ControllerLimits",
                        "When measurement is outside the limits, ")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_debugDrawEntity,
                        "Setpoint",
                        "Allows to apply debug setpoint visualizer")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_zeroOffset,
                        "Zero Off.",
                        "Allows to change offset of zero to set point")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_linear,
                        "Linear joint",
                        "Applies linear force instead of torque")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_animationMode,
                        "Animation mode",
                        "In animation mode, the transform API is used instead of Rigid Body. "
                        "If this property is set to true the Rigid Body Component should be disabled.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_pidPos, "PidPosition", "PidPosition")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_testSinusoidal,
                        "SinusoidalTest",
                        "Allows to apply sinusoidal test signal")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_sinAmplitude,
                        "Amplitude",
                        "Amplitude of sinusoidal test signal.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_sinFreq,
                        "Frequency",
                        "Frequency of sinusoidal test signal.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_sinDC, "DC", "DC of sinusoidal test signal.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MotorizedJointComponent::m_debugPrint, "Debug", "Print debug to console")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &MotorizedJointComponent::m_measurementReferenceEntity,
                        "Step Parent",
                        "Allows to override a parent to get correct measurement");
            }
        }
    }
    void MotorizedJointComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        const float measurement = ComputeMeasurement(time);
        if (m_testSinusoidal)
        {
            m_setpoint = m_sinDC + m_sinAmplitude * AZ::Sin(m_sinFreq * time.GetSeconds());
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
                "MotorizedJointComponent",
                " %s | pos: %f | err: %f | cntrl : %f | set : %f |",
                GetEntity()->GetName().c_str(),
                measurement,
                control_position_error,
                speed_control,
                m_setpoint);
        }
        SetVelocity(speed_control, deltaTime);
    }

    float MotorizedJointComponent::ComputeMeasurement(AZ::ScriptTimePoint time)
    {
        AZ::Transform transform;
        if (!m_measurementReferenceEntity.IsValid())
        {
            AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        }
        else
        {
            AZ::Transform transformStepParent;
            AZ::TransformBus::EventResult(transformStepParent, m_measurementReferenceEntity, &AZ::TransformBus::Events::GetWorldTM);
            AZ::Transform transformStepChild;
            AZ::TransformBus::EventResult(transformStepChild, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
            transform = transformStepParent.GetInverse() * transformStepChild;
        }
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

    void MotorizedJointComponent::SetVelocity(float velocity, float deltaTime)
    {
        if (m_animationMode)
        {
            ApplyLinVelAnimation(velocity, deltaTime);
        }
        else
        {
            deltaTime = AZStd::min(deltaTime, 0.1f); // limit max force for small FPS
            if (m_linear)
            {
                // TODO decide which approach is better here.
                ApplyLinVelRigidBodyImpulse(velocity, deltaTime);
                // ApplyLinVelRigidBody(velocity, deltaTime);
            }
        }
    }

    void MotorizedJointComponent::ApplyLinVelAnimation(float velocity, float deltaTime)
    {
        AZ::Transform transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetLocalTM);
        transform.SetTranslation(transform.GetTranslation() + velocity * m_jointDir * deltaTime);
        AZ::TransformBus::Event(this->GetEntityId(), &AZ::TransformBus::Events::SetLocalTM, transform);
    }

    void MotorizedJointComponent::ApplyLinVelRigidBodyImpulse(float velocity, float deltaTime)
    {
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        auto force_impulse = transform.TransformVector(m_effortAxis * velocity);
        Physics::RigidBodyRequestBus::Event(
            this->GetEntityId(), &Physics::RigidBodyRequests::ApplyLinearImpulse, force_impulse * deltaTime);
    }

    void MotorizedJointComponent::ApplyLinVelRigidBody(float velocity, float deltaTime)
    {
        AZ::Quaternion transform;
        AZ::TransformBus::EventResult(transform, this->GetEntityId(), &AZ::TransformBus::Events::GetWorldRotationQuaternion);
        AZ::Vector3 currentVelocity;
        auto transformed_velocity_increment = transform.TransformVector(m_effortAxis * velocity);
        Physics::RigidBodyRequestBus::EventResult(currentVelocity, this->GetEntityId(), &Physics::RigidBodyRequests::GetLinearVelocity);
        AZ::Vector3 new_velocity = currentVelocity + transformed_velocity_increment;
        Physics::RigidBodyRequestBus::Event(this->GetEntityId(), &Physics::RigidBodyRequests::SetLinearVelocity, new_velocity);
    }

    void MotorizedJointComponent::SetSetpoint(float setpoint)
    {
        m_setpoint = setpoint;
    }

    float MotorizedJointComponent::GetSetpoint()
    {
        return m_setpoint;
    }

    float MotorizedJointComponent::GetError()
    {
        return m_error;
    }

    float MotorizedJointComponent::GetCurrentMeasurement()
    {
        return m_currentPosition - m_zeroOffset;
    }

} // namespace ROS2
