/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <ROS2/Manipulation/MotorizedJoints/PidMotorControllerComponent.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void PidMotorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PidMotorControllerComponent, JointMotorControllerComponent>()
                ->Field("ZeroOffset", &PidMotorControllerComponent::m_zeroOffset)
                ->Field("PidPosition", &PidMotorControllerComponent::m_pidPos)
                ->Version(2);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<PidMotorControllerComponent>("PID Motor Controller", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/PidMotorController.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/PidMotorController.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &PidMotorControllerComponent::m_zeroOffset,
                        "Zero Offset",
                        "Allows to change offset of zero to set point")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &PidMotorControllerComponent::m_pidPos, "Pid Position", "Pid Position");
            }
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<PidMotorControllerRequestBus>("PidMotorControllerComponent", "PidMotorControllerRequestBus")
                ->Attribute(AZ::Edit::Attributes::Category, "ROS2/PidMotorController")
                ->Event("SetSetpoint", &PidMotorControllerRequestBus::Events::SetSetpoint)
                ->Event("GetSetpoint", &PidMotorControllerRequestBus::Events::GetSetpoint)
                ->Event("GetCurrentMeasurement", &PidMotorControllerRequestBus::Events::GetCurrentMeasurement)
                ->Event("GetError", &PidMotorControllerRequestBus::Events::GetError);
        }
    }

    void PidMotorControllerComponent::Activate()
    {
        m_pidPos.InitializePid();
        PidMotorControllerRequestBus::Handler::BusConnect(GetEntityId());
        JointMotorControllerComponent::Activate();
    }

    void PidMotorControllerComponent::Deactivate()
    {
        JointMotorControllerComponent::Deactivate();
        PidMotorControllerRequestBus::Handler::BusDisconnect();
    }

    void PidMotorControllerComponent::SetSetpoint(float setpoint)
    {
        m_setPoint = setpoint;
    }

    float PidMotorControllerComponent::GetSetpoint()
    {
        return m_setPoint;
    }

    float PidMotorControllerComponent::GetCurrentMeasurement()
    {
        return m_currentPosition - m_zeroOffset;
    }

    float PidMotorControllerComponent::GetError()
    {
        return m_error;
    }

    float PidMotorControllerComponent::CalculateMotorSpeed([[maybe_unused]] float deltaTime)
    {
        const float controlPositionError = (m_setPoint + m_zeroOffset) - m_currentPosition;
        m_error = controlPositionError;

        const auto deltaTimeNs = aznumeric_cast<uint64_t>(deltaTime * 1.0e9f);
        return aznumeric_cast<float>(m_pidPos.ComputeCommand(controlPositionError, deltaTimeNs));
    }

    void PidMotorControllerComponent::DisplayControllerParameters()
    {
        AZStd::pair<float, float> limits{ 0.0f, 0.0f };
        PhysX::JointRequestBus::EventResult(limits, m_jointComponentIdPair, &PhysX::JointRequests::GetLimits);

        ImGui::PushItemWidth(200.0f);
        ImGui::SliderFloat("SetPoint", &m_setPoint, limits.first + m_zeroOffset, limits.second + m_zeroOffset);
        ImGui::PopItemWidth();
    }
} // namespace ROS2
