/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <HingeJointComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <PrismaticJointComponent.h>
#include <ROS2/Manipulation/MotorizedJoints/JointMotorControllerComponent.h>
#include <imgui/imgui.h>

namespace ROS2
{
    void JointMotorControllerComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
        AZ::EntityBus::Handler::BusConnect(GetEntityId());
    }

    void JointMotorControllerComponent::Deactivate()
    {
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    void JointMotorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        JointMotorControllerConfiguration::Reflect(context);

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JointMotorControllerComponent, AZ::Component>()->Version(2)->Field(
                "JointMotorControllerConfiguration", &JointMotorControllerComponent::m_jointMotorControllerConfiguration);
            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<JointMotorControllerComponent>(
                      "Joint Motor Controller Component", "Base component for motor controller components.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointMotorControllerComponent::m_jointMotorControllerConfiguration,
                        "Motor Controller Configuration",
                        "Motor Controller Configuration");
            }
        }
    }

    void JointMotorControllerComponent::OnImGuiUpdate()
    {
        if (!m_jointComponentIdPair.GetEntityId().IsValid() ||
            !(m_jointMotorControllerConfiguration.m_isDebugController || m_jointMotorControllerConfiguration.m_debugMode))
        {
            return;
        }

        AZStd::pair<float, float> limits{ 0.0f, 0.0f };
        PhysX::JointRequestBus::EventResult(limits, m_jointComponentIdPair, &PhysX::JointRequests::GetLimits);
        PhysX::JointRequestBus::EventResult(m_currentSpeed, m_jointComponentIdPair, &PhysX::JointRequests::GetVelocity);

        AZStd::string s =
            AZStd::string::format("Joint Motor Controller %s:%s", GetEntity()->GetName().c_str(), GetEntity()->GetId().ToString().c_str());
        ImGui::Begin(s.c_str());

        ImGui::PushItemWidth(200.0f);
        ImGui::SliderFloat("Position", &m_currentPosition, limits.first, limits.second);
        ImGui::SliderFloat("Speed", &m_currentSpeed, -5.0f, 5.0f);
        ImGui::PopItemWidth();

        DisplayControllerParameters();

        ImGui::End();
    }

    void JointMotorControllerComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_jointComponentIdPair.GetEntityId().IsValid())
        {
            return;
        }

        PhysX::JointRequestBus::EventResult(m_currentPosition, m_jointComponentIdPair, &PhysX::JointRequests::GetPosition);
        float setSpeed = CalculateMotorSpeed(deltaTime);
        PhysX::JointRequestBus::Event(m_jointComponentIdPair, &PhysX::JointRequests::SetVelocity, setSpeed);
    }

    void JointMotorControllerComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        AZ::ComponentId componentId;
        if (auto* prismaticJointComponent = GetEntity()->FindComponent<PhysX::PrismaticJointComponent>(); prismaticJointComponent)
        {
            componentId = prismaticJointComponent->GetId();
        }
        else if (auto* hingeJointComponent = GetEntity()->FindComponent<PhysX::HingeJointComponent>(); hingeJointComponent)
        {
            componentId = hingeJointComponent->GetId();
        }
        else
        {
            AZ_Warning(
                "MotorizedJointComponent",
                false,
                "Entity with ID %s either has no PhysX::Joint component or the joint is neither a Prismatic nor a Hinge Joint",
                GetEntityId().ToString().c_str());
        }

        m_jointComponentIdPair = { GetEntityId(), componentId };
    }
} // namespace ROS2
