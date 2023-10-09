/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Manipulation/MotorizedJoints/ManualMotorControllerComponent.h>
#include <imgui/imgui.h>

namespace ROS2
{
    ManualMotorControllerComponent::ManualMotorControllerComponent()
    {
        m_jointMotorControllerConfiguration.m_isDebugController = true;
    }

    void ManualMotorControllerComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ManualMotorControllerComponent, JointMotorControllerComponent>()->Version(2);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ManualMotorControllerComponent>("Manual Motor Controller", "Debug motor controller used via ImGui.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ManualMotorController.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ManualMotorController.svg");
            }
        }
    }

    float ManualMotorControllerComponent::CalculateMotorSpeed([[maybe_unused]] float deltaTime)
    {
        return m_setSpeed;
    }

    void ManualMotorControllerComponent::DisplayControllerParameters()
    {
        ImGui::PushItemWidth(200.0f);
        ImGui::SliderFloat("SetSpeed", &m_setSpeed, -5.0f, 5.0f);

        ImGui::PopItemWidth();

        if (ImGui::Button("Zero"))
        {
            m_setSpeed = 0.0f;
        }
    }
} // namespace ROS2
