/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ImGuiBus.h>
#include <ROS2/Manipulation/MotorizedJoints/JointMotorControllerConfiguration.h>

namespace ROS2
{
    class JointMotorControllerComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
        , public AZ::EntityBus::Handler
    {
    public:
        JointMotorControllerComponent() = default;
        JointMotorControllerComponent(const JointMotorControllerComponent& other) = default;
        JointMotorControllerComponent(JointMotorControllerComponent&& other) = default;
        ~JointMotorControllerComponent() = default;
        AZ_COMPONENT(JointMotorControllerComponent, "{88e725fc-29d8-45b9-b3e8-bd268ad9f413}");

        // Component overrides
        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        // ImGui::ImGuiUpdateListenerBus overrides
        void OnImGuiUpdate() override;

        // EntityBus overrides
        void OnEntityActivated(const AZ::EntityId& entityId) override;

    protected:
        AZ::EntityComponentIdPair m_jointComponentIdPair; //!< Joint component managed by the motorized joint.
        float m_currentPosition{ 0.0f }; //!< Last measured position.
        float m_currentSpeed{ 0.0f }; //!< Last measured speed.

        JointMotorControllerConfiguration m_jointMotorControllerConfiguration;

    private:
        virtual float CalculateMotorSpeed([[maybe_unused]] float deltaTime)
        {
            return 0.0f;
        };

        virtual void DisplayControllerParameters(){};

        // AZ::TickBus overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
    };
} // namespace ROS2
