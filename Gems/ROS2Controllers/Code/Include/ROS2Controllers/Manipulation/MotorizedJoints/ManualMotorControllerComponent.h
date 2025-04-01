/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Manipulation/MotorizedJoints/JointMotorControllerComponent.h>

namespace ROS2
{
    class ManualMotorControllerComponent : public JointMotorControllerComponent
    {
    public:
        AZ_COMPONENT(ManualMotorControllerComponent, "{0817634e-4862-4245-a66e-72d1a6939705}", JointMotorControllerComponent);

        ManualMotorControllerComponent();
        ~ManualMotorControllerComponent() = default;
        static void Reflect(AZ::ReflectContext* context);

    private:
        float m_setSpeed{ 0.0f };

        float CalculateMotorSpeed([[maybe_unused]] float deltaTime) override;
        void DisplayControllerParameters() override;
    };
} // namespace ROS2
