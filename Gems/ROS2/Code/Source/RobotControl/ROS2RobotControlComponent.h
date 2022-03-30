/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <memory>
#include <AzCore/Component/Component.h>
#include "RobotControl.h"

namespace ROS2
{
    class ROS2RobotControlComponent
        : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2RobotControlComponent, "{CBFB0764-99F9-40EE-9FEE-F5F5A66E59D2}", AZ::Component);

        // AZ::Component interface implementation.
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

    private:
        AZStd::string m_topic = "/o3de_robot_control";
        std::unique_ptr<IRobotControl> m_robotControl;
    };
}  // namespace ROS2