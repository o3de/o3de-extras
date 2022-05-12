/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/std/string/string.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

#include "RobotControl/RobotConfiguration.h"

#include "QoS/QoS.h"

namespace ROS2
{
//! Configuration for handling of robot control buses
struct ControlConfiguration
{
public:
    AZ_TYPE_INFO(ControlConfiguration, "{3D3E69EE-0F28-46D5-95F1-956550BA97B9}");

    enum Steering
    {
        Twist,
        Ackermann
    };

    static void Reflect(AZ::ReflectContext* context);

    QoS m_qos;
    AZStd::string m_topic = "o3de_robot_control";
    Steering m_steering = Steering::Twist;
    RobotConfiguration m_robotConfiguration;
    //! Switch between two modes. If enabled, only notification bus is running and no control is handled on
    //! component side.
    bool m_broadcastBusMode = true;

private:
    [[nodiscard]] bool IsBroadcastBusModeDisabled() const;

};
}  // namespace ROS2

