/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Holds information on robots parts such as wheels.
    //! This configuration is important for robot mobility and used within ROS2RobotControlComponent.
    // TODO - this structure is currently simplified, generalize into other types of mobile base and traction.
    struct RobotConfiguration
    {
    public:
        AZ_TYPE_INFO(RobotConfiguration, "{0E179498-AFCE-4589-A845-5BF1A35228DA}");

        static void Reflect(AZ::ReflectContext* context);

        //! Robot body object.
        AZ::EntityId m_body;

        //! Robot wheel objects.
        AZ::EntityId m_wheelFrontLeft;
        AZ::EntityId m_wheelFrontRight;
        AZ::EntityId m_wheelBackLeft;
        AZ::EntityId m_wheelBackRight;

    private:
        AZ::Outcome<void, AZStd::string> ValidateField(void* newValue, const AZ::Uuid& valueType);
    };
} // namespace ROS2
