/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2Sensors/Odometry/ROS2OdometryCovariance.h>
#include <ROS2Sensors/Odometry/WheelOdometrySensorConfiguration.h>

namespace ROS2Sensors
{
    void WheelOdometrySensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        ROS2OdometryCovariance::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<WheelOdometrySensorConfiguration>()
                ->Version(1)
                ->Field("Pose covariance", &WheelOdometrySensorConfiguration::m_poseCovariance)
                ->Field("Twist covariance", &WheelOdometrySensorConfiguration::m_twistCovariance);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext
                    ->Class<WheelOdometrySensorConfiguration>("ROS2 Wheel Odometry configuration", "Wheel odometry sensor configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelOdometrySensorConfiguration::m_poseCovariance,
                        "Pose covariance",
                        "Set ROS pose covariance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &WheelOdometrySensorConfiguration::m_twistCovariance,
                        "Twist covariance",
                        "Set ROS twist covariance");
            }
        }
    }
} // namespace ROS2Sensors
