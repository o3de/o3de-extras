/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ROS2Sensors/Configuration/ROS2OdometryCovariance.h>
#include <ROS2Sensors/Configuration/ROS2WheelOdometryConfiguration.h>
#include <AzCore/Serialization/EditContext.h>

namespace ROS2
{
    void ROS2WheelOdometryConfiguration::Reflect(AZ::ReflectContext* context)
    {
        ROS2OdometryCovariance::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2WheelOdometryConfiguration>()
                ->Version(1)
                ->Field("Pose covariance", &ROS2WheelOdometryConfiguration::m_poseCovariance)
                ->Field("Twist covariance", &ROS2WheelOdometryConfiguration::m_twistCovariance);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2WheelOdometryConfiguration>("ROS2 Wheel Odometry configuration", "Wheel odometry sensor configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2WheelOdometryConfiguration::m_poseCovariance,
                        "Pose covariance",
                        "Set ROS pose covariance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2WheelOdometryConfiguration::m_twistCovariance,
                        "Twist covariance",
                        "Set ROS twist covariance");
            }
        }
    }
} // namespace ROS2
