/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ImuSensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void ImuSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ImuSensorConfiguration>()
                ->Version(1)
                ->Field("FilterSize", &ImuSensorConfiguration::m_filterSize)
                ->Field("IncludeGravity", &ImuSensorConfiguration::m_includeGravity)
                ->Field("AbsoluteRotation", &ImuSensorConfiguration::m_absoluteRotation)
                ->Field("AccelerationVariance", &ImuSensorConfiguration::m_linearAccelerationVariance)
                ->Field("AngularVelocityVariance", &ImuSensorConfiguration::m_angularVelocityVariance)
                ->Field("OrientationVariance", &ImuSensorConfiguration::m_orientationVariance);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ImuSensorConfiguration>("ROS2 IMU sensor configuration", "IMU sensor configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Slider,
                        &ImuSensorConfiguration::m_filterSize,
                        "Filter Length",
                        "Filter Length, Large value reduce numeric noise but increase lag")
                    ->Attribute(AZ::Edit::Attributes::Max, &ImuSensorConfiguration::m_maxFilterSize)
                    ->Attribute(AZ::Edit::Attributes::Min, &ImuSensorConfiguration::m_minFilterSize)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImuSensorConfiguration::m_includeGravity,
                        "Include Gravitation",
                        "Enable accelerometer to observe gravity force.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImuSensorConfiguration::m_absoluteRotation,
                        "Absolute Rotation",
                        "Include Absolute rotation in message.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImuSensorConfiguration::m_linearAccelerationVariance,
                        "Linear Acceleration Variance",
                        "Variance of linear acceleration.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImuSensorConfiguration::m_angularVelocityVariance,
                        "Angular Velocity Variance",
                        "Variance of angular velocity.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ImuSensorConfiguration::m_orientationVariance,
                        "Orientation Variance",
                        "Variance of orientation.");
            }
        }
    }

} // namespace ROS2
