/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2OdometryCovariance.h"
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/base.h>
#include <array>

namespace ROS2
{
    void ROS2OdometryCovariance::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2OdometryCovariance>()
                ->Version(1)
                ->Field("Linear covariance", &ROS2OdometryCovariance::m_linearCovariance)
                ->Field("Angular covariance", &ROS2OdometryCovariance::m_angularCovariance);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2OdometryCovariance>("ROS2 Odometry Covariance", "Define Odometry covariance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2OdometryCovariance::m_linearCovariance,
                        "Linear covariance",
                        "Set the ROS linear covariance")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2OdometryCovariance::m_angularCovariance,
                        "Angular covariance",
                        "Set the ROS angular covariance");
            }
        }
    }

    std::array<double, 36> ROS2OdometryCovariance::GetRosCovariance() const
    {
        const AZ::Vector3& lin = m_linearCovariance;
        const AZ::Vector3& ang = m_angularCovariance;

        // clang-format off
        return {
            lin.GetX(), 0.0L,       0.0L,       0.0L,       0.0L,       0.0L, 
            0.0L,       lin.GetY(), 0.0L,       0.0L,       0.0L,       0.0L,
            0.0L,       0.0L,       lin.GetZ(), 0.0L,       0.0L,       0.0L,
            0.0L,       0.0L,       0.0L,       ang.GetX(), 0.0L,       0.0L,
            0.0L,       0.0L,       0.0L,       0.0L,       ang.GetY(), 0.0L, 
            0.0L,       0.0L,       0.0L,       0.0L,       0.0L,       ang.GetZ() 
        };
        // clang-format on
    }

} // namespace ROS2
