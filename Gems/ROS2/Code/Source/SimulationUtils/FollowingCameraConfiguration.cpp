/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowingCameraConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void FollowingCameraConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FollowingCameraConfiguration>()
                ->Version(1)
                ->Field("PredefinedViews", &FollowingCameraConfiguration::m_predefinedViews)
                ->Field("SmoothingLength", &FollowingCameraConfiguration::m_smoothingBuffer)
                ->Field("SmoothFactor", &FollowingCameraConfiguration::m_smoothFactor)
                ->Field("RotationSpeed", &FollowingCameraConfiguration::m_rotationSpeed)
                ->Field("TranslationSpeed", &FollowingCameraConfiguration::m_translationSpeed)
                ->Field("LockZAxis", &FollowingCameraConfiguration::m_lockZAxis)
                ->Field("RotationSensitivity", &FollowingCameraConfiguration::m_rotationSensitivity)
                ->Field("DefaultView", &FollowingCameraConfiguration::m_defaultView);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FollowingCameraConfiguration>("Follow Camera Configuration", "Configuration for the Following Camera Component")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_smoothingBuffer,
                        "Smoothing Length",
                        "Number of past transforms used to smooth, larger value gives smoother result, but more lag")
                    ->Attribute(AZ::Edit::Attributes::Min, 1)
                    ->Attribute(AZ::Edit::Attributes::Max, 100)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_smoothFactor,
                        "Smooth Factor",
                        "Smoothing factor for camera movement (the higher the value, the newer entries have a progressively higher influence on the result.). Value 1.1 increase each subsequent weight by 10%")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 2.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_rotationSpeed,
                        "Rotation Speed",
                        "Rotation Speed around with keyboard QE movement")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_translationSpeed,
                        "Translation Speed",
                        "Translation Speed with keyboard WSAD movement")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_rotationSensitivity,
                        "Rotation Sensitivity",
                        "Sensitivity of mouse rotation")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &FollowingCameraConfiguration::m_predefinedViews, "Views", "Views to follow")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_lockZAxis,
                        "Lock Z Axis",
                        "Prevent camera from tilting")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraConfiguration::m_defaultView,
                        "Default View",
                        "Default View to follow")
                    ->Attribute(AZ::Edit::Attributes::Min, 0);
            }
        }
    }

} // namespace ROS2
