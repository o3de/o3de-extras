/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! A structure capturing configuration of a single camera sensor with up to two image sources (color and depth).
    struct CameraSensorConfiguration
    {
        AZ_TYPE_INFO(CameraSensorConfiguration, CameraSensorConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr int m_minWidth = 1;
        static constexpr int m_minHeight = 1;
        static constexpr float m_minVerticalFieldOfViewDeg = 0.0f;
        static constexpr float m_maxVerticalFieldOfViewDeg = 360.0f;

        float m_verticalFieldOfViewDeg = 90.0f; //!< Vertical field of view of camera sensor.
        int m_width = 640; //!< Camera image width in pixels.
        int m_height = 480; //!< Camera image height in pixels.
        bool m_colorCamera = true; //!< Use color camera?
        bool m_depthCamera = true; //!< Use depth camera?
        float m_nearClipDistance = 0.1f; //!< Near clip distance of the camera.
        float m_farClipDistance = 100.0f; //!< Far clip distance of the camera.
    };
} // namespace ROS2Sensors
