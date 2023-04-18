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

namespace ROS2
{
    //! A structure capturing configuration of a single camera sensor with up to two image sources (color and depth).
    struct CameraSensorConfiguration
    {
        AZ_TYPE_INFO(CameraSensorConfiguration, "{386A2640-442B-473D-BC2A-665D049D7EF5}");
        static void Reflect(AZ::ReflectContext* context);

        float m_verticalFieldOfViewDeg = 90.0f; //!< Vertical field of view of camera sensor.
        int m_width = 640; //!< Camera image width in pixels.
        int m_height = 480; //!< Camera image height in pixels.
        bool m_colorCamera = true; //!< Use color camera?
        bool m_depthCamera = true; //!< Use depth camera?
    };
} // namespace ROS2
