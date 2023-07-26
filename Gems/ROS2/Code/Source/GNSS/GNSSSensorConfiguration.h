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
    //! A structure capturing configuration of a Global Navigation Satellite Systems (GNSS) sensor.
    struct GNSSSensorConfiguration
    {
        AZ_TYPE_INFO(GNSSSensorConfiguration, "{8bc39c6b-2f2c-4612-bcc7-0cc7033001d4}");
        static void Reflect(AZ::ReflectContext* context);

        float m_originLatitudeDeg = 0.0f;
        float m_originLongitudeDeg = 0.0f;
        float m_originAltitude = 0.0f;
    };
} // namespace ROS2
