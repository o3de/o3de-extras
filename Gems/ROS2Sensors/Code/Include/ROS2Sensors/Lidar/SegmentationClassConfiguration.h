/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Color.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! A structure capturing configuration of a segmentation class.
    class SegmentationClassConfiguration
    {
    public:
        AZ_TYPE_INFO(SegmentationClassConfiguration, SegmentationClassConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        static const SegmentationClassConfiguration UnknownClass;
        static const SegmentationClassConfiguration GroundClass;

        SegmentationClassConfiguration() = default;

        SegmentationClassConfiguration(const AZStd::string& className, const uint8_t classId, const AZ::Color& classColor)
            : m_className(className)
            , m_classId(classId)
            , m_classColor(classColor)
        {
        }

        AZStd::string m_className = "Default";
        uint8_t m_classId = 0;
        AZ::Color m_classColor = AZ::Color(1.0f, 1.0f, 1.0f, 1.0f);
    };
} // namespace ROS2Sensors
