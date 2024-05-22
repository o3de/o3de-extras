/*
* Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

#include "LidarRegistrarSystemComponent.h"
#include "LidarTemplate.h"
#include "LidarTemplateUtils.h"

namespace ROS2
{
    //! A structure capturing configuration of a lidar sensor (to be used with LidarCore).
    class LidarSegmentationClassConfiguration
    {
    public:
        AZ_TYPE_INFO(LidarSegmentationClassConfiguration, "{e46e75f4-1e0e-48ca-a22f-43afc8f25133}");
        static void Reflect(AZ::ReflectContext* context);

        LidarSegmentationClassConfiguration() =default;
        LidarSegmentationClassConfiguration(const AZStd::string& className, uint8_t classId, const AZ::Color& classColor)
            : m_className(className)
            , m_classId(classId)
            , m_classColor(classColor)
        {
        };
        AZStd::string m_className = "Default";
        uint8_t m_classId = 0;
        AZ::Color m_classColor = AZ::Color(1.0f, 1.0f, 1.0f, 1.0f);
    };
} // namespace ROS2
