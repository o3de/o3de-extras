/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    struct LidarTemplate
    {
    public:
        AZ_TYPE_INFO(LidarTemplate, "{9E9EF583-733D-4450-BBA0-ADD4D1BEFBF2}");
        static void Reflect(AZ::ReflectContext* context);

        enum LidarModel
        {
            Generic3DLidar
        };

        LidarModel m_model;
        AZStd::string m_name;
        float m_minHAngle;
        float m_maxHAngle;
        float m_minVAngle;
        float m_maxVAngle;
        int m_layers;
        int m_numberOfIncrements;
        float m_maxRange;
    };
} // namespace ROS2
