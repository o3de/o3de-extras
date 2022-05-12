/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarTemplateUtils.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Utils/Utils.h>

namespace ROS2
{
    LidarTemplate LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel model)
    {
        static std::unordered_map<LidarTemplate::LidarModel, LidarTemplate> templates;

        if (templates.empty())
        {
            LidarTemplate generic3DLidar =
            {
                .m_model = LidarTemplate::Generic3DLidar,
                .m_name = "GenericLidar",
                .m_minHAngle = -180.0f,
                .m_maxHAngle = 180.0f,
                .m_minVAngle = 35.0f,
                .m_maxVAngle = -35.0f,
                .m_layers = 24,
                .m_numberOfIncrements = 924,
                .m_maxRange = 100.0f
            };
            templates[LidarTemplate::Generic3DLidar] = generic3DLidar;
        }

        auto it = templates.find(model);
        if (it == templates.end())
        {
            return LidarTemplate(); // TODO - handle it
        }

        return it->second;
    }

    size_t LidarTemplateUtils::TotalPointCount(const LidarTemplate& t)
    {
        return t.m_layers * t.m_numberOfIncrements;
    }

    // TODO - lidars in reality do not have uniform distributions - populating needs to be defined per model
    AZStd::vector<AZ::Vector3> LidarTemplateUtils::PopulateRayDirections(LidarTemplate::LidarModel model,
                                                                         const AZ::Vector3& rootRotation)
    {
        auto lidarTemplate = GetTemplate(model);

        const float minVertAngle = AZ::DegToRad(lidarTemplate.m_minVAngle);
        const float maxVertAngle = AZ::DegToRad(lidarTemplate.m_maxVAngle);
        const float minHorAngle = AZ::DegToRad(lidarTemplate.m_minHAngle);
        const float maxHorAngle = AZ::DegToRad(lidarTemplate.m_maxHAngle);

        const float verticalStep = (maxVertAngle - minVertAngle)
            / static_cast<float>(lidarTemplate.m_layers);
        const float horizontalStep = (maxHorAngle - minHorAngle)
            / static_cast<float>(lidarTemplate.m_numberOfIncrements);

        AZStd::vector<AZ::Vector3> directions;

        for (int incr = 0; incr < lidarTemplate.m_numberOfIncrements; incr++)
        {
            for (int layer = 0; layer < lidarTemplate.m_layers; layer++)
            {
                // TODO: also include roll. Move to quaternions to avoid abnormalities
                const float pitch = minVertAngle + layer * verticalStep + rootRotation.GetY();
                const float yaw = minHorAngle + incr * horizontalStep + rootRotation.GetZ();

                const float x = AZ::Cos(yaw) * AZ::Cos(pitch);
                const float y = AZ::Sin(yaw) * AZ::Cos(pitch);
                const float z = AZ::Sin(pitch);

                directions.push_back(AZ::Vector3(x, y, z));
            }
        }

        return directions;
    }
} // namespace ROS2
