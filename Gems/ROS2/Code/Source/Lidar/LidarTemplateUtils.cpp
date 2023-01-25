/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Lidar/LidarTemplateUtils.h>

namespace ROS2
{
    LidarTemplate LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel model)
    {
        static const std::unordered_map<LidarTemplate::LidarModel, LidarTemplate> templates = {
            {
                LidarTemplate::LidarModel::Custom3DLidar,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Custom3DLidar,
                    /*.m_name = */ "CustomLidar",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ -35.0f,
                    /*.m_maxVAngle = */ 35.0f,
                    /*.m_layers = */ 24,
                    /*.m_numberOfIncrements = */ 924,
                    /*.m_maxRange = */ 100.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.01f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
            {
                LidarTemplate::LidarModel::Ouster_OS0_64,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Ouster_OS0_64,
                    /*.m_name = */ "Ouster OS0-64",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ -45.0f,
                    /*.m_maxVAngle = */ 45.0f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_maxRange = */ 47.5f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.0f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.002f,
                    },
                },
            },
            {
                LidarTemplate::LidarModel::Ouster_OS1_64,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Ouster_OS1_64,
                    /*.m_name = */ "Ouster OS1-64",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 22.5f,
                    /*.m_maxVAngle = */ -22.5f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_maxRange = */ 120.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.002f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.0008f,
                    },
                },
            },
            {
                LidarTemplate::LidarModel::Ouster_OS2_64,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Ouster_OS1_64,
                    /*.m_name = */ "Ouster OS1-64",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 11.25f,
                    /*.m_maxVAngle = */ -11.25f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_maxRange = */ 225.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.006f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
            {
                LidarTemplate::LidarModel::Velodyne_Puck,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Velodyne_Puck,
                    /*.m_name = */ "Velodyne Puck (VLP-16)",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 15.0f,
                    /*.m_maxVAngle = */ -15.0f,
                    /*.m_layers = */ 16,
                    /*.m_numberOfIncrements = */ 1800, // For 0.2 angular resolution
                    /*.m_maxRange = */ 100.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.03f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
            {
                LidarTemplate::LidarModel::Velodyne_HDL_32E,
                {
                    /*.m_model = */ LidarTemplate::LidarModel::Velodyne_HDL_32E,
                    /*.m_name = */ "Velodyne HDL-32E",
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 10.67f,
                    /*.m_maxVAngle = */ -30.67f,
                    /*.m_layers = */ 32,
                    /*.m_numberOfIncrements = */ 1800, // For 0.2 angular resolution
                    /*.m_maxRange = */ 100.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.02f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
        };

        auto it = templates.find(model);
        if (it == templates.end())
        {
            return LidarTemplate();
        }

        return it->second;
    }

    size_t LidarTemplateUtils::TotalPointCount(const LidarTemplate& t)
    {
        return t.m_layers * t.m_numberOfIncrements;
    }

    AZStd::vector<AZ::Vector3> LidarTemplateUtils::PopulateRayRotations(const LidarTemplate& lidarTemplate)
    {
        const float minVertAngle = AZ::DegToRad(lidarTemplate.m_minVAngle);
        const float maxVertAngle = AZ::DegToRad(lidarTemplate.m_maxVAngle);
        const float minHorAngle = AZ::DegToRad(lidarTemplate.m_minHAngle);
        const float maxHorAngle = AZ::DegToRad(lidarTemplate.m_maxHAngle);

        const float verticalStep = (maxVertAngle - minVertAngle) / static_cast<float>(lidarTemplate.m_layers);
        const float horizontalStep = (maxHorAngle - minHorAngle) / static_cast<float>(lidarTemplate.m_numberOfIncrements);

        AZStd::vector<AZ::Vector3> rotations;
        for (int incr = 0; incr < lidarTemplate.m_numberOfIncrements; incr++)
        {
            for (int layer = 0; layer < lidarTemplate.m_layers; layer++)
            {
                const float pitch = minVertAngle + layer * verticalStep;
                const float yaw = minHorAngle + incr * horizontalStep;

                rotations.emplace_back(AZ::Vector3(0.0f, pitch, yaw));
            }
        }

        return rotations;
    }

    AZStd::vector<AZ::Vector3> LidarTemplateUtils::RotationsToDirections(
        const AZStd::vector<AZ::Vector3>& rotations, const AZ::Vector3& rootRotation)
    {
        AZStd::vector<AZ::Vector3> directions;
        directions.reserve(rotations.size());
        for (auto angle : rotations)
        {
            const AZ::Quaternion rotation = AZ::Quaternion::CreateFromEulerRadiansZYX(
                { 0.0f, -(angle.GetY() + rootRotation.GetY()), angle.GetZ() + rootRotation.GetZ() });

            directions.emplace_back(rotation.TransformVector(AZ::Vector3::CreateAxisX()));
        }

        return directions;
    }
} // namespace ROS2
