/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Transform.h>
#include <Lidar/LidarTemplateUtils.h>

namespace ROS2
{
    namespace
    {
        using Model = LidarTemplate::LidarModel;

        static const AZStd::map<Model, LidarTemplate> templates = {
            {
                Model::Custom3DLidar,
                {
                    /*.m_model = */ Model::Custom3DLidar,
                    /*.m_name = */ "CustomLidar",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ -35.0f,
                    /*.m_maxVAngle = */ 35.0f,
                    /*.m_layers = */ 24,
                    /*.m_numberOfIncrements = */ 924,
                    /*.m_minRange = */ 0.0f,
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
                Model::Ouster_OS0_64,
                {
                    /*.m_model = */ Model::Ouster_OS0_64,
                    /*.m_name = */ "Ouster OS0-64",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ -45.0f,
                    /*.m_maxVAngle = */ 45.0f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_minRange = */ 0.0f,
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
                Model::Ouster_OS1_64,
                {
                    /*.m_model = */ Model::Ouster_OS1_64,
                    /*.m_name = */ "Ouster OS1-64",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 22.5f,
                    /*.m_maxVAngle = */ -22.5f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_minRange = */ 0.0f,
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
                Model::Ouster_OS2_64,
                {
                    /*.m_model = */ Model::Ouster_OS2_64,
                    /*.m_name = */ "Ouster OS2-64",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 11.25f,
                    /*.m_maxVAngle = */ -11.25f,
                    /*.m_layers = */ 64,
                    /*.m_numberOfIncrements = */ 2048,
                    /*.m_minRange = */ 0.0f,
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
                Model::Velodyne_Puck,
                {
                    /*.m_model = */ Model::Velodyne_Puck,
                    /*.m_name = */ "Velodyne Puck (VLP-16)",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 15.0f,
                    /*.m_maxVAngle = */ -15.0f,
                    /*.m_layers = */ 16,
                    /*.m_numberOfIncrements = */ 1800, // For 0.2 angular resolution
                    /*.m_minRange = */ 0.0f,
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
                Model::Velodyne_HDL_32E,
                {
                    /*.m_model = */ Model::Velodyne_HDL_32E,
                    /*.m_name = */ "Velodyne HDL-32E",
                    /*.m_is2D = */ false,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 10.67f,
                    /*.m_maxVAngle = */ -30.67f,
                    /*.m_layers = */ 32,
                    /*.m_numberOfIncrements = */ 1800, // For 0.2 angular resolution
                    /*.m_minRange = */ 0.0f,
                    /*.m_maxRange = */ 100.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.02f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
            {
                Model::Custom2DLidar,
                {
                    /*.m_model = */ Model::Custom2DLidar,
                    /*.m_name = */ "CustomLidar2D",
                    /*.m_is2D = */ true,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 0.f,
                    /*.m_maxVAngle = */ 0.f,
                    /*.m_layers = */ 1,
                    /*.m_numberOfIncrements = */ 924,
                    /*.m_minRange = */ 0.0f,
                    /*.m_maxRange = */ 100.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.02f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            },
            {
                Model::Slamtec_RPLIDAR_S1,
                {
                    /*.m_model = */ Model::Slamtec_RPLIDAR_S1,
                    /*.m_name = */ "Slamtec RPLIDAR S1",
                    /*.m_is2D = */ true,
                    /*.m_minHAngle = */ -180.0f,
                    /*.m_maxHAngle = */ 180.0f,
                    /*.m_minVAngle = */ 0.f,
                    /*.m_maxVAngle = */ 0.f,
                    /*.m_layers = */ 1,
                    /*.m_numberOfIncrements = */ 921,
                    /*.m_minRange = */ 0.1f,
                    /*.m_maxRange = */ 40.0f,
                    /*.m_noiseParameters = */
                    {
                        /*.m_angularNoiseStdDev = */ 0.0f,
                        /*.m_distanceNoiseStdDevBase = */ 0.02f,
                        /*.m_distanceNoiseStdDevRisePerMeter = */ 0.001f,
                    },
                },
            }
        };
    } // namespace

    LidarTemplate LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel model)
    {
        auto it = templates.find(model);
        if (it == templates.end())
        {
            return {};
        }

        return it->second;
    }

    AZStd::vector<LidarTemplate::LidarModel> LidarTemplateUtils::Get2DModels()
    {
        AZStd::vector<LidarTemplate::LidarModel> result;

        for (const auto& it : templates)
        {
            if (it.second.m_is2D)
            {
                result.push_back(it.first);
            }
        }
        return result;
    }

    AZStd::vector<LidarTemplate::LidarModel> LidarTemplateUtils::Get3DModels()
    {
        AZStd::vector<LidarTemplate::LidarModel> result;

        for (const auto& it : templates)
        {
            if (!it.second.m_is2D)
            {
                result.push_back(it.first);
            }
        }
        return result;
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
            const float yaw = minHorAngle + incr * horizontalStep;
            for (int layer = 0; layer < lidarTemplate.m_layers; layer++)
            {
                const float pitch = minVertAngle + layer * verticalStep;

                rotations.emplace_back(0.0f, pitch, yaw);
            }
        }

        return rotations;
    }

    AZStd::vector<AZ::Vector3> LidarTemplateUtils::RotationsToDirections(
        const AZStd::vector<AZ::Vector3>& rotations, const AZ::Transform& rootTransform)
    {
        AZStd::vector<AZ::Vector3> directions;
        directions.reserve(rotations.size());
        for (const auto& angle : rotations)
        {
            const AZ::Quaternion rotation =
                rootTransform.GetRotation() * AZ::Quaternion::CreateFromEulerRadiansZYX({ 0.0f, -angle.GetY(), angle.GetZ() });
            directions.emplace_back(rotation.TransformVector(AZ::Vector3::CreateAxisX()));
        }

        return directions;
    }
} // namespace ROS2
