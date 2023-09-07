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
    //! Configuration reflecting a specific Lidar model.
    //! This is meant to capture differences between different Lidars available on the market.
    //! @note Current implementation is simplified. Rays in real lidars are often not uniformly
    //! distributed among angular range, there is noise etc.
    struct LidarTemplate
    {
    public:
        AZ_TYPE_INFO(LidarTemplate, "{9E9EF583-733D-4450-BBA0-ADD4D1BEFBF2}");
        static void Reflect(AZ::ReflectContext* context);

        enum class LidarModel
        {
            Custom3DLidar,
            Ouster_OS0_64,
            Ouster_OS1_64,
            Ouster_OS2_64,
            Velodyne_Puck,
            Velodyne_HDL_32E,
            // 2D Lidars
            Custom2DLidar,
            Slamtec_RPLIDAR_S1
        };

        struct NoiseParameters
        {
        public:
            AZ_TYPE_INFO(NoiseParameters, "{58c007ad-320f-49df-bc20-6419159ee176}");
            static void Reflect(AZ::ReflectContext* context);

            //! Angular noise standard deviation, in degrees
            float m_angularNoiseStdDev = 0.0f;
            //! Distance noise standard deviation base value, in meters
            float m_distanceNoiseStdDevBase = 0.0f;
            //! Distance noise standard deviation increase per meter distance traveled, in meters
            float m_distanceNoiseStdDevRisePerMeter = 0.0f;
        };

        LidarModel m_model;

        //! Name of lidar template
        AZStd::string m_name;
        //! Whether the template is for a 2D Lidar.
        //! This causes vertical parameters of the Lidar to be unmodifiable (m_minVAngle, m_maxVAngle, m_layers).
        bool m_is2D = false;
        //! Minimum horizontal angle (altitude of the ray), in degrees
        float m_minHAngle = 0.0f;
        //! Maximum horizontal angle (altitude of the ray), in degrees
        float m_maxHAngle = 0.0f;
        //! Minimum vertical angle (azimuth of the ray), in degrees
        float m_minVAngle = 0.0f;
        //! Maximum vertical angle (azimuth of the ray), in degrees
        float m_maxVAngle = 0.0f;
        //! Number of lasers layers (resolution in horizontal direction)
        unsigned int m_layers = 0;
        //! Resolution in vertical direction
        unsigned int m_numberOfIncrements = 0;
        //! Minimum range of simulated LiDAR
        float m_minRange = 0.0f;
        //! Maximum range of simulated LiDAR
        float m_maxRange = 0.0f;

        NoiseParameters m_noiseParameters;
        bool m_isNoiseEnabled = true;
        bool m_showNoiseConfig = false;

    private:
        bool IsLayersVisible() const;
        [[nodiscard]] bool IsNoiseConfigVisible() const;
    };
} // namespace ROS2
