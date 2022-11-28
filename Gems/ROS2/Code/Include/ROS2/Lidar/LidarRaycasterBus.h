/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>

namespace ROS2
{
    class LidarRaycasterRequests
    {
    public:
        AZ_RTTI(LidarRaycasterRequests, "{253a02c8-b6cb-493c-b16f-012ccf9db226}");
        virtual ~LidarRaycasterRequests() = default;

        //! Configures ray orientations.
        //! @param orientations Vector of orientations as Euler angles in radians. Each ray direction is computed by transforming a unit
        //! vector in the positive z direction first by the y, next by the z axis. The x axis is currently not included in calculations.
        virtual void ConfigureRayOrientations(const AZStd::vector<AZ::Vector3>& orientations) = 0;

        //! Configures ray maximum travel distance.
        //! @param range Ray range in meters.
        virtual void ConfigureRayRange(float range) = 0;

        //! Schedules a raycast that originates from the point described by the lidarTransform.
        //! @param lidarTransform Current transform from global to lidar reference frame.
        //! @return Results of the raycast in form of coordinates in 3D space.
        //! The returned vector size can be anything between zero and size of directions. No hits further than distance will be reported.
        virtual AZStd::vector<AZ::Vector3> PerformRaycast(const AZ::Transform& lidarTransform) = 0;

        //! Configures ray Gaussian Noise parameters.
        //! Each call overrides the previous configuration.
        //! This type of noise is especially useful when trying to simulate real-life lidars, since it noise mimics
        //! the imperfections arising due to various physical factors e.g. fluctuations in rotary motion of the lidar (angular noise) or
        //! distance accuracy (distance noise).
        //! For the the details about Gaussian noise, please refer to https://en.wikipedia.org/wiki/Gaussian_noise.
        //! You can also check-out the RobotecGPULidar (integrated in the RobotecGPULidar Gem) docs concerning these types of lidar noise at
        //! https://github.com/RobotecAI/RobotecGPULidar/blob/v11/docs/GaussianNoise.md.
        //! @param angularNoiseStdDev Angular noise standard deviation.
        //! @param distanceNoiseStdDevBase Base value for Distance noise standard deviation.
        //! @param distanceNoiseStdDevRisePerMeter Value by which the distance noise standard deviation increases per meter length from
        //! the lidar.
        // TODO - different starting points for rays, distance from reference point, noise models, rotating mirror sim, other
        // TODO - customized settings. Encapsulate in lidar definition and pass in constructor, update transform.
        virtual void ConfigureNoiseParameters(
            float angularNoiseStdDev, float distanceNoiseStdDevBase, float distanceNoiseStdDevRisePerMeter)
        {
            AZ_Assert(false, "This Lidar Implementation does not support noise!");
        }

        //! Configures Layer ignoring parameters
        //! @param ignoreLayer Should a specified collision layer be ignored?
        //! @param layerIndex Index of collision layer to be ignored.
        virtual void ConfigureLayerIgnoring(bool ignoreLayer, unsigned int layerIndex)
        {
            AZ_Assert(false, "This Lidar Implementation does not support collision layer configurations!");
        }

        //! Excludes entities with given EntityIds from raycasting.
        //! @param excludedEntities list of entities marked for exclusion.
        virtual void ExcludeEntities(const AZStd::vector<AZ::EntityId>& excludedEntities)
        {
            AZ_Assert(false, "This Lidar Implementation does not support entity exclusion!");
        }

        //! Configures max range point addition.
        //! @param includeMaxRange Should the raycaster add points at max range for rays that exceeded their range?
        virtual void ConfigureMaxRangePointAddition(bool addMaxRangePoints)
        {
            AZ_Assert(false, "This Lidar Implementation does not support Max range point addition configuration!");
        }

    protected:
        static void ValidateRayRange(float range)
        {
            AZ_Assert(range > 0.0f, "Provided ray range was of incorrect value: Ray range value must be greater than zero.")
        }

        static void ValidateRayOrientations(const AZStd::vector<AZ::Vector3>& orientations)
        {
            AZ_Assert(!orientations.empty(), "Provided ray orientations were of incorrect value: Ray orientations must not be empty.")
        }
    };

    class LidarRaycasterBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = AZ::Uuid;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using LidarRaycasterRequestBus = AZ::EBus<LidarRaycasterRequests, LidarRaycasterBusTraits>;
} // namespace ROS2