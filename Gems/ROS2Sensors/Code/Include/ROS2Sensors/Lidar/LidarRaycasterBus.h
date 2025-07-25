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
#include <AzCore/Outcome/Outcome.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2Sensors/Lidar/RaycastResults.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Class used for creating typesafe Uuid types.
    //! It utilizes the phantom types technique.
    template<typename Tag>
    class StronglyTypedUuid
    {
    public:
        StronglyTypedUuid() = default;
        explicit constexpr StronglyTypedUuid(AZ::Uuid value)
            : m_uuid(value)
        {
        }

        constexpr static StronglyTypedUuid CreateNull()
        {
            return StronglyTypedUuid(AZ::Uuid::CreateNull());
        }

        constexpr static StronglyTypedUuid CreateRandom()
        {
            return StronglyTypedUuid(AZ::Uuid::CreateRandom());
        }

        constexpr bool IsNull() const
        {
            return m_uuid.IsNull();
        }

        constexpr bool operator==(const StronglyTypedUuid& rhs) const
        {
            return m_uuid == rhs.m_uuid;
        }

        constexpr bool operator!=(const StronglyTypedUuid& rhs) const
        {
            return m_uuid != rhs.m_uuid;
        }

        constexpr bool operator<(const StronglyTypedUuid& rhs) const
        {
            return m_uuid < rhs.m_uuid;
        }

        constexpr bool operator>(const StronglyTypedUuid& rhs) const
        {
            return m_uuid > rhs.m_uuid;
        }

        constexpr bool operator<=(const StronglyTypedUuid& rhs) const
        {
            return m_uuid <= rhs.m_uuid;
        }

        constexpr bool operator>=(const StronglyTypedUuid& rhs) const
        {
            return m_uuid >= rhs.m_uuid;
        }

        constexpr size_t GetHash() const
        {
            return m_uuid.GetHash();
        }

    private:
        AZ::Uuid m_uuid;
    };

    //! Unique id used by lidar raycasters.
    using LidarId = StronglyTypedUuid<struct LidarIdTag>;

    //! Structure used to describe both minimal and maximal
    //! ray travel distance in meters.
    struct RayRange
    {
        float m_min{ 0.0f };
        float m_max{ 0.0f };
    };

    //! Interface class that allows for communication with a single Lidar instance.
    class LidarRaycasterRequests
    {
    public:
        AZ_RTTI(LidarRaycasterRequests, LidarRaycasterBusTypeId);

        //! Configures ray orientations.
        //! @param orientations Vector of orientations as Euler angles in radians. Each ray direction is computed by transforming a unit
        //! vector in the positive z direction first by the y, next by the z axis. The x axis is currently not included in calculations.
        virtual void ConfigureRayOrientations(const AZStd::vector<AZ::Vector3>& orientations) = 0;

        //! Configures ray range.
        //! @param range Ray range.
        virtual void ConfigureRayRange(RayRange range) = 0;

        //! Configures result flags.
        //! @param flags Raycast result flags define set of data types returned by lidar.
        virtual void ConfigureRaycastResultFlags(RaycastResultFlags flags)
        {
            AZ_Assert(false, "This Lidar Implementation does not support configurable result flags!");
        }

        //! Schedules a raycast that originates from the point described by the lidarTransform.
        //! @param lidarTransform Current transform from global to lidar reference frame.
        //! @param flags Used to request different kinds of data returned by raycast query
        //! @return Results of the raycast in the requested form if the raycast was successfull or an error message if it was not.
        //! The returned error messages are c-style string literals which are statically allocated and therefore do not need to be
        //! dealocated.
        virtual AZ::Outcome<RaycastResults, const char*> PerformRaycast(const AZ::Transform& lidarTransform) = 0;

        //! Configures ray Gaussian Noise parameters.
        //! Each call overrides the previous configuration.
        //! This type of noise is especially useful when trying to simulate real-life lidars, since its noise mimics
        //! the imperfections arising due to various physical factors e.g. fluctuations in rotary motion of the lidar (angular noise) or
        //! distance accuracy (distance noise).
        //! For the the details about Gaussian noise, please refer to https://en.wikipedia.org/wiki/Gaussian_noise.
        //! @param angularNoiseStdDev Angular noise standard deviation.
        //! @param distanceNoiseStdDevBase Base value for Distance noise standard deviation.
        //! @param distanceNoiseStdDevRisePerMeter Value by which the distance noise standard deviation increases per meter length from
        //! the lidar.
        virtual void ConfigureNoiseParameters(
            [[maybe_unused]] float angularNoiseStdDev,
            [[maybe_unused]] float distanceNoiseStdDevBase,
            [[maybe_unused]] float distanceNoiseStdDevRisePerMeter)
        {
            AZ_Assert(false, "This Lidar Implementation does not support noise!");
        }

        //! Configures which collision layers should be ignored.
        //! @param layerIndices Indices of collision layers to be ignored.
        virtual void ConfigureIgnoredCollisionLayers([[maybe_unused]] const AZStd::unordered_set<AZ::u32>& layerIndices)
        {
            AZ_Assert(false, "This Lidar Implementation does not support collision layer configurations!");
        }

        //! Excludes entities with given EntityIds from raycasting.
        //! @param excludedEntities List of entities marked for exclusion.
        virtual void ExcludeEntities([[maybe_unused]] const AZStd::vector<AZ::EntityId>& excludedEntities)
        {
            AZ_Assert(false, "This Lidar Implementation does not support entity exclusion!");
        }

        //! Configures max range point addition.
        //! @param includeMaxRange Should the raycaster add points at max range for rays that exceeded their range?
        virtual void ConfigureMaxRangePointAddition([[maybe_unused]] bool addMaxRangePoints)
        {
            AZ_Assert(false, "This Lidar Implementation does not support Max range point addition configuration!");
        }

        //! Enables and configures raycaster-side Point Cloud Publisher.
        //! If not called, no publishing (raycaster-side) is performed. For some implementations it might be beneficial
        //! to publish internally (e.g. for the RGL gem, published points can be transformed from global to sensor
        //! coordinates on the GPU and published without unnecessary data copying or CPU manipulation) This API enables
        //! raycaster implementations that also handle publishing and provides them with necessary publisher configuration.
        //! @param topicName Name of the ROS 2 topic the pointcloud is published on.
        //! @param frameId Id of the ROS 2 frame of the sensor.
        //! @param qoSPolicy QoS policy of published pointcloud messages.
        virtual void ConfigurePointCloudPublisher(
            [[maybe_unused]] const AZStd::string& topicName,
            [[maybe_unused]] const AZStd::string& frameId,
            [[maybe_unused]] const ROS2::QoS& qoSPolicy)
        {
            AZ_Assert(false, "This Lidar Implementation does not support PointCloud publishing!");
        }

        //! Updates the timestamp of the messages published by the raycaster.
        //! @param timestampNanoseconds timestamp in nanoseconds
        //! (Time.msg: sec = timestampNanoseconds / 10^9; nanosec = timestampNanoseconds mod 10^9).
        virtual void UpdatePublisherTimestamp([[maybe_unused]] AZ::u64 timestampNanoseconds)
        {
            AZ_Assert(false, "This Lidar Implementation does not support PointCloud publishing!");
        }

        //! Can the raycaster handle publishing?
        //! This function should be called after the raycaster has been configured.
        //! The raycaster may not be able to handle point-cloud publishing in certain configurations (e.g. when the maxPointAddition
        //! is selected) in which case publishing must be handled somewhere else (e.g. by the ROS2LidarComponent).
        virtual bool CanHandlePublishing()
        {
            AZ_Assert(false, "This Lidar Implementation does not support PointCloud publishing!");
            return false;
        }

    protected:
        ~LidarRaycasterRequests() = default;

        static void ValidateRayOrientations([[maybe_unused]] const AZStd::vector<AZ::Vector3>& orientations)
        {
            AZ_Assert(!orientations.empty(), "Provided ray orientations were of incorrect value: Ray orientations must not be empty.")
        }
    };

    class LidarRaycasterBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = LidarId;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using LidarRaycasterRequestBus = AZ::EBus<LidarRaycasterRequests, LidarRaycasterBusTraits>;
} // namespace ROS2Sensors

// Since we want to use the LidarId type as a Bus Id type,
// we need to create a specialization for the hash template operator.
namespace AZStd
{
    // hash specialization
    template<>
    struct hash<ROS2Sensors::LidarId>
    {
        constexpr size_t operator()(const ROS2Sensors::LidarId& id) const
        {
            return id.GetHash();
        }
    };
} // namespace AZStd
