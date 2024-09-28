/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>
#include <Lidar/LidarRaycaster.h>
#include <Lidar/LidarTemplateUtils.h>
#include <ROS2/Lidar/SegmentationUtils.h>

namespace ROS2
{
    static AzPhysics::SceneHandle GetPhysicsSceneFromEntityId(const AZ::EntityId& entityId)
    {
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        auto foundBody = physicsSystem->FindAttachedBodyHandleFromEntityId(entityId);
        AzPhysics::SceneHandle lidarPhysicsSceneHandle = foundBody.first;
        if (foundBody.first == AzPhysics::InvalidSceneHandle)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            lidarPhysicsSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        }

        AZ_Assert(lidarPhysicsSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid physics scene handle for entity");
        return lidarPhysicsSceneHandle;
    }

    LidarRaycaster::LidarRaycaster(LidarId busId, AZ::EntityId sceneEntityId)
        : m_busId{ busId }
        , m_sceneEntityId{ sceneEntityId }
    {
        ROS2::LidarRaycasterRequestBus::Handler::BusConnect(busId);
    }

    LidarRaycaster::LidarRaycaster(LidarRaycaster&& lidarRaycaster)
        : m_busId{ lidarRaycaster.m_busId }
        , m_sceneEntityId{ lidarRaycaster.m_sceneEntityId }
        , m_sceneHandle{ lidarRaycaster.m_sceneHandle }
        , m_resultFlags{ lidarRaycaster.m_resultFlags }
        , m_range{ lidarRaycaster.m_range }
        , m_addMaxRangePoints{ lidarRaycaster.m_addMaxRangePoints }
        , m_rayRotations{ AZStd::move(lidarRaycaster.m_rayRotations) }
        , m_ignoredCollisionLayers{ lidarRaycaster.m_ignoredCollisionLayers }
    {
        lidarRaycaster.BusDisconnect();
        lidarRaycaster.m_busId = LidarId::CreateNull();

        ROS2::LidarRaycasterRequestBus::Handler::BusConnect(m_busId);
    }

    LidarRaycaster::~LidarRaycaster()
    {
        ROS2::LidarRaycasterRequestBus::Handler::BusDisconnect();
    }

    void LidarRaycaster::ConfigureRayOrientations(const AZStd::vector<AZ::Vector3>& orientations)
    {
        ValidateRayOrientations(orientations);

        m_rayRotations.reserve(orientations.size());
        for (const auto& angle : orientations)
        {
            m_rayRotations.emplace_back(AZ::Quaternion::CreateFromEulerRadiansZYX({ 0.0f, -angle.GetY(), angle.GetZ() }));
        }
    }

    void LidarRaycaster::ConfigureRayRange(RayRange range)
    {
        m_range = range;
    }

    void LidarRaycaster::ConfigureRaycastResultFlags(RaycastResultFlags flags)
    {
        m_resultFlags = flags;
    }

    AzPhysics::SceneQueryRequests LidarRaycaster::prepareRequests(
        const AZ::Transform& lidarTransform, const AZStd::vector<AZ::Vector3>& rayDirections) const
    {
        using AzPhysics::SceneQuery::HitFlags;
        const AZ::Vector3& lidarPosition = lidarTransform.GetTranslation();

        AzPhysics::SceneQueryRequests requests;
        requests.reserve(rayDirections.size());
        for (const AZ::Vector3& direction : rayDirections)
        {
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = lidarPosition;
            request->m_direction = direction;
            request->m_distance = m_range->m_max;
            request->m_reportMultipleHits = false;

            request->m_filterCallback = [ignoredCollisionLayers = this->m_ignoredCollisionLayers](
                                            const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
            {
                if (ignoredCollisionLayers.contains(shape->GetCollisionLayer().GetIndex()))
                {
                    return AzPhysics::SceneQuery::QueryHitType::None;
                }
                return AzPhysics::SceneQuery::QueryHitType::Block;
            };

            requests.emplace_back(AZStd::move(request));
        }
        return requests;
    }

    uint8_t LidarRaycaster::GetClassIdForEntity(AZ::EntityId entityId)
    {
        if (auto it = m_entityIdToClassIdCache.find(entityId); it != m_entityIdToClassIdCache.end())
        {
            return it->second;
        }

        const uint8_t classId = SegmentationUtils::FetchClassIdForEntity(entityId);
        m_entityIdToClassIdCache.emplace(entityId, classId);
        return classId;
    }

    AZ::Outcome<RaycastResults, const char*> LidarRaycaster::PerformRaycast(const AZ::Transform& lidarTransform)
    {
        AZ_Assert(!m_rayRotations.empty(), "Ray poses are not configured. Unable to Perform a raycast.");
        AZ_Assert(m_range.has_value(), "Ray range is not configured. Unable to Perform a raycast.");

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(m_sceneEntityId);
        }

        const AZStd::vector<AZ::Vector3> rayDirections = LidarTemplateUtils::RotationsToDirections(m_rayRotations, lidarTransform);
        AzPhysics::SceneQueryRequests requests = prepareRequests(lidarTransform, rayDirections);

        const bool handlePoints = (m_resultFlags & RaycastResultFlags::Point) == RaycastResultFlags::Point;
        const bool handleRanges = (m_resultFlags & RaycastResultFlags::Range) == RaycastResultFlags::Range;
        const bool handleSegmentation = (m_resultFlags & RaycastResultFlags::SegmentationData) == RaycastResultFlags::SegmentationData;
        RaycastResults results(m_resultFlags, rayDirections.size());

        AZStd::optional<RaycastResults::FieldSpan<RaycastResultFlags::Point>::iterator> pointIt;
        AZStd::optional<RaycastResults::FieldSpan<RaycastResultFlags::Range>::iterator> rangeIt;
        AZStd::optional<RaycastResults::FieldSpan<RaycastResultFlags::SegmentationData>::iterator> segmentationIt;
        if (handlePoints)
        {
            pointIt = results.GetFieldSpan<RaycastResultFlags::Point>().value().begin();
        }
        if (handleRanges)
        {
            rangeIt = results.GetFieldSpan<RaycastResultFlags::Range>().value().begin();
        }
        if (handleSegmentation)
        {
            segmentationIt = results.GetFieldSpan<RaycastResultFlags::SegmentationData>().value().begin();
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto requestResults = sceneInterface->QuerySceneBatch(m_sceneHandle, requests);
        AZ_Assert(requestResults.size() == rayDirections.size(), "Request size should be equal to directions size");
        const auto localTransform =
            AZ::Transform::CreateFromQuaternionAndTranslation(lidarTransform.GetRotation(), lidarTransform.GetTranslation()).GetInverse();
        const float maxRange = m_addMaxRangePoints ? m_range->m_max : AZStd::numeric_limits<float>::infinity();

        size_t usedSize = 0U;
        for (size_t i = 0U; i < requestResults.size(); i++)
        {
            const auto& requestResult = requestResults[i];
            float hitRange = requestResult ? requestResult.m_hits[0].m_distance : maxRange;
            if (hitRange < m_range->m_min)
            {
                hitRange = -AZStd::numeric_limits<float>::infinity();
            }

            bool wasUsed = false;
            if (rangeIt.has_value())
            {
                *rangeIt.value() = hitRange;
                wasUsed = true;
            }

            if (pointIt.has_value())
            {
                if (hitRange == maxRange)
                {
                    // to properly visualize max points they need to be transformed to local coordinate system before applying maxRange
                    const AZ::Vector3 maxPoint = lidarTransform.TransformPoint(localTransform.TransformVector(rayDirections[i]) * hitRange);
                    *pointIt.value() = maxPoint;
                    wasUsed = true;
                }
                else if (!AZStd::isinf(hitRange))
                {
                    // otherwise they are already calculated by PhysX
                    *pointIt.value() = requestResult.m_hits[0].m_position;
                    wasUsed = true;
                }
            }

            if (segmentationIt.has_value())
            {
                segmentationIt.value()->m_classId = 0;
                segmentationIt.value()->m_entityId = 0;

                if (requestResult)
                {
                    const auto entityId = requestResult.m_hits[0].m_entityId;
                    const uint8_t classId = GetClassIdForEntity(entityId);
                    // We have to map existing entity IDs from the original 64 bits onto 32 bits.
                    // This may result in collisions but the chances are slim.
                    const int32_t compressedEntityId =
                        (aznumeric_cast<AZ::u64>(entityId) >> 32) ^ (aznumeric_cast<AZ::u64>(entityId) & 0xFFFFFFFF);

                    segmentationIt.value()->m_classId = classId;
                    segmentationIt.value()->m_entityId = compressedEntityId;
                }

                wasUsed = true;
            }

            if (wasUsed)
            {
                if (rangeIt.has_value())
                {
                    ++rangeIt.value();
                }

                if (pointIt.has_value())
                {
                    ++pointIt.value();
                }

                if (segmentationIt.has_value())
                {
                    ++segmentationIt.value();
                }

                ++usedSize;
            }
        }
        results.Resize(usedSize);

        return results;
    }

    void LidarRaycaster::ConfigureIgnoredCollisionLayers(const AZStd::unordered_set<AZ::u32>& layerIndices)
    {
        m_ignoredCollisionLayers = layerIndices;
    }
    void LidarRaycaster::ConfigureMaxRangePointAddition(bool addMaxRangePoints)
    {
        m_addMaxRangePoints = addMaxRangePoints;
    }
} // namespace ROS2
