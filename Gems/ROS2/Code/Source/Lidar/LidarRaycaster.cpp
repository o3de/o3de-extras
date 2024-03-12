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
        , m_minRange{ lidarRaycaster.m_minRange }
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
        m_rayRotations = orientations;
    }

    void LidarRaycaster::ConfigureRayRange(float range)
    {
        ValidateRayRange(range);
        m_range = range;
    }

    void LidarRaycaster::ConfigureMinimumRayRange(float range)
    {
        m_minRange = range;
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
            request->m_distance = m_range;
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

    RaycastResult LidarRaycaster::PerformRaycast(const AZ::Transform& lidarTransform)
    {
        AZ_Assert(!m_rayRotations.empty(), "Ray poses are not configured. Unable to Perform a raycast.");
        AZ_Assert(m_range > 0.0f, "Ray range is not configured. Unable to Perform a raycast.");

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(m_sceneEntityId);
        }

        const AZStd::vector<AZ::Vector3> rayDirections = LidarTemplateUtils::RotationsToDirections(m_rayRotations, lidarTransform);
        AzPhysics::SceneQueryRequests requests = prepareRequests(lidarTransform, rayDirections);

        RaycastResult results;
        const bool handlePoints = (m_resultFlags & RaycastResultFlags::Points) == RaycastResultFlags::Points;
        const bool handleRanges = (m_resultFlags & RaycastResultFlags::Ranges) == RaycastResultFlags::Ranges;
        if (handlePoints)
        {
            results.m_points.reserve(rayDirections.size());
        }
        if (handleRanges)
        {
            results.m_ranges.reserve(rayDirections.size());
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto requestResults = sceneInterface->QuerySceneBatch(m_sceneHandle, requests);
        AZ_Assert(requestResults.size() == rayDirections.size(), "Request size should be equal to directions size");
        const auto localTransform =
            AZ::Transform::CreateFromQuaternionAndTranslation(lidarTransform.GetRotation(), lidarTransform.GetTranslation()).GetInverse();
        const float maxRange = m_addMaxRangePoints ? m_range : AZStd::numeric_limits<float>::infinity();

        for (int i = 0; i < requestResults.size(); i++)
        {
            const auto& requestResult = requestResults[i];
            float hitRange = requestResult ? requestResult.m_hits[0].m_distance : maxRange;
            if (hitRange < m_minRange)
            {
                hitRange = -AZStd::numeric_limits<float>::infinity();
            }
            if (handleRanges)
            {
                results.m_ranges.push_back(hitRange);
            }
            if (handlePoints)
            {
                if (hitRange == maxRange)
                {
                    // to properly visualize max points they need to be transformed to local coordinate system before applying maxRange
                    const AZ::Vector3 maxPoint = lidarTransform.TransformPoint(localTransform.TransformVector(rayDirections[i]) * hitRange);
                    results.m_points.push_back(maxPoint);
                }
                else if (!AZStd::isinf(hitRange))
                {
                    // otherwise they are already calculated by PhysX
                    results.m_points.push_back(requestResult.m_hits[0].m_position);
                }
            }
        }

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
