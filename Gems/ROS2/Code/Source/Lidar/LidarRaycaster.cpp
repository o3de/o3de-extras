/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
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
        , m_range{ lidarRaycaster.m_range }
        , m_addMaxRangePoints{ lidarRaycaster.m_addMaxRangePoints }
        , m_rayRotations{ AZStd::move(lidarRaycaster.m_rayRotations) }
        , m_ignoreLayer{ lidarRaycaster.m_ignoreLayer }
        , m_ignoredLayerIndex{ lidarRaycaster.m_ignoredLayerIndex }
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

    AZStd::vector<AZ::Vector3> LidarRaycaster::PerformRaycast(const AZ::Transform& lidarTransform)
    {
        AZ_Assert(!m_rayRotations.empty(), "Ray poses are not configured. Unable to Perform a raycast.");
        AZ_Assert(m_range > 0.0f, "Ray range is not configured. Unable to Perform a raycast.");

        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            m_sceneHandle = GetPhysicsSceneFromEntityId(m_sceneEntityId);
        }

        const AZStd::vector<AZ::Vector3> rayDirections =
            LidarTemplateUtils::RotationsToDirections(m_rayRotations, lidarTransform.GetEulerRadians());

        const AZ::Vector3 lidarPosition = lidarTransform.GetTranslation();

        AZStd::vector<AZ::Vector3> results;
        AzPhysics::SceneQueryRequests requests;
        requests.reserve(rayDirections.size());
        results.reserve(rayDirections.size());
        for (const AZ::Vector3& direction : rayDirections)
        {
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = lidarPosition;
            request->m_direction = direction;
            request->m_distance = m_range;
            request->m_reportMultipleHits = false;
            request->m_filterCallback = [ignoredLayerIndex = this->m_ignoredLayerIndex, ignoreLayer = this->m_ignoreLayer](
                                            const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
            {
                if (ignoreLayer && (shape->GetCollisionLayer().GetIndex() == ignoredLayerIndex))
                {
                    return AzPhysics::SceneQuery::QueryHitType::None;
                }
                else
                {
                    return AzPhysics::SceneQuery::QueryHitType::Block;
                }
            };
            requests.emplace_back(AZStd::move(request));
        }

        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        auto requestResults = sceneInterface->QuerySceneBatch(m_sceneHandle, requests);
        AZ_Assert(requestResults.size() == rayDirections.size(), "Request size should be equal to directions size");
        for (int i = 0; i < requestResults.size(); i++)
        {
            const auto& requestResult = requestResults[i];
            if (!requestResult.m_hits.empty())
            {
                results.push_back(requestResult.m_hits[0].m_position);
            }
            else if (m_addMaxRangePoints)
            {
                const AZ::Vector3 maxPoint = lidarTransform.TransformPoint(rayDirections[i] * m_range);
                results.push_back(maxPoint);
            }
        }
        return results;
    }

    void LidarRaycaster::ConfigureLayerIgnoring(bool ignoreLayer, AZ::u32 layerIndex)
    {
        m_ignoreLayer = ignoreLayer;
        m_ignoredLayerIndex = layerIndex;
    }
    void LidarRaycaster::ConfigureMaxRangePointAddition(bool addMaxRangePoints)
    {
        m_addMaxRangePoints = addMaxRangePoints;
    }
} // namespace ROS2
