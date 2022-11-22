/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "LidarRaycaster.h"
#include "LidarTemplateUtils.h"
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>

namespace ROS2
{
    LidarRaycaster::LidarRaycaster(const LidarRaycasterRequestBus::BusIdType& busId)
        : m_uuid{ busId }
    {
        ROS2::LidarRaycasterRequestBus::Handler::BusConnect(busId);
    }

    LidarRaycaster::LidarRaycaster(LidarRaycaster&& lidarRaycaster) noexcept
        : m_uuid{ lidarRaycaster.m_uuid }
        , m_sceneHandle{ lidarRaycaster.m_sceneHandle }
        , m_range{ lidarRaycaster.m_range }
        , m_addMaxRangePoints{ lidarRaycaster.m_addMaxRangePoints }
        , m_rayRotations{ AZStd::move(lidarRaycaster.m_rayRotations) }
        , m_ignoreLayer{ lidarRaycaster.m_ignoreLayer }
        , m_ignoredLayerIndex{ lidarRaycaster.m_ignoredLayerIndex }
    {
        lidarRaycaster.BusDisconnect();
        lidarRaycaster.m_uuid = AZ::Uuid::CreateNull();

        ROS2::LidarRaycasterRequestBus::Handler::BusConnect(m_uuid);
    }

    LidarRaycaster::~LidarRaycaster()
    {
        if (!m_uuid.IsNull())
        {
            ROS2::LidarRaycasterRequestBus::Handler::BusDisconnect();
        }
    }

    void LidarRaycaster::SetRaycasterScene(const AzPhysics::SceneHandle& handle)
    {
        m_sceneHandle = handle;
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

    // A simplified, non-optimized first version. TODO - generalize results (fields)
    AZStd::vector<AZ::Vector3> LidarRaycaster::PerformRaycast(const AZ::Transform& lidarTransform)
    {
        AZ_Assert(!m_rayRotations.empty(), "Ray poses are not configured. Unable to Perform a raycast.");
        AZ_Assert(m_range > 0.0f, "Ray range is not configured. Unable to Perform a raycast.");

        AZStd::vector<AZ::Vector3> results;
        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            AZ_Warning("LidarRaycaster", false, "No valid scene handle");
            return results;
        }

        AZStd::vector<AZ::Vector3> rayDirections =
            LidarTemplateUtils::RotationsToDirections(m_rayRotations, lidarTransform.GetEulerRadians());

        AZ::Vector3 lidarPosition = lidarTransform.GetTranslation();

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
            request->m_filterCallback = [this](const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
            {
                if (m_ignoreLayer && (shape->GetCollisionLayer().GetIndex() == m_ignoredLayerIndex))
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
                AZ::Vector3 maxPoint = lidarTransform.TransformPoint(rayDirections[i] * m_range);
                results.push_back(maxPoint);
            }
        }
        return results;
    }

    void LidarRaycaster::ConfigureLayerIgnoring(bool ignoreLayer, unsigned int layerIndex)
    {
        m_ignoreLayer = ignoreLayer;
        m_ignoredLayerIndex = layerIndex;
    }
    void LidarRaycaster::ConfigureMaxRangePointAddition(bool addMaxRangePoints)
    {
        m_addMaxRangePoints = addMaxRangePoints;
    }
} // namespace ROS2
