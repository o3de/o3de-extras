/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "LidarRaycaster.h"
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Shape.h>

namespace ROS2
{
    void LidarRaycaster::SetRaycasterScene(const AzPhysics::SceneHandle& handle)
    {
        m_sceneHandle = handle;
    }

    // A simplified, non-optimized first version. TODO - generalize results (fields)
    AZStd::vector<AZ::Vector3> LidarRaycaster::PerformRaycast(
        const AZ::Vector3& start,
        const AZStd::vector<AZ::Vector3>& directions,
        const AZ::Transform& globalToLidarTM,
        float distance,
        bool ignoreLayer,
        unsigned int ignoredLayerIndex)
    {
        AZStd::vector<AZ::Vector3> results;
        if (m_sceneHandle == AzPhysics::InvalidSceneHandle)
        {
            AZ_Warning("LidarRaycaster", false, "No valid scene handle");
            return results;
        }

        AzPhysics::SceneQueryRequests requests;
        requests.reserve(directions.size());
        results.reserve(directions.size());
        for (const AZ::Vector3& direction : directions)
        { // NOTE - performance-wise, consider reusing requests
            AZStd::shared_ptr<AzPhysics::RayCastRequest> request = AZStd::make_shared<AzPhysics::RayCastRequest>();
            request->m_start = start;
            request->m_direction = direction;
            request->m_distance = distance;
            request->m_reportMultipleHits = false;
            request->m_filterCallback =
                [ignoreLayer, ignoredLayerIndex](const AzPhysics::SimulatedBody* simBody, const Physics::Shape* shape)
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
        AZ_Assert(requestResults.size() == directions.size(), "request size should be equal to directions size");
        for (int i = 0; i < requestResults.size(); i++)
        { // TODO - check flag for SceneQuery::ResultFlags::Position
            const auto& requestResult = requestResults[i];
            if (requestResult.m_hits.size() > 0)
            {
                auto globalHitPoint = requestResult.m_hits[0].m_position;
                results.push_back(globalToLidarTM.TransformPoint(globalHitPoint)); // Transform back to local frame
            }
            else if (m_addPointsMaxRange)
            {
                results.push_back(globalToLidarTM.TransformPoint(start + directions[i] * distance));
            }
        }
        return results;
    }

    void LidarRaycaster::SetAddPointsMaxRange(bool addPointsMaxRange)
    {
        m_addPointsMaxRange = addPointsMaxRange;
    }

} // namespace ROS2
