/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/smart_ptr/make_shared.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <simulation_interfaces/msg/bounds.hpp>

namespace ROS2SimulationInterfaces::Utils
{
    template<typename RequestType>
    AZ::Outcome<SimulationInterfaces::EntityFilters, AZStd::string> GetEntityFiltersFromRequest(const RequestType& request)
    {
        SimulationInterfaces::EntityFilters filter;
        filter.m_nameFilter = request.filters.filter.c_str();
        uint8_t type = request.filters.bounds.type;
        if (type == simulation_interfaces::msg::Bounds::TYPE_BOX)
        {
            if (request.filters.bounds.points.size() != 2)
            {
                return AZ::Failure("Invalid number of points! Type 'TYPE_BOX' should have exactly 2 points.");
            }
            const auto& p1 = request.filters.bounds.points.front();
            const auto& p2 = request.filters.bounds.points.back();
            if (p1.x > p2.x || p1.y > p2.y || p1.z > p2.z)
            {
                return AZ::Failure("Invalid points! The first point should be lower than the second point.");
            }
            const auto min = ROS2::ROS2Conversions::FromROS2Vector3(p1);
            const auto max = ROS2::ROS2Conversions::FromROS2Vector3(p2);
            const AZ::Aabb aabb = AZ::Aabb::CreateFromMinMax(min, max);
            filter.m_boundsShape = AZStd::make_shared<Physics::BoxShapeConfiguration>(aabb.GetExtents());
        }
        else if (type == simulation_interfaces::msg::Bounds::TYPE_CONVEX_HULL)
        {
            if (request.filters.bounds.points.size() < 3)
            {
                return AZ::Failure("Invalid number of points! Type 'TYPE_CONVEX_HULL' should have at least 3 points.");
            }
            // TODO: Implement convex hull shape configuration
            // filter.m_boundsShape = AZStd::make_shared<Physics::ConvexHullShapeConfiguration>();
            return AZ::Failure("Unsupported type! Type 'TYPE_CONVEX_HULL' is not supported.");
        }
        else if (type == simulation_interfaces::msg::Bounds::TYPE_SPHERE)
        {
            if (request.filters.bounds.points.size() != 2)
            {
                return AZ::Failure("Invalid number of points! Type 'TYPE_SPHERE' should have exactly 2 points.");
            }
            filter.m_boundsShape = AZStd::make_shared<Physics::SphereShapeConfiguration>(request.filters.bounds.points.back().x);
            filter.m_boundsPose = { ROS2::ROS2Conversions::FromROS2Vector3(request.filters.bounds.points.front()),
                                    AZ::Quaternion::CreateIdentity(),
                                    1.0f };
        }
        return AZ::Success(AZStd::move(filter));
    }
} // namespace ROS2SimulationInterfaces::Utils
