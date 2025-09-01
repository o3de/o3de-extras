/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <SimulationInterfaces/Bounds.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/TagFilter.h>
#include <SimulationInterfaces/WorldResource.h>
#include <simulation_interfaces/msg/world_resource.hpp>

namespace SimulationInterfaces::Utils
{
    //! Convert a relative path to a URI
    //! relative path: "path/to/file.txt"
    //! URI: "product_asset:///path/to/file.txt"
    AZStd::string RelPathToUri(AZStd::string_view relPath);
    AZStd::string UriToRelPath(AZStd::string_view relPath);

    //! Filter named poses given by map by given tag filter
    //! @param entitiesToFilter map [entityName,entityId] describing NamedPoses which should be processed
    //! @param tagFilter definition of tag filter to apply
    AZStd::unordered_map<AZStd::string, AZ::EntityId> FilterNamedPosesByTag(
        const AZStd::unordered_map<AZStd::string, AZ::EntityId>& entitiesToFilter, const TagFilter& tagFilter);

    //! Helper function to check if entity tags matcher with given filter
    //! @param tagFilter filter defined by simulation interfaces with rules of the tag matching
    //! @param entityTags tags assigned to entity which is being tested
    //! @return status of tag matching
    bool AreTagsMatching(const TagFilter& tagFilter, const AZStd::vector<AZStd::string>& entityTags);

    //! Helper function to retrieve simulated body from the entity byt given ID
    //! @param entityId entity ID of the entity with potential simulated body
    //! @return pointer to simulated body, or failure in case when simulated body wasn't found
    AZ::Outcome<AzPhysics::SimulatedBody*, AZStd::string> GetSimulatedBody(AZ::EntityId entityId);

    //! Helper function which converts collider to Bounds defined by the simulation interfaces
    //! @param shape physics shape you want to convert
    //! @param entityId id of the physical shape owner
    //! @return simulation interfaces style bound or failure in something was wrong during processing
    AZ::Outcome<Bounds, AZStd::string> ConvertPhysicalShapeToBounds(AZStd::shared_ptr<Physics::Shape> shape, const AZ::EntityId& entityId);

    //! Helper function which converts ROS 2 request with entity Filter to structure defined by SimulationInterfaces Gem
    //! @param request ROS 2 request with field `filters`
    //! @return EntityFilter structure or error message if something went wrong
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
            // https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/Bounds.msg
            // Axis-aligned bounding box, points field should have two values, which are upper right and lower left corners of the box.
            if (p1.x < p2.x || p1.y < p2.y || p1.z < p2.z)
            {
                return AZ::Failure("Invalid points! The first point should be higher than the second point.");
            }
            const auto max = ROS2::ROS2Conversions::FromROS2Vector3(p1);
            const auto min = ROS2::ROS2Conversions::FromROS2Vector3(p2);
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

        // copy categories
        AZStd::transform(
            request.filters.categories.begin(),
            request.filters.categories.end(),
            AZStd::back_inserter(filter.m_entityCategories),
            [](const simulation_interfaces::msg::EntityCategory& category)
            {
                return category.category;
            });
        // copy tags
        filter.m_tagsFilter.m_mode = request.filters.tags.filter_mode;
        AZStd::transform(
            request.filters.tags.tags.begin(),
            request.filters.tags.tags.end(),
            AZStd::inserter(filter.m_tagsFilter.m_tags, filter.m_tagsFilter.m_tags.end()),
            [](const std::string& tag)
            {
                return tag.c_str();
            });
        return AZ::Success(AZStd::move(filter));
    }

    //! Helper function to convert WorldResource defined by SimulationInterfaces Gem to structure defined by ROS 2
    //! @param resource data to convert
    //! @return ROS 2 version of WorldResource
    simulation_interfaces::msg::WorldResource ConvertToRos2WorldResource(const SimulationInterfaces::WorldResource& resource);

} // namespace SimulationInterfaces::Utils
