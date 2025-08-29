/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CommonUtilities.h"
#include "Components/NamedPoseComponent.h"
#include "SimulationInterfaces/NamedPoseManagerRequestBus.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Shape.h>
#include <simulation_interfaces/msg/tags_filter.hpp>

namespace SimulationInterfaces::Utils
{
    const char* const ProductAssetPrefix = "product_asset:///";
    AZStd::string RelPathToUri(AZStd::string_view relPath)
    {
        AZStd::string uri = relPath;
        AZStd::replace(uri.begin(), uri.end(), '\\', '/');
        uri.insert(0, ProductAssetPrefix);
        return uri;
    }

    AZStd::string UriToRelPath(AZStd::string_view uri)
    {
        if (uri.starts_with(ProductAssetPrefix))
        {
            const AZStd::string_view productAssetPrefix{ ProductAssetPrefix };
            return uri.substr(productAssetPrefix.length());
        }
        return {};
    }

    bool AreTagsMatching(const TagFilter& tagFilter, const AZStd::vector<AZStd::string>& entityTags)
    {
        if (tagFilter.m_tags.empty())
        {
            return true;
        }

        bool matchAllTags = tagFilter.m_mode == simulation_interfaces::msg::TagsFilter::FILTER_MODE_ALL;
        for (auto& tag : tagFilter.m_tags)
        {
            bool tagExistInEntity = AZStd::find(entityTags.begin(), entityTags.end(), tag) != entityTags.end();
            // if all tags need to match but entity doesn't have requested one, return with false
            if (matchAllTags && !tagExistInEntity)
            {
                return false;
            }
            // if match any and first match found, condition already satisfied, return with true
            if (!matchAllTags && tagExistInEntity)
            {
                return true;
            }
        }
        // if code goes here it means it went through whole loop, In MATCH_ALL mode it means all tags were found => return true
        // In MATCH_ANY it means no match was found, return false
        // this logical AND handles these cases
        return matchAllTags && true;
    }

    AZStd::unordered_map<AZStd::string, AZ::EntityId> FilterNamedPosesByTag(
        const AZStd::unordered_map<AZStd::string, AZ::EntityId>& entitiesToFilter, const TagFilter& tagFilter)
    {
        AZStd::unordered_map<AZStd::string, AZ::EntityId> filteredEntities;
        for (auto& [name, entityId] : entitiesToFilter)
        {
            NamedPose configuration;
            NamedPoseComponentRequestBus::EventResult(configuration, entityId, &NamedPoseComponentRequests::GetConfiguration);
            if (AreTagsMatching(tagFilter, configuration.m_tags))
            {
                filteredEntities[name] = entityId;
            }
        }
        return filteredEntities;
    }

    AZ::Outcome<AzPhysics::SimulatedBody*, AZStd::string> GetSimulatedBody(AZ::EntityId entityId)
    {
        AzPhysics::SimulatedBody* simulatedBody = nullptr;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            simulatedBody, entityId, &AzPhysics::SimulatedBodyComponentRequests::GetSimulatedBody);
        if (simulatedBody == nullptr)
        {
            const auto msg = AZStd::string::format("Entity's simulated body doesn't exist");
            return AZ::Failure(msg);
        }
        if (simulatedBody->m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
        {
            const auto msg = AZStd::string::format("Entity is not a valid simulated body");
            return AZ::Failure(msg);
        }
        return AZ::Success(simulatedBody);
    }

    AZ::Outcome<Bounds, AZStd::string> ConvertPhysicalShapeToBounds(AZStd::shared_ptr<Physics::Shape> shape, const AZ::EntityId& entityId)
    {
        auto config = shape->GetShapeConfiguration();
        auto shapeType = config->GetShapeType();

        // get final collider transform including entity TM and offsets
        auto [colliderOffsetTranslation, collideerOffsetRotation] = shape->GetLocalPose();
        AZ::Transform offsetTransform =
            AZ::Transform::CreateFromQuaternionAndTranslation(collideerOffsetRotation, colliderOffsetTranslation);
        AZ::Transform entityTransform;
        AZ::TransformBus::EventResult(entityTransform, entityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Transform colliderAbsoluteTransform = entityTransform * offsetTransform;

        Bounds bounds;
        switch (shapeType)
        {
        case Physics::ShapeType::Box:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_BOX;

                auto boxConfig = azdynamic_cast<Physics::BoxShapeConfiguration*>(config);
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation() + (boxConfig->m_dimensions / 2)); // upper Right
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation() - (boxConfig->m_dimensions / 2)); // bottom left
                return bounds;
            }
        case Physics::ShapeType::Sphere:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_SPHERE;
                auto sphereConfig = azdynamic_cast<Physics::SphereShapeConfiguration*>(config);
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation()); // sphere center
                bounds.m_points.emplace_back(sphereConfig->m_radius, 0.f, 0.f); // radius and two ignored fields
                return bounds;
            }
        // this type of collider is currently unsupported by the PhysX engine, but this implementation uses provided abstractions and is
        // independent from selected physics engine.
        case Physics::ShapeType::ConvexHull:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_CONVEX_HULL;
                AZStd::vector<AZ::Vector3> vertices;
                AZStd::vector<AZ::u32> indices;
                shape->GetGeometry(vertices, indices);
                for (auto& vertex : vertices)
                {
                    bounds.m_points.emplace_back(colliderAbsoluteTransform.TransformPoint(vertex));
                }
                return bounds;
            }
        default:
            {
                return AZ::Failure(AZStd::string::format(
                    "Passed shape type with id %d is not supported by simulation interfaces", static_cast<AZ::u8>(shapeType)));
            }
        }
    }
} // namespace SimulationInterfaces::Utils
