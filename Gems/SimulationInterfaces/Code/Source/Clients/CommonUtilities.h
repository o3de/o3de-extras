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
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <SimulationInterfaces/Bounds.h>
#include <SimulationInterfaces/TagFilter.h>

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

    //! Helper method which converts collider to Bounds defined by the simulation interfaces
    //! @param shape physics shape you want to convert
    //! @param entityId id of the physical shape owner
    //! @return simulation interfaces style bound or failure in something was wrong during processing
    AZ::Outcome<Bounds, AZStd::string> ConvertPhysicalShapeToBounds(AZStd::shared_ptr<Physics::Shape> shape, const AZ::EntityId& entityId);

} // namespace SimulationInterfaces::Utils
