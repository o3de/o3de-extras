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
#include <SimulationInterfaces/TagFilter.h>

namespace SimulationInterfaces::Utils
{
    //! Convert a relative path to a URI
    //! relative path: "path/to/file.txt"
    //! URI: "product_asset:///path/to/file.txt"
    AZStd::string RelPathToUri(AZStd::string_view relPath);
    AZStd::string UriToRelPath(AZStd::string_view relPath);

    //! Filter given map by given tag
    //! @param entitiesToFilter map [entityName,entityId] which should be processed
    //! @param tagFilter definition of tag filter to apply
    AZStd::unordered_map<AZStd::string, AZ::EntityId> FilterEntitiesByTag(
        const AZStd::unordered_map<AZStd::string, AZ::EntityId>& entitiesToFilter, const TagFilter& tagFilter);

    //! Helper function to check if entity tags matcher with given filter
    bool AreTagsMatching(const TagFilter& tagFilter, const AZStd::vector<AZStd::string>& entityTags);

    AZ::Outcome<AzPhysics::SimulatedBody*, AZStd::string> GetSimulatedBody(AZ::EntityId entityId);

} // namespace SimulationInterfaces::Utils
