/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CommonUtilities.h"
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

    bool AreTagsMatching(const TagFilter& tagFilter, const LmbrCentral::Tags& entityTags)
    {
        if (tagFilter.m_tags.empty())
        {
            return true;
        }

        bool matchAllTags = tagFilter.m_mode == simulation_interfaces::msg::TagsFilter::FILTER_MODE_ALL;
        for (auto& tag : tagFilter.m_tags)
        {
            bool tagExistInEntity = entityTags.contains(LmbrCentral::Tag(tag));
            // if all tags needs to match but entity doesn't have requested one, return with false
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

    AZStd::unordered_map<AZStd::string, AZ::EntityId> FilterEntitiesByTag(
        const AZStd::unordered_map<AZStd::string, AZ::EntityId>& entitiesToFilter, const TagFilter& tagFilter)
    {
        AZStd::unordered_map<AZStd::string, AZ::EntityId> filteredEntities;
        for (auto& [name, entityId] : entitiesToFilter)
        {
            LmbrCentral::Tags tags;
            LmbrCentral::TagComponentRequestBus::EventResult(tags, entityId, &LmbrCentral::TagComponentRequestBus::Events::GetTags);
            if (AreTagsMatching(tagFilter, tags))
            {
                filteredEntities[name] = entityId;
            }
        }
        return filteredEntities;
    }
} // namespace SimulationInterfaces::Utils
