/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/Outcome/Outcome.h"
#include "SimulationInterfaces/Resource.h"
#include "SimulationInterfaces/Result.h"
#include "SimulationInterfaces/TagFilter.h"
#include "SimulationInterfacesTypeIds.h"
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include "WorldResource.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/unordered_set.h>
#include <simulation_interfaces/msg/simulator_features.hpp>

namespace SimulationInterfaces
{
    //! @see <a
    //! href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetAvailableWorlds.srv">GetAvailableWorlds.srv</a>
    struct GetWorldsRequest
    {
        // Optional sources (local or remote) where simulator should look for worlds
        AZStd::vector<AZStd::string> additionalSources;
        TagFilter filter; // A filter to match against world tags
        bool offlineOnly = false; // Whether to limit search to only offline resources
        bool continueOnError = false; // By default fail in case of missing asset
    };

    //! @see <a
    //! href="https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/LoadWorld.srv">LoadWorld.srv</a>
    struct LoadWorldRequest
    {
        Resource levelResource; // which level needs to be loaded
        bool failOnUnsupportedElement = false; // By default just skip failed elements
        bool ignoreMissingOrUnsupportedAssets = false; // Whether search should be continue on some noncritical fails
    };

    using WorldResourcesList = AZStd::vector<WorldResource>;

    class LevelManagerRequests
    {
    public:
        AZ_RTTI(LevelManagerRequests, LevelManagerRequestsTypeId);
        virtual ~LevelManagerRequests() = default;

        virtual AZ::Outcome<WorldResourcesList, FailedResult> GetAvailableWorlds(const GetWorldsRequest& request) = 0;
        virtual AZ::Outcome<WorldResource, FailedResult> GetCurrentWorld() = 0;
        virtual AZ::Outcome<WorldResource, FailedResult> LoadWorld(const LoadWorldRequest& request) = 0;
        virtual AZ::Outcome<void, FailedResult> UnloadWorld() = 0;
    };

    class LevelManagerRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using LevelManagerRequestBus = AZ::EBus<LevelManagerRequests, LevelManagerRequestBusTraits>;
    using LevelManagerRequestBusInterface = AZ::Interface<LevelManagerRequests>;

} // namespace SimulationInterfaces
