/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/Outcome/Outcome.h"
#include "SimulationInterfacesTypeIds.h"
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>

#include "Bounds.h"
#include "NamedPose.h"
#include "Result.h"
#include "TagFilter.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/containers/vector.h>

namespace SimulationInterfaces
{
    using NamedPoseSet = AZStd::vector<NamedPose>;

    class NamedPoseManagerRequests
    {
    public:
        AZ_RTTI(NamedPoseManagerRequests, SimulationFeaturesAggregatorRequestsTypeId);
        virtual ~NamedPoseManagerRequests() = default;

        //! Create named pose with given parameters
        virtual AZ::Outcome<void, FailedResult> CreateNamedPose(NamedPose namedPose) = 0;

        //! Get named poses matching given TagFiler
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/TagsFilter.msg
        virtual AZ::Outcome<NamedPoseSet, FailedResult> GetNamedPoses(const TagFilter& tags) = 0;

        //! Get boundaries defined by the pose with given name
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/srv/GetNamedPoses.srv
        virtual AZ::Outcome<Bounds, FailedResult> GetNamedPoseBounds(const AZStd::string& name) = 0;
    };

    class NamedPoseManagerRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using NamedPoseManagerRequestBus = AZ::EBus<NamedPoseManagerRequests, NamedPoseManagerRequestBusTraits>;
    using NamedPoseManagerRequestBusInterface = AZ::Interface<NamedPoseManagerRequests>;

} // namespace SimulationInterfaces
