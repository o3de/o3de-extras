/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "SimulationInterfacesTypeIds.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
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
    using NamedPoseList = AZStd::vector<NamedPose>;

    class NamedPoseManagerRequests
    {
    public:
        AZ_RTTI(NamedPoseManagerRequests, NamedPoseManagerRequestsTypeId);
        virtual ~NamedPoseManagerRequests() = default;

        //! Register named pose with given id.
        virtual AZ::Outcome<void, FailedResult> RegisterNamedPose(AZ::EntityId namedPoseEntityId) = 0;

        //! Unregister named pose with given id.
        virtual AZ::Outcome<void, FailedResult> UnregisterNamedPose(AZ::EntityId namedPoseEntityId) = 0;

        //! Get named poses matching given TagFiler
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/TagsFilter.msg
        virtual AZ::Outcome<NamedPoseList, FailedResult> GetNamedPoses(const TagFilter& tags) = 0;

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

    class NamedPoseComponentRequests
    {
    public:
        AZ_RTTI(NamedPoseComponentRequests, NamedPoseComponentRequestsTypeId);
        virtual ~NamedPoseComponentRequests() = default;

        //! get configuration of the namedPoseComponent
        virtual NamedPose GetConfiguration() = 0;
    };

    class NamedPoseComponentRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //////////////////////////////////////////////////////////////////////////
    };

    using NamedPoseComponentRequestBus = AZ::EBus<NamedPoseComponentRequests, NamedPoseComponentRequestBusTraits>;
    using NamedPoseComponentRequestBusInterface = AZ::Interface<NamedPoseComponentRequests>;

} // namespace SimulationInterfaces
