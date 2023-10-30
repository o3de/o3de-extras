/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/EBus/Policies.h"
#include "AzCore/Interface/Interface.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/std/string/string.h"
#include "ROS2/Frame/ROS2FrameComponent.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>

namespace ROS2
{
    class ROS2FrameSystemRequests
    {
    public:
        AZ_RTTI(ROS2FrameSystemRequests, "{24fe4584-0499-4a37-bc1a-00ca04bd22f5}");

        virtual void RegisterFrame(AZ::EntityId frameEntityId) = 0;
        virtual void UnregisterFrame(AZ::EntityId frameEntityId) = 0;
        virtual void MoveFrame(AZ::EntityId frameEntityId, AZ::EntityId newParent) = 0;

        virtual void NotifyChange(AZ::EntityId frameEntityId) = 0;

        virtual bool IsTopLevel(AZ::EntityId frameEntityId) const = 0;
        virtual AZ::EntityId GetParentEntityId(AZ::EntityId frameEntityId) const = 0;
    };

    class ROS2FrameSystemBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusHandlerPolicy AddressPolicy = AZ::EBusHandlerPolicy::Single;
    };

    using ROS2FrameSystemInterface = AZ::Interface<ROS2FrameSystemRequests>;
    using ROS2FrameSystemBus = AZ::EBus<ROS2FrameSystemRequests>;
} // namespace ROS2