/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>

namespace ROS2
{
    class ROS2FrameSystemRequests
    {
    public:
        AZ_RTTI(ROS2FrameSystemRequests, "{24fe4584-0499-4a37-bc1a-00ca04bd22f5}");

        //! Registers the ROS2FrameEditorComponent into the ROS2FrameSystemComponent. All ROS2FrameEditorComponents should
        //! register using this function during activation.
        //! This is used to properly handle the entity tree and namespaces, for the ROS2FrameEditorComponents.
        //! @param frameEntityId entityId containing the frame to register.
        virtual void RegisterFrame(const AZ::EntityId& frameEntityId) = 0;

        //! Unregister the ROS2FrameEditorComponent into the system. All ROS2FrameEditorComponents should
        //! unregister using this function during deactivation.
        //! @param frameEntityId entityId containing the frame to unregister.
        virtual void UnregisterFrame(const AZ::EntityId& frameEntityId) = 0;
    };

    class ROS2FrameSystemBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };

    using ROS2FrameSystemInterface = AZ::Interface<ROS2FrameSystemRequests>;
    using ROS2FrameSystemBus = AZ::EBus<ROS2FrameSystemRequests>;
} // namespace ROS2
