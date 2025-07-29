/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2TypeIds.h>

#include <AzCore/Component/Component.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2
{
    //! Interface for creating ROS2 editor components.
    //! It is intended to be used by external tools that create entities, e.g., when importing robots from SDF, URDF, or OpenUSD.
    class ROS2EditorRequests
    {
    public:
        AZ_RTTI(ROS2EditorRequests, ROS2EditorRequestsTypeId);
        virtual ~ROS2EditorRequests() = default;

        //! Create a new ROS2FrameEditorComponent.
        //! @param entity The entity to which the frame component will be added.
        //! @return A pointer to the newly created AZ::Component representing the ROS2FrameEditorComponent (or nullptr if failed).
        virtual AZ::Component* CreateROS2FrameEditorComponent(AZ::Entity& entity) = 0;

        //! Create a new ROSS2FrameEditorComponent.
        //! @param entity The entity to which the frame component will be added.
        //! @param frameConfiguration The configuration for the ROS2FrameEditorComponent.
        //! @return A pointer to the newly created AZ::Component representing the ROS2FrameEditorComponent (or nullptr if failed).
        virtual AZ::Component* CreateROS2FrameEditorComponent(
            AZ::Entity& entity, const ROS2::ROS2FrameConfiguration& frameConfiguration) = 0;
    };

    class ROS2EditorBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ROS2EditorInterface = AZ::Interface<ROS2EditorRequests>;

} // namespace ROS2
