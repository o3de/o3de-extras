/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2FrameGameSystemComponent.h"
#include "ROS2FrameSystemBus.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>

namespace ROS2
{

    //! Component which manages the frame entities and their hierarchy.
    //! It is responsible for updating the namespaces of the frame entities and their children.
    //! It also notifies the ROS2FrameEditorComponent about changes in the tree.
    //! Used to register, unregister, track the frame entities in the level entity tree.
    class ROS2FrameEditorSystemComponent
        : public ROS2FrameGameSystemComponent
        , protected AzToolsFramework::EntitySelectionEvents::Bus::MultiHandler
        , protected AZ::TransformNotificationBus::MultiHandler
    {
    public:
        AZ_COMPONENT(ROS2FrameEditorSystemComponent, "{360c4b45-ac02-42d2-9e1a-1d77eb22a054}");

        ROS2FrameEditorSystemComponent();
        ~ROS2FrameEditorSystemComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        // AZ::Component overrides.
        void Activate() override;
        void Deactivate() override;

    private:
        // ROS2FrameSystemInterface::Registrar overrides.
        void RegisterFrame(const AZ::EntityId& frameEntityId) override;
        void UnregisterFrame(const AZ::EntityId& frameEntityId) override;

        // EntitySelectionEvents::Bus::MultiHandler overrides.
        void OnSelected() override;
        //! AZ::TransformNotificationBus::MultiHandler overrides.
        void OnParentChanged(AZ::EntityId oldParent, AZ::EntityId newParent) override;

        AZStd::set<AZ::EntityId> m_registeredEntities; //!< Set of all registered frame entities.
    };
} // namespace ROS2
