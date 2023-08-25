/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "API/ToolsApplicationAPI.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityBus.h"
#include "AzCore/Component/EntityId.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/std/string/string.h"
#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{

    using ROS2FrameEditorComponentBase =
        AzToolsFramework::Components::EditorComponentAdapter<ROS2FrameComponentController, ROS2FrameComponent, ROS2FrameConfiguration>;

    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameEditorComponent
        : public ROS2FrameEditorComponentBase
        , public ROS2FrameNotificationBus::Handler
        , public AzToolsFramework::ToolsApplicationNotificationBus::Handler
        , public AZ::EntityBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameEditorComponent, "{ed8cf823-1813-4423-a874-5de13e9124d2}", AzToolsFramework::Components::EditorComponentBase);

        ROS2FrameEditorComponent();
        //! Initialize to a specific frame id
        ROS2FrameEditorComponent(const AZStd::string& frameId);
        ROS2FrameEditorComponent(const ROS2FrameConfiguration& config);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        bool ShouldActivateController() const override;

        void OnActivate(AZ::EntityId entity, AZ::EntityId parentEntity) override;

        void OnDeactivate(AZ::EntityId entity, AZ::EntityId parentEntity) override;

        void OnReconfigure(AZ::EntityId entity) override;

        void OnConfigurationChange() override;

        void EntityParentChanged(AZ::EntityId entityId, AZ::EntityId newParentId, AZ::EntityId oldParentId) override;

        void OnEntityNameChanged(const AZStd::string& name);

    private:
        void RefreshEffectiveNamespace();

        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;

        AZ::EntityId m_parentFrameEntity;

        AZStd::set<AZ::EntityId> m_nonFramePredecessors;

        AZStd::string m_effectiveNamespace;
    };
} // namespace ROS2
