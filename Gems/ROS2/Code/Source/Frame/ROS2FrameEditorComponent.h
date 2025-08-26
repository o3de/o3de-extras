/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponentInterface.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! This component marks a reference frame for ROS 2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS 2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public ROS2FrameEditorComponentBus::Handler
        , public ROS2FrameComponentBus::Handler
        , public AZ::EntityBus::Handler
        , public ROSFrameInterface
    {
    public:
        AZ_EDITOR_COMPONENT(
            ROS2FrameEditorComponent, ROS2FrameEditorComponentTypeId, AzToolsFramework::Components::EditorComponentBase, ROSFrameInterface);

        ROS2FrameEditorComponent() = default;
        ~ROS2FrameEditorComponent() = default;
        ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // ROSFrameInterface overrides
        ROS2FrameConfiguration GetConfiguration() const override;
        void SetConfiguration(const ROS2FrameConfiguration& config) override;

    private:
        // ROS2FrameEditorComponentBus::Handler overrides
        AZStd::string GetNamespacedFrameID() const override;
        AZStd::string GetNamespacedJointName() const override;
        AZStd::string GetNamespace() const override;
        AZStd::string GetJointName() const override;
        AZStd::string GetFrameName() const override;
        void UpdateNamespace() override;
        AZStd::string GetGlobalFrameID() const override;
        bool IsTopLevel() const override; //!< True if this entity does not have a parent entity with ROS2.
        AZ::EntityId GetFrameParent() const override;
        AZStd::set<AZ::EntityId> GetFrameChildren() const override;
        void SetJointName(const AZStd::string& frameId) override;

        // AZ::EntityBus::Handler override.
        void OnEntityNameChanged(const AZStd::string& name) override;

        AZ::Crc32 OnFrameConfigurationChange();

        ROS2FrameConfiguration m_configuration;

        AZStd::string m_effectiveNamespace; //! <! Full namespace to show in the Editor
        AZStd::string m_fullName; //! <! Full frame name to show in the Editor (that will pbe published to ROS 2)
    };
} // namespace ROS2
