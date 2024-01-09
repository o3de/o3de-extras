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
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    //! This component marks a reference frame for ROS 2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS 2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , public ROS2FrameComponentBus::Handler
        , public AZ::EntityBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameEditorComponent, "{f76d6f29-73c3-40b2-bcc2-47fc824c25df}", AzToolsFramework::Components::EditorComponentBase);

        ROS2FrameEditorComponent() = default;
        ~ROS2FrameEditorComponent() = default;
        //! Initialize to a specific frame id
        ROS2FrameEditorComponent(const AZStd::string& frameId);

        ROS2FrameEditorComponent(const ROS2FrameConfiguration ros2FrameConfiguration);

        // AzToolsFramework::Components::EditorComponentBase overrides
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Set the frame id.
        //! @param frameId frameId to be set in the configuration.
        void SetFrameID(const AZStd::string& frameId);

        //! Set the joint name
        //! @note May be populated during URDF import or set by the user in the Editor view
        //! @param jointName joint name without the namespace (the namespace prefix is added automatically).
        void SetJointName(const AZStd::string& jointName);

        //! Updates the namespace and namespace strategy of the underlying namespace configuration
        //! @param ros2Namespace Namespace to set.
        //! @param strategy Namespace strategy to use.
        void UpdateNamespaceConfiguration(const AZStd::string& ros2Namespace, const NamespaceConfiguration::NamespaceStrategy& strategy);

        // ROS2FrameComponentBus::Handler overrides
        AZStd::string GetFrameID() const override;
        AZ::Name GetJointName() const override;
        AZStd::string GetNamespace() const override;
        void UpdateNamespace(const AZStd::string& parentNamespace) override;
        AZStd::string GetGlobalFrameName() const override;
        bool IsTopLevel() const override; //!< True if this entity does not have a parent entity with ROS2.
        AZ::EntityId GetFrameParent() const override;
        AZStd::set<AZ::EntityId> GetFrameChildren() const override;

    private:
        AZ::Crc32 OnFrameConfigurationChange();

        // AZ::EntityBus::Handler override.
        void OnEntityNameChanged(const AZStd::string& name) override;

        ROS2FrameConfiguration m_configuration;
    };
} // namespace ROS2
