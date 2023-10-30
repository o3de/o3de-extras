/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
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

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        AZStd::string GetFrameID() const override;

        //! Set a above-mentioned frame id
        void SetFrameID(const AZStd::string& frameId);

        //! Get the joint name including the namespace
        //! @note Supplementary metadata for Joint components, necessary in some cases for joints addressed by name in ROS 2
        //! @return The namespaced joint name, ready to send in a ROS2 message
        AZ::Name GetJointName() const override;

        //! Set the joint name
        //! @note May be populated during URDF import or set by the user in the Editor view
        //! @param jointNameString does not include the namespace. The namespace prefix is added automatically.
        void SetJointName(const AZStd::string& jointNameString);

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        AZStd::string GetNamespace() const override;

        //! Update the parents namespace
        //! @param parentsNamespace
        void OnNamespaceChange(AZStd::string parentsNamespace) override;

        //! Updates the namespace and namespace strategy of the underlying namespace configuration
        //! @param ns Namespace to set.
        //! @param strategy Namespace strategy to use.
        void UpdateNamespaceConfiguration(const AZStd::string& ns, NamespaceConfiguration::NamespaceStrategy strategy);

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        AZStd::string GetGlobalFrameName() const override;

        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        bool IsTopLevel() const; //!< True if this entity does not have a parent entity with ROS2.

        AZ::Crc32 OnConfigurationChange();

        void OnEntityNameChanged(const AZStd::string& name) override;

        ROS2FrameConfiguration m_configuration;
    };
} // namespace ROS2
