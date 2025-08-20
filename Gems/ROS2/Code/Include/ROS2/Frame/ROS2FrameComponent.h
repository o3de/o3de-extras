/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/Json/BaseJsonSerializer.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2FrameEditorComponentBus.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! This component marks an important reference frame in the ROS2 ecosystem.
    //!
    //! It serves as a frame of reference for sensor data and is responsible for publishing
    //! ROS2 static and dynamic transforms (/tf_static, /tf) through ROS2Transform.
    //! It also facilitates namespace handling within entity hierarchies.
    //!
    //! An entity can only have a single ROS2FrameComponent. Many ROS2 components require this component.
    //!
    //! @note For proper robot modeling, this component should be present on every level
    //!       of entity hierarchy that represents a joint (both fixed and dynamic).
    class ROS2FrameComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ROS2FrameEditorComponentBus::Handler
    {
        friend class JsonFrameComponentConfigSerializer;

    public:
        AZ_COMPONENT(ROS2FrameComponent, "{EE743472-3E25-41EA-961B-14096AC1D66F}");

        ROS2FrameComponent();
        ROS2FrameComponent(const ROS2FrameConfiguration& configuration);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Get the frame ID including namespace, ready for use in ROS2 messages.
        //! @return Frame ID with namespace prefix, suitable for ROS2 message headers
        AZStd::string GetNamespacedFrameID() const override;

        //! Set the frame ID (without namespace).
        //! @param frameId The frame identifier (namespace will be added automatically)
        void SetFrameID(const AZStd::string& frameId) override;

        //! Get the joint name including namespace.
        //! @note This is supplementary metadata for Joint components, necessary when
        //!       joints are addressed by name in ROS2 systems.
        //! @return The namespaced joint name, ready for use in ROS2 messages
        AZ::Name GetNamespacedJointName() const override;

        //! Set the joint name (without namespace).
        //! @note May be populated during URDF import or set manually in the Editor.
        //! @param jointName The joint identifier (namespace prefix added automatically)
        void SetJointName(const AZStd::string& jointName) override;

        //! Get the complete namespace for this frame.
        //! @note This namespace should be used for any publisher or subscriber in the same entity.
        //! @return Complete namespace including parent namespaces
        AZStd::string GetNamespace() const override;

        //! Get the global frame name used in the ROS2 ecosystem.
        //! @return The name of the global frame with namespace. Typically "odom", "map", or "world".
        AZStd::string GetGlobalFrameName() const override;

        //! Check if this is a top-level frame (no parent ROS2FrameComponent).
        bool IsTopLevel() const override;

        //! Check if this frame publishes dynamic transforms.
        //! @return true if transforms are published continuously to /tf, false if published once to /tf_static
        bool IsDynamic() const override;

        //! Get a copy of the current configuration.
        //! @return Current component configuration
        ROS2FrameConfiguration GetConfiguration() const override;

        //! Set the configuration for this component.
        //! @param configuration The new configuration to apply
        void SetConfiguration(const ROS2FrameConfiguration& configuration) override;

        //! Update both namespace and namespace strategy.
        //! @param ros2Namespace The namespace to set
        //! @param strategy The namespace strategy to use
        void UpdateNamespaceConfiguration(const AZStd::string& ros2Namespace, NamespaceConfiguration::NamespaceStrategy strategy);

        //! Disables transform publishing on component activation.
        void DisablePublishingOnActivate();

        //! Starts publishing transforms. Should be called only after DisablePublishingOnActivate was
        //! called and the component is active.
        void EnableTransformPublishing();

    private:
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        //! Get the transform between this frame and its parent frame.
        //! @return Transform from parent frame to this frame. If no parent frame exists,
        //!         returns the world transform. If parent frame is found but has multiple
        //!         transforms in between, calculates the relative transform properly.
        //! @note Parent frame is not necessarily the immediate parent Transform - there
        //!       could be multiple Transforms in between without ROS2FrameComponents.
        AZ::Transform GetFrameTransform() const;

        //! Find the first parent entity with a ROS2FrameComponent.
        //! @return Pointer to parent ROS2FrameComponent, or nullptr if none found
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        //! Get the frame ID of this frame's parent.
        //! @return Parent frame ID, or global frame name if this is a top-level frame
        //! @note This works with top-level frames by returning the global frame name
        //! @see GetGlobalFrameName()
        AZStd::string GetParentFrameID() const;

        ROS2FrameConfiguration m_configuration;

        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
        bool m_shouldStartPublishingOnActivate = true; //!< Track if automatic publishing is enabled
        bool m_publishingInitialized = false; //!< Track if publishing has been initialized
    };
} // namespace ROS2
