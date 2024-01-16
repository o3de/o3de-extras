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
#include <ROS2/Frame/ROS2FrameBus.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    // Custom JSON serializer for ROS2FrameComponent configuration to handle version conversion
    class JsonFrameComponentConfigSerializer : public AZ::BaseJsonSerializer
    {
    public:
        AZ_RTTI(ROS2::JsonFrameComponentConfigSerializer, "{ac74cbc1-a5dc-4014-85d7-0e7934f352bd}", AZ::BaseJsonSerializer);
        AZ_CLASS_ALLOCATOR_DECL;

        AZ::JsonSerializationResult::Result Load(
            void* outputValue,
            const AZ::Uuid& outputValueTypeId,
            const rapidjson::Value& inputValue,
            AZ::JsonDeserializerContext& context) override;
    };

    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
        friend class JsonFrameComponentConfigSerializer;

    public:
        AZ_COMPONENT(ROS2FrameComponent, "{EE743472-3E25-41EA-961B-14096AC1D66F}");

        ROS2FrameComponent();
        ROS2FrameComponent(const ROS2FrameConfiguration& configuration);

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
        AZStd::string GetFrameID() const;

        //! Set a above-mentioned frame id
        void SetFrameID(const AZStd::string& frameId);

        //! Get the joint name including the namespace
        //! @note Supplementary metadata for Joint components, necessary in some cases for joints addressed by name in ROS 2
        //! @return The namespaced joint name, ready to send in a ROS2 message
        AZ::Name GetJointName() const;

        //! Set the joint name
        //! @note May be populated during URDF import or set by the user in the Editor view
        //! @param jointName does not include the namespace. The namespace prefix is added automatically.
        void SetJointName(const AZStd::string& jointName);

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        AZStd::string GetNamespace() const;

        //! Get a transform between this frame and the next frame up in hierarchy.
        //! @return If the parent frame is found, return a Transform between this frame and the parent.
        //! Otherwise, return a global Transform.
        //! @note Parent frame is not the same as parent Transform: there could be many Transforms in between without ROS2Frame components.
        AZ::Transform GetFrameTransform() const;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        AZStd::string GetGlobalFrameName() const;

        //! Updates the namespace and namespace strategy of the underlying namespace configuration
        //! @param ros2Namespace Namespace to set.
        //! @param strategy Namespace strategy to use.
        void UpdateNamespaceConfiguration(const AZStd::string& ros2Namespace, NamespaceConfiguration::NamespaceStrategy strategy);

        //! Get the configuration of this component.
        ROS2FrameConfiguration GetConfiguration() const;

    private:
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        bool IsTopLevel() const; //!< True if this entity does not have a parent entity with ROS2.

        //! Whether transformation to parent frame can change during the simulation, or is fixed.
        bool IsDynamic() const;

        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        //! Return the frame id of this frame's parent. It can be useful to determine ROS 2 transformations.
        //! @return Parent frame ID.
        //! @note This also works with top-level frames, returning a global frame name.
        //! @see GetGlobalFrameName().
        AZStd::string GetParentFrameID() const;

        // Deprecated values used for backwards compatibility
        NamespaceConfiguration m_namespaceConfiguration;
        AZStd::string m_frameName;
        AZStd::string m_jointName;

        bool m_publishTransform;
        bool m_isDynamic;

        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
    };
} // namespace ROS2
