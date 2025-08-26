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
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameComponentInterface.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{

    namespace
    {
        inline constexpr const char* DefaultGlobalFrameName = "odom";
        inline constexpr const char* DefaultGlobalFrameNameConfigurationKey = "/O3DE/ROS2/GlobalFrameName";
    } // namespace

    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ROS2FrameComponentBus::Handler
        , public ROSFrameInterface
    {
    public:
        AZ_COMPONENT(ROS2FrameComponent, ROS2FrameComponentTypeId, ROSFrameInterface);

        ROS2FrameComponent();

        ROS2FrameComponent(const ROS2FrameConfiguration& ros2FrameConfiguration);

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

        //! Disable publishing the transform. It allows to tinker with the transform without spamming /tf_static
        //! you can disable it inactive state, and then enable it again.
        void DisablePublishingTransform();
        //! Enable publishing the transform. It will compute the namespace and frame name again.
        //! Note that other won't be notified of the change.
        void EnablePublishingTransform();

        AZStd::string GetNamespace() const override;
        AZStd::string GetNamespacedFrameID() const override;
        AZStd::string GetNamespacedJointName() const override;
        AZStd::string GetJointName() const override;
        AZStd::string GetFrameName() const override;
        AZStd::string GetGlobalFrameID() const override;

        // ROSFrameInterface overrides
        ROS2FrameConfiguration GetConfiguration() const override;
        void SetConfiguration(const ROS2FrameConfiguration& config) override;
    private:
        AZStd::string m_computedNamespace; //!< Cached namespace
        AZStd::string m_computedFrameName; //!< Cached full frame name, including namespace
        AZStd::string m_computedJointName; //!< Cached full joint name, including namespace
        ROS2FrameConfiguration m_configuration; //!< Configuration for this frame component
        bool m_disabled = false;
        AZStd::optional<AZ::EntityId> m_parentFrame; //!< Cached parent entity with ROS2FrameComponent, if any
        AZStd::optional<AZStd::string> m_sourceFrame; //!< If not set, the source frame is assumed to be the parent frame in the TF tree
        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;

        AZ::Transform GetFrameTransform() const;
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        void ComputeNamespaceAndFrameName();
    };
} // namespace ROS2
