/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Frame/NamespaceConfiguration.h"
#include "Frame/ROS2Transform.h"
#include <AzCore/Component/Component.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzFramework/Components/TransformComponent.h>

namespace ROS2
{
    //! This component marks an interesting reference frame for ROS2 ecosystem.
    //! It serves as sensor data frame of reference and is responsible, through ROS2Transform, for publishing
    //! ros2 static and dynamic transforms (/tf_static, /tf). It also facilitates namespace handling.
    //! An entity can only have a single ROS2Frame on each level. Many ROS2 Components require this component.
    //! @note A robot should have this component on every level of entity hierarchy (for each joint, fixed or dynamic)
    class ROS2FrameComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameComponent, "{EE743472-3E25-41EA-961B-14096AC1D66F}");

        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        AZStd::string GetFrameID() const;

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        AZStd::string GetNamespace() const;

        //! Get AZ Transform for this frame
        //! @return If parent ROS2Frame is found, return its Transform.
        //! Otherwise, return a global Transform.
        const AZ::Transform& GetFrameTransform() const;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame. It is typically "odom", "map", "world".
        static const char* GetGlobalFrameName(); // TODO - allow to configure global frame in a specialized component

    private:
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        bool IsTopLevel() const; //!< True if this entity does not have a parent entity with ROS2.

        //! Whether transformation to parent frame can change during the simulation, or is fixed
        bool IsDynamic() const;

        AZ::TransformInterface* GetEntityTransformInterface() const;
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        //! If parent entity does not exist or does not have a ROS2FrameComponent, return ROS2 default global frame.
        //! @see GetGlobalFrameName().
        AZStd::string GetParentFrameID() const;

        // TODO - Editor component: validation of fields, constraints between values and so on
        NamespaceConfiguration m_namespaceConfiguration;
        AZStd::string m_frameName = "sensor_frame"; // TODO - option to fill from entity name

        bool m_publishTransform = true;
        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
    };
}  // namespace ROS2
