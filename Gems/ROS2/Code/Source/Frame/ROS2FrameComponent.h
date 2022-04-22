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
    /// This component marks an interesting reference frame and captures common behavior
    /// for sensor data frame of reference, publishing ros2 static and dynamic transforms (tf)
    /// and helping to export robot prefab to URDF.
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

        // namespaced and validated, ready to send in ros2 messages (TODO-validate)
        AZStd::string GetFrameID() const;
        AZStd::string GetNamespace() const;

        // Transform for direct parent if parent ROS2Frame is found, otherwise transformation is global ("world")
        const AZ::Transform& GetFrameTransform() const;

        // Returns whatever is set as global frame name in ros2 ecosystem (typically, "map" or "world")
        static const char* GetGlobalFrameName();

    private:
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // True if this entity does not have a parent entity with ROS2 Frame
        bool IsTopLevel() const;

        // Whether transformation to parent frame can change during the simulation, or is fixed
        bool IsDynamic() const;

        AZ::TransformInterface* GetEntityTransformInterface() const;
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        // If parent entity does not exist or does not have a ROS2Frame component, return ros2 default global frame: "world"
        AZStd::string GetParentFrameID() const;

        // TODO - Editor component: validation of fields, constraints between values and so on
        NamespaceConfiguration m_namespaceConfiguration; // TODO - validation
        AZStd::string m_frameName = "sensor_frame"; // TODO - option to fill from entity name, validation

        bool m_publishTransform = true;
        AZStd::unique_ptr<ROS2Transform> m_ros2Transform;
    };
}  // namespace ROS2
