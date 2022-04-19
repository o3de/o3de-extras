/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "TransformPublisher.h"
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
    {
    public:
        AZ_COMPONENT(ROS2FrameComponent, "{EE743472-3E25-41EA-961B-14096AC1D66F}", AZ::Component);

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

    private:
        AZ::TransformInterface* GetEntityTransformInterface() const;
        const ROS2FrameComponent* GetParentROS2FrameComponent() const;

        // If parent entity does not exist or does not have a ROS2Frame component, return ros2 default global frame: "world"
        AZStd::string GetParentFrameID() const;

        // TODO - Editor component: validation of fields, constraints between values and so on
        AZStd::string m_namespace; // TODO - option to fill from entity name, default = true, validation
        AZStd::string m_frameName = "sensor_frame"; // TODO - option to fill from entity name, validation

        // Whether relationship to parent frame is static (fixed joint) or dynamic
        // TODO - this will be extended to URDF joint types
        // bool m_isDynamic = true; // TODO - determine automatically

        // TODO - transform-related data (ROS2Transform needs ROS2Frame, ROS2Publisher also needs ROS2Frame)
        // bool m_publishTransform = true;
        AZStd::unique_ptr<TransformPublisher> m_transformPublisher;
    };
}  // namespace ROS2
