/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

namespace ROS2
{

    //! Simple component that reads forces that applied to fixed joint. It currently supports fixed joints.
    class ROS2EffortSensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2EffortSensorComponent, "{a518fd30-bcf9-11ed-afa1-0242ac120002}", ROS2SensorComponent);
        ROS2EffortSensorComponent();
        ~ROS2EffortSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        // Component overrides
        void Activate() override;
    private:

        // ROS2SensorComponent overrides ...
        void FrequencyTick() override;

        AZ::Entity::ComponentArrayType FindCompatibleJointComponents();

        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> m_effortPublisher;
        AZ::EntityComponentIdPair m_jointId;
        const AZStd::vector<AZ::TypeId> CompatibleJointComponents{
            AZ::TypeId("{02E6C633-8F44-4CEE-AE94-DCB06DE36422}"), // PhysX::FixedJointComponent
        };
    };
} // namespace ROS2
