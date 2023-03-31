/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2EffortSensorComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <Source/RigidBodyComponent.h>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
namespace ROS2
{
    namespace Internal
    {
        const char* WrenchMessageType = "geometry_msgs::msg::WrenchStamped";
    }

    void ROS2EffortSensorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2EffortSensorComponent, ROS2SensorComponent>()->Version(1);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2EffortSensorComponent>("ROS2 Effort Sensor", "Effort sensor component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }

    AZ::Entity::ComponentArrayType ROS2EffortSensorComponent::FindCompatibleJointComponents(){
        AZ::Entity::ComponentArrayType components;
        for (auto type : CompatibleJointComponents)
        {
            auto foundComponents = m_entity->FindComponents(type);
            AZStd::move(foundComponents.begin(), foundComponents.end(), AZStd::back_inserter(components));
        }
        return components;
    }

    void ROS2EffortSensorComponent::Activate()
    {
        AZ::Entity::ComponentArrayType componentPtrVec = FindCompatibleJointComponents();
        AZ_Warning("ROS2EffortSensorComponent", componentPtrVec.size()>1, "ROS2EffortSensorComponent can work with exactly only one fixed joint component.");
        AZ_Warning("ROS2EffortSensorComponent", !componentPtrVec.empty(), "ROS2EffortSensorComponent needs compatible joint");
        if (componentPtrVec.size()>0){
            m_jointId = AZ::EntityComponentIdPair(m_entity->GetId(), componentPtrVec.front()->GetId());
        }
        ROS2SensorComponent::Activate();

        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Effort sensor");

        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::WrenchMessageType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_effortPublisher = ros2Node->create_publisher<geometry_msgs::msg::WrenchStamped>(fullTopic.data(), publisherConfig.GetQoS());
    }
    ROS2EffortSensorComponent::ROS2EffortSensorComponent()
    {
        TopicConfiguration tc;
        const AZStd::string type = Internal::WrenchMessageType;
        tc.m_type = type;
        tc.m_topic = "effort";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
    }

    void ROS2EffortSensorComponent::FrequencyTick()
    {
        AZStd::pair<AZ::Vector3, AZ::Vector3> wrench{AZ::Vector3::CreateZero(),AZ::Vector3::CreateZero()};
        PhysX::JointRequestBus::EventResult(wrench, m_jointId, &PhysX::JointRequests::GetForces);
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());

        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        msg.header.frame_id = ros2Frame->GetParentFrameID().c_str();

        msg.wrench.torque.x = wrench.second.GetX();
        msg.wrench.torque.y = wrench.second.GetY();
        msg.wrench.torque.z = wrench.second.GetZ();

        msg.wrench.force.x = wrench.first.GetX();
        msg.wrench.force.y = wrench.first.GetY();
        msg.wrench.force.z = wrench.first.GetZ();
        m_effortPublisher->publish(msg);
    }

    void ROS2EffortSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

} // namespace ROS2
