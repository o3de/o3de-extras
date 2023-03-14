/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "ROS2WheelOdometry.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include "VehicleModelComponent.h"
namespace ROS2
{
   namespace Internal
   {
       const char* kHweelOdometryMsgType = "nav_msgs::msg::Odometry";
   }

   void ROS2WheelOdometryComponent::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<ROS2WheelOdometryComponent, ROS2SensorComponent>()
               ->Version(1)
               ->Field("AngularGtImprovement", &ROS2WheelOdometryComponent::m_gtWeightAngular)
               ->Field("LinearGtImprovement", &ROS2WheelOdometryComponent::m_gtWeightLinear);

                   if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<ROS2WheelOdometryComponent>("ROS2 Wheel Odometry Sensor", "Odometry sensor component")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                   ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                   ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2WheelOdometryComponent::m_gtWeightAngular, "Angular Ground Truth Improvement", "Angular Ground Truth Improvement")
                   ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2WheelOdometryComponent::m_gtWeightLinear, "Linear Ground Truth Improvement", "Linear Ground Truth Improvement");
           }
       }
   }

   ROS2WheelOdometryComponent::ROS2WheelOdometryComponent()
   {
       TopicConfiguration tc;
       const AZStd::string type = Internal::kOdometryMsgType;
       tc.m_type = type;
       tc.m_topic = "odom";
       m_sensorConfiguration.m_frequency = 10;
       m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
   }

   void ROS2WheelOdometryComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
   {
       required.push_back(AZ_CRC_CE("ROS2Frame"));
       required.push_back(AZ_CRC_CE("SkidSteeringModelService"));
   }

   void ROS2WheelOdometryComponent::Activate()
   {
       m_rigidBodyPtr = nullptr;
       m_robotPose = AZ::Vector3 {0};
       m_robotRotation = AZ::Quaternion {0,0,0,1};

       // "odom" is globally fixed frame for all robots, no matter the namespace
       m_odometryMsg.header.frame_id = ROS2Names::GetNamespacedName(GetNamespace(), "odom").c_str();
       m_odometryMsg.child_frame_id = GetFrameID().c_str();

       ROS2SensorComponent::Activate();
       auto ros2Node = ROS2Interface::Get()->GetNode();
       AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for Odometry sensor");

       const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[Internal::kHweelOdometryMsgType];
       const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
       m_odometryPublisher = ros2Node->create_publisher<nav_msgs::msg::Odometry>(fullTopic.data(), publisherConfig.GetQoS());

       m_onSceneActiveSimulatedBodiesEvent = AzPhysics::SceneEvents::OnSceneActiveSimulatedBodiesEvent::Handler(
       [this]([[maybe_unused]]AzPhysics::SceneHandle sceneHandle, [[maybe_unused]]const AzPhysics::SimulatedBodyHandleList& activeBodyList, float deltaTime)
       {
               if (m_rigidBodyPtr == nullptr)
               {
                   Physics::RigidBodyRequestBus::EventResult(m_rigidBodyPtr, m_entity->GetId(), &Physics::RigidBodyRequests::GetRigidBody);
                   AZ_Assert(m_rigidBodyPtr, "NoRigdBody");
               }
               AZ_Assert(m_rigidBodyPtr, "NoRigdBody");

               const auto transformGt = m_rigidBodyPtr->GetTransform().GetInverse();
               const auto localAngularGt = transformGt.TransformVector(m_rigidBodyPtr->GetAngularVelocity());
               const auto localLinearGt = transformGt.TransformVector(m_rigidBodyPtr->GetLinearVelocity());

               AZStd::pair<AZ::Vector3, AZ::Vector3> vt;

               VehicleDynamics::VehicleInputControlRequestBus::EventResult(
                   vt, GetEntityId(), &VehicleDynamics::VehicleInputControlRequests::GetWheelsOdometry);

               vt.first = (1.0f - m_gtWeightLinear) * vt.first + m_gtWeightLinear * localLinearGt;
               vt.second = (1.0f - m_gtWeightAngular) * vt.second + m_gtWeightAngular * localAngularGt;

               m_odometryMsg.twist.twist.linear = ROS2Conversions::ToROS2Vector3(vt.first);
               m_odometryMsg.twist.twist.angular = ROS2Conversions::ToROS2Vector3(vt.second);
               if (m_sensorConfiguration.m_frequency>0)
           {
               auto updatePos = deltaTime * vt.first; // in meters
               auto updateRot = deltaTime * vt.second; // in radians
               m_robotPose += m_robotRotation.TransformVector(updatePos);
               m_robotRotation *= AZ::Quaternion::CreateFromScaledAxisAngle(updateRot);
           }
           if (IsPublicationDeadline(deltaTime, deltaTime))
           {
               m_odometryMsg.pose.pose.position = ROS2Conversions::ToROS2Point(m_robotPose);
               m_odometryMsg.pose.pose.orientation = ROS2Conversions::ToROS2Quaternion(m_robotRotation);

               m_odometryPublisher->publish(m_odometryMsg);
           }
       });


       auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
       AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
       sceneInterface->RegisterSceneActiveSimulatedBodiesHandler(sceneHandle, m_onSceneActiveSimulatedBodiesEvent);

   }

   void ROS2WheelOdometryComponent::Deactivate()
   {
       ROS2SensorComponent::Deactivate();
       m_odometryPublisher.reset();
   }
} // namespace ROS2
