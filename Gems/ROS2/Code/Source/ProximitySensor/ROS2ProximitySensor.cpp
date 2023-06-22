/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ProximitySensor.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/std/string/string.h>

#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/PhysicsScene.h>

#include <std_msgs/msg/detail/bool__struct.hpp>

namespace ROS2
{
    void ROS2ProximitySensor::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2ProximitySensor, ROS2SensorComponent>()->Version(1)->Field(
                "detectionDistance", &ROS2ProximitySensor::m_detectionDistance);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ROS2ProximitySensor>("ROS2 Proximity Sensor", "Proximity object detection component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ProximitySensor::m_detectionDistance,
                        "Detection distance",
                        "The maximum distance from where object is detected")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.);
            }
        }
    }

    ROS2ProximitySensor::ROS2ProximitySensor()
    {
        TopicConfiguration tc;
        AZStd::string type = "std_msgs::msg::bool";
        tc.m_type = type;
        tc.m_topic = "proximity_sensor";
        m_sensorConfiguration.m_frequency = 10;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(type, tc));
    }

    void ROS2ProximitySensor::Activate()
    {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.size() == 1, "Invalid configuration of publishers for proximity sensor");

        const TopicConfiguration& publisherConfig = m_sensorConfiguration.m_publishersConfigurations["std_msgs::msg::bool"];
        AZStd::string fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_detectionPublisher = ros2Node->create_publisher<std_msgs::msg::Bool>(fullTopic.data(), publisherConfig.GetQoS());

        if (m_sensorConfiguration.m_visualise)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }

        AZ::Transform entityTransform;
        AZ::TransformBus::EventResult(entityTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_detectionDirection = AZ::Vector3::CreateAxisX();
        m_detectionDirection = entityTransform.TransformVector(m_detectionDirection);
        m_detectionDirection.Normalize();

        ROS2SensorComponent::Activate();
    }

    void ROS2ProximitySensor::Deactivate()
    {
        m_detectionPublisher.reset();
        ROS2SensorComponent::Deactivate();
    }

    void ROS2ProximitySensor::Visualise()
    {
        if (m_drawQueue)
        {
            AZStd::vector<AZ::Vector3> linePoints;

            AZ::Vector3 entityTranslation;
            AZ::TransformBus::EventResult(entityTranslation, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

            linePoints.push_back(entityTranslation);
            linePoints.push_back(entityTranslation + m_detectionDirection * m_detectionDistance);

            const uint8_t pixelSize = 5;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = linePoints.data();
            drawArgs.m_vertCount = linePoints.size();
            drawArgs.m_colors = m_wasObjectDetected ? &AZ::Colors::Green : &AZ::Colors::Red;
            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawLines(drawArgs);
        }
    }

    void ROS2ProximitySensor::FrequencyTick()
    {
        auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();

        AZ::Vector3 entityTranslation;
        AZ::TransformBus::EventResult(entityTranslation, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);

        AzPhysics::RayCastRequest request;
        request.m_start = entityTranslation;
        request.m_direction = m_detectionDirection;
        request.m_distance = m_detectionDistance;

        if (AzPhysics::SceneHandle sceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
            sceneHandle != AzPhysics::InvalidSceneHandle)
        {
            AzPhysics::SceneQueryHits result = sceneInterface->QueryScene(sceneHandle, &request);

            std_msgs::msg::Bool msg;
            m_wasObjectDetected = !result.m_hits.empty();
            msg.set__data(m_wasObjectDetected);
            m_detectionPublisher->publish(msg);
        }
    }
} // namespace ROS2
