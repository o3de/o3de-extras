/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ProximitySensor.h"

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/std/string/string.h>

#include <WarehouseAutomation/ProximitySensor/ProximitySensorNotificationBus.h>
#include <WarehouseAutomation/ProximitySensor/ProximitySensorNotificationBusHandler.h>

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>

#include <AzFramework/Physics/Common/PhysicsSceneQueries.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace WarehouseAutomation
{
    void ProximitySensor::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ProximitySensor, AZ::Component>()
                ->Version(1)
                ->Field("visualize", &ProximitySensor::m_visualize)
                ->Field("frequency", &ProximitySensor::m_frequency)
                ->Field("detectionDistance", &ProximitySensor::m_detectionDistance);

            if (AZ::EditContext* editContext = serialize->GetEditContext())
            {
                editContext->Class<ProximitySensor>("Proximity Sensor", "Proximity object detection component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "WarehouseAutomation")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ProximitySensor::m_visualize, "Visualize", "Whether the detection beam is shown")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ProximitySensor::m_frequency, "Frequency", "Detection frequency")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ProximitySensor::m_detectionDistance,
                        "Detection distance",
                        "The maximum distance from where object is detected")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.f);
            }
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ProximitySensorNotificationBus>("ProximitySensor", "ProximitySensorNotificationBus")
                ->Attribute(AZ::Edit::Attributes::Category, "WarehouseAutomation/ProximitySensor")
                ->Handler<ProximitySensorNotificationBusHandler>();
        }
    }

    ProximitySensor::ProximitySensor()
    {
    }

    void ProximitySensor::Activate()
    {
        if (m_visualize)
        {
            auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
            m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);
        }

        AZ::Transform entityTransform;
        AZ::TransformBus::EventResult(entityTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_detectionDirection = entityTransform.TransformVector(AZ::Vector3::CreateAxisX());
        m_detectionDirection.Normalize();

        AZ::TickBus::Handler::BusConnect();
    }

    void ProximitySensor::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ProximitySensor::Visualize()
    {
        if (m_drawQueue)
        {
            AZStd::vector<AZ::Vector3> linePoints;

            AZ::Vector3 entityTranslation;
            AZ::TransformBus::EventResult(entityTranslation, GetEntityId(), &AZ::TransformBus::Events::GetWorldTranslation);
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;

            linePoints.push_back(entityTranslation);
            if (m_position)
            {
                drawArgs.m_colors = &AZ::Colors::Green;
                linePoints.push_back(m_position.value());
            }
            else
            {
                drawArgs.m_colors = &AZ::Colors::Red;
                linePoints.push_back(entityTranslation + m_detectionDirection * m_detectionDistance);
            }

            const uint8_t pixelSize = 5;
            drawArgs.m_verts = linePoints.data();
            drawArgs.m_vertCount = linePoints.size();

            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawLines(drawArgs);
        }
    }

    void ProximitySensor::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        if (m_visualize)
        {
            Visualize();
        }

        AZ_Assert(m_frequency > 0.f, "ProximitySensor frequency must be greater than zero");
        auto frameTime = 1.f / m_frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
        {
            return;
        }

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching
          // up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }
        DetectionCheck();
    }

    void ProximitySensor::DetectionCheck()
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

            m_position = !result.m_hits.empty() ? std::make_optional(result.m_hits.front().m_position) : std::nullopt;

            if (m_position)
            {
                ProximitySensorNotificationBus::Event(GetEntityId(), &ProximitySensorNotifications::OnObjectInRange);
            }
            else
            {
                ProximitySensorNotificationBus::Event(GetEntityId(), &ProximitySensorNotifications::OnObjectOutOfRange);
            }
        }
    }
} // namespace WarehouseAutomation
