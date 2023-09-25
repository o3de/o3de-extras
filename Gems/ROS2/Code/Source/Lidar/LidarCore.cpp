/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LidarCore.h"
#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{

    void LidarCore::Reflect(AZ::ReflectContext* context)
    {
        LidarSensorConfiguration::Reflect(context);

        if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<LidarCore>()->Version(1)->Field("lidarConfiguration", &LidarCore::m_lidarConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<LidarCore>("ROS2 Lidar Sensor", "Lidar sensor component")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox, &LidarCore::m_lidarConfiguration, "Lidar configuration", "Lidar configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void LidarCore::ConnectToLidarRaycaster()
    {
        if (auto raycasterId = m_implementationToRaycasterMap.find(m_lidarConfiguration.m_lidarSystem);
            raycasterId != m_implementationToRaycasterMap.end())
        {
            m_lidarRaycasterId = raycasterId->second;
            return;
        }

        m_lidarRaycasterId = LidarId::CreateNull();
        LidarSystemRequestBus::EventResult(
            m_lidarRaycasterId, AZ_CRC(m_lidarConfiguration.m_lidarSystem), &LidarSystemRequestBus::Events::CreateLidar, m_entityId);
        AZ_Assert(!m_lidarRaycasterId.IsNull(), "Could not access selected Lidar System.");

        m_implementationToRaycasterMap.emplace(m_lidarConfiguration.m_lidarSystem, m_lidarRaycasterId);
    }

    void LidarCore::ConfigureLidarRaycaster()
    {
        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayOrientations, m_lastRotations);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId,
            &LidarRaycasterRequestBus::Events::ConfigureMinimumRayRange,
            m_lidarConfiguration.m_lidarParameters.m_minRange);
        LidarRaycasterRequestBus::Event(
            m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRayRange, m_lidarConfiguration.m_lidarParameters.m_maxRange);

        if ((m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::Noise) &&
            m_lidarConfiguration.m_lidarParameters.m_isNoiseEnabled)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureNoiseParameters,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_angularNoiseStdDev,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_distanceNoiseStdDevBase,
                m_lidarConfiguration.m_lidarParameters.m_noiseParameters.m_distanceNoiseStdDevRisePerMeter);
        }

        RaycastResultFlags requestedFlags = RaycastResultFlags::Ranges | RaycastResultFlags::Points;

        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRaycastResultFlags, requestedFlags);

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureIgnoredCollisionLayers,
                m_lidarConfiguration.m_ignoredCollisionLayers);
        }

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ExcludeEntities, m_lidarConfiguration.m_excludedEntities);
        }

        if (m_lidarConfiguration.m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints)
        {
            LidarRaycasterRequestBus::Event(
                m_lidarRaycasterId,
                &LidarRaycasterRequestBus::Events::ConfigureMaxRangePointAddition,
                m_lidarConfiguration.m_addPointsAtMax);
        }
    }

    LidarCore::LidarCore(const AZStd::vector<LidarTemplate::LidarModel>& availableModels)
        : m_lidarConfiguration(availableModels)
    {
    }

    LidarCore::LidarCore(const LidarSensorConfiguration& lidarConfiguration)
        : m_lidarConfiguration(lidarConfiguration)
    {
    }

    void LidarCore::VisualizeResults() const
    {
        if (m_lastScanResults.m_points.empty())
        {
            return;
        }

        if (m_drawQueue)
        {
            const uint8_t pixelSize = 2;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = m_lastScanResults.m_points.data();
            drawArgs.m_vertCount = m_lastScanResults.m_points.size();
            drawArgs.m_colors = &AZ::Colors::Red;
            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawPoints(drawArgs);
        }
    }

    void LidarCore::Init(AZ::EntityId entityId)
    {
        m_entityId = entityId;

        auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(m_entityId);
        m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);

        m_lastRotations = LidarTemplateUtils::PopulateRayRotations(m_lidarConfiguration.m_lidarParameters);

        m_lidarConfiguration.FetchLidarImplementationFeatures();
        ConnectToLidarRaycaster();
        ConfigureLidarRaycaster();
    }

    void LidarCore::Deinit()
    {
        for (auto& [implementation, raycasterId] : m_implementationToRaycasterMap)
        {
            LidarSystemRequestBus::Event(AZ_CRC(implementation), &LidarSystemRequestBus::Events::DestroyLidar, raycasterId);
        }

        m_implementationToRaycasterMap.clear();
    }

    LidarId LidarCore::GetLidarRaycasterId() const
    {
        return m_lidarRaycasterId;
    }

    RaycastResult LidarCore::PerformRaycast()
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, m_entityId);
        const auto entityTransform = entity->FindComponent<AzFramework::TransformComponent>();

        LidarRaycasterRequestBus::EventResult(
            m_lastScanResults, m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::PerformRaycast, entityTransform->GetWorldTM());
        if (m_lastScanResults.m_points.empty())
        {
            AZ_TracePrintf("Lidar Sensor Component", "No results from raycast\n");
            return RaycastResult();
        }
        return m_lastScanResults;
    }
} // namespace ROS2
