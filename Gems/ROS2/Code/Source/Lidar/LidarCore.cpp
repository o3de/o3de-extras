/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/AuxGeom/AuxGeomFeatureProcessorInterface.h>
#include <Atom/RPI.Public/Scene.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <Lidar/LidarCore.h>
#include <Lidar/LidarRegistrarSystemComponent.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <AzCore/base.h>

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

    RaycastResultFlags LidarCore::GetProvidedResultFlags(const LidarSensorConfiguration& configuration)
    {
        RaycastResultFlags flags = RaycastResultFlags::Range | RaycastResultFlags::Point | RaycastResultFlags::IsHit;
        if (configuration.m_lidarSystemFeatures & LidarSystemFeatures::Intensity)
        {
            flags |= RaycastResultFlags::Intensity;
        }

        if (configuration.m_lidarSystemFeatures & LidarSystemFeatures::Segmentation && configuration.m_isSegmentationEnabled)
        {
            if (ClassSegmentationInterface::Get())
            {
                flags |= RaycastResultFlags::SegmentationData;
            }
            else
            {
                AZ_Error(
                    "ROS2",
                    false,
                    "Segmentation feature was enabled for this lidar sensor but the segmentation interface is not accessible. Make sure to "
                    "either add the Class segmentation component to the level entity or disable the feature in the lidar component "
                    "configuration.");
            }
        }

        return flags;
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
            &LidarRaycasterRequestBus::Events::ConfigureRayRange,
            RayRange{ m_lidarConfiguration.m_lidarParameters.m_minRange, m_lidarConfiguration.m_lidarParameters.m_maxRange });

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

        LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureRaycastResultFlags, m_resultFlags);

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

        {
            const bool returnNonHits = IsFlagEnabled(RaycastResultFlags::IsHit, GetResultFlags()) || m_lidarConfiguration.m_addPointsAtMax || m_is2DLidar;
            LidarRaycasterRequestBus::Event(m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::ConfigureNonHitReturn, returnNonHits);
        }
    }

    void LidarCore::UpdatePoints(const RaycastResults& results)
    {
        const auto pointsField = results.GetConstFieldSpan<RaycastResultFlags::Point>().value();
        m_lastPoints.assign(pointsField.begin(), pointsField.end());
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
        if (m_lastPoints.empty())
        {
            return;
        }

        if (m_drawQueue)
        {
            const uint8_t pixelSize = 2;
            AZ::RPI::AuxGeomDraw::AuxGeomDynamicDrawArguments drawArgs;
            drawArgs.m_verts = m_lastPoints.data();
            drawArgs.m_vertCount = m_lastPoints.size();
            drawArgs.m_colors = &AZ::Colors::Red;
            drawArgs.m_colorCount = 1;
            drawArgs.m_opacityType = AZ::RPI::AuxGeomDraw::OpacityType::Opaque;
            drawArgs.m_size = pixelSize;
            m_drawQueue->DrawPoints(drawArgs);
        }
    }

    void LidarCore::Init(AZ::EntityId entityId, RaycastResultFlags requestedResultFlags, bool is2DLidar)
    {
        m_is2DLidar = is2DLidar;
        m_entityId = entityId;

        auto* entityScene = AZ::RPI::Scene::GetSceneForEntityId(m_entityId);
        m_drawQueue = AZ::RPI::AuxGeomFeatureProcessorInterface::GetDrawQueueForScene(entityScene);

        m_lastRotations = LidarTemplateUtils::PopulateRayRotations(m_lidarConfiguration.m_lidarParameters);

        m_lidarConfiguration.FetchLidarImplementationFeatures();
        m_resultFlags = GetProvidedResultFlags(m_lidarConfiguration) & requestedResultFlags;
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

    RaycastResultFlags LidarCore::GetResultFlags() const
    {
        return m_resultFlags;
    }

    AZStd::optional<RaycastResults> LidarCore::PerformRaycast()
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, m_entityId);
        const auto entityTransform = entity->FindComponent<AzFramework::TransformComponent>();

        AZ::Outcome<RaycastResults, const char*> results = AZ::Failure("EBus failure occurred.");
        LidarRaycasterRequestBus::EventResult(
            results, m_lidarRaycasterId, &LidarRaycasterRequestBus::Events::PerformRaycast, entityTransform->GetWorldTM());
        if (!results.IsSuccess())
        {
            AZ_Error(__func__, false, "Unable to obtain raycast results. %s", results.GetError());
            return {};
        }

        AZ_Warning("Lidar Sensor Component", !results.GetValue().IsEmpty(), "No results from raycast\n");

        UpdatePoints(results.GetValue());

        return results.GetValue();
    }
} // namespace ROS2
