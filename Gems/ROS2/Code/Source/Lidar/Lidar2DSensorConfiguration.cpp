/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Lidar2DSensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void Lidar2DSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<Lidar2DSensorConfiguration>()
                ->Version(1)
                ->Field("lidarModel", &Lidar2DSensorConfiguration::m_lidarModel)
                ->Field("lidarImplementation", &Lidar2DSensorConfiguration::m_lidarSystem)
                ->Field("LidarParameters", &Lidar2DSensorConfiguration::m_lidarParameters)
                ->Field("IgnoredLayerIndices", &Lidar2DSensorConfiguration::m_ignoredCollisionLayers)
                ->Field("ExcludedEntities", &Lidar2DSensorConfiguration::m_excludedEntities)
                ->Field("PointsAtMax", &Lidar2DSensorConfiguration::m_addPointsAtMax);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<Lidar2DSensorConfiguration>("ROS2 Lidar 2D Sensor configuration", "Lidar 2D sensor configuration")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &Lidar2DSensorConfiguration::m_lidarModel, "Lidar Model", "Lidar model")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &Lidar2DSensorConfiguration::OnLidarModelSelected)
                    ->EnumAttribute(LidarTemplate::LidarModel::Custom2DLidar, "Custom Lidar 2D")
                    ->EnumAttribute(LidarTemplate::LidarModel::Slamtec_RPLIDAR_S1, "Slamtec RPLIDAR S1")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &Lidar2DSensorConfiguration::m_lidarSystem,
                        "Lidar Implementation",
                        "Select a lidar implementation out of registered ones.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &Lidar2DSensorConfiguration::OnLidarImplementationSelected)
                    ->Attribute(AZ::Edit::Attributes::StringList, &Lidar2DSensorConfiguration::FetchLidarSystemList)
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &Lidar2DSensorConfiguration::m_lidarParameters,
                        "Lidar parameters",
                        "Configuration of Custom lidar")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &Lidar2DSensorConfiguration::IsConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &Lidar2DSensorConfiguration::m_ignoredCollisionLayers,
                        "Ignored collision layers",
                        "Indices of collision layers to ignore")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &Lidar2DSensorConfiguration::IsIgnoredLayerConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &Lidar2DSensorConfiguration::m_excludedEntities,
                        "Excluded Entities",
                        "List of entities excluded from raycasting.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &Lidar2DSensorConfiguration::IsEntityExclusionVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &Lidar2DSensorConfiguration::m_addPointsAtMax,
                        "Points at Max",
                        "If set true LiDAR will produce points at max range for free space")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &Lidar2DSensorConfiguration::IsMaxPointsConfigurationVisible);
            }
        }
    }

    void Lidar2DSensorConfiguration::FetchLidarImplementationFeatures()
    {
        if (m_lidarSystem.empty())
        {
            m_lidarSystem = Details::GetDefaultLidarSystem();
        }
        const auto* lidarMetaData = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem);
        AZ_Warning("Lidar2DSensorConfiguration", lidarMetaData, "No metadata for \"%s\"", m_lidarSystem.c_str());
        if (lidarMetaData)
        {
            m_lidarSystemFeatures = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem)->m_features;
        }
    }

    bool Lidar2DSensorConfiguration::IsConfigurationVisible() const
    {
        return m_lidarModel == LidarTemplate::LidarModel::Custom2DLidar;
    }

    bool Lidar2DSensorConfiguration::IsIgnoredLayerConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers;
    }

    bool Lidar2DSensorConfiguration::IsEntityExclusionVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion;
    }

    bool Lidar2DSensorConfiguration::IsMaxPointsConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints;
    }

    AZ::Crc32 Lidar2DSensorConfiguration::OnLidarModelSelected()
    {
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
        UpdateShowNoise();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Crc32 Lidar2DSensorConfiguration::OnLidarImplementationSelected()
    {
        FetchLidarImplementationFeatures();
        UpdateShowNoise();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZStd::vector<AZStd::string> Lidar2DSensorConfiguration::FetchLidarSystemList()
    {
        FetchLidarImplementationFeatures();
        UpdateShowNoise();
        return LidarRegistrarInterface::Get()->GetRegisteredLidarSystems();
    }

    void Lidar2DSensorConfiguration::UpdateShowNoise()
    {
        m_lidarParameters.m_showNoiseConfig = m_lidarSystemFeatures & LidarSystemFeatures::Noise;
    }
} // namespace ROS2
