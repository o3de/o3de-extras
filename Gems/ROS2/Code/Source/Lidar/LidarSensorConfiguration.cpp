/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LidarSensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void LidarSensorConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarSensorConfiguration>()
                ->Version(1)
                ->Field("lidarModel", &LidarSensorConfiguration::m_lidarModel)
                ->Field("lidarImplementation", &LidarSensorConfiguration::m_lidarSystem)
                ->Field("LidarParameters", &LidarSensorConfiguration::m_lidarParameters)
                ->Field("IgnoreLayer", &LidarSensorConfiguration::m_ignoreLayer)
                ->Field("IgnoredLayerIndex", &LidarSensorConfiguration::m_ignoredLayerIndex)
                ->Field("ExcludedEntities", &LidarSensorConfiguration::m_excludedEntities)
                ->Field("PointsAtMax", &LidarSensorConfiguration::m_addPointsAtMax);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<LidarSensorConfiguration>("ROS2 Lidar Sensor configuration", "Lidar sensor configuration")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &LidarSensorConfiguration::m_lidarModel, "Lidar Model", "Lidar model")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &LidarSensorConfiguration::OnLidarModelSelected)
                    ->EnumAttribute(LidarTemplate::LidarModel::Custom3DLidar, "Custom Lidar")
                    ->EnumAttribute(LidarTemplate::LidarModel::Ouster_OS0_64, "Ouster OS0-64")
                    ->EnumAttribute(LidarTemplate::LidarModel::Ouster_OS1_64, "Ouster OS1-64")
                    ->EnumAttribute(LidarTemplate::LidarModel::Ouster_OS2_64, "Ouster OS2-64")
                    ->EnumAttribute(LidarTemplate::LidarModel::Velodyne_Puck, "Velodyne Puck (VLP-16)")
                    ->EnumAttribute(LidarTemplate::LidarModel::Velodyne_HDL_32E, "Velodyne HDL-32E")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &LidarSensorConfiguration::m_lidarSystem,
                        "Lidar Implementation",
                        "Select a lidar implementation out of registered ones.")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &LidarSensorConfiguration::OnLidarImplementationSelected)
                    ->Attribute(AZ::Edit::Attributes::StringList, &LidarSensorConfiguration::FetchLidarSystemList)
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &LidarSensorConfiguration::m_lidarParameters,
                        "Lidar parameters",
                        "Configuration of Custom lidar")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarSensorConfiguration::IsConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &LidarSensorConfiguration::m_ignoreLayer,
                        "Ignore layer",
                        "Should we ignore selected layer index")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarSensorConfiguration::IsIgnoredLayerConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarSensorConfiguration::m_ignoredLayerIndex,
                        "Ignored layer index",
                        "Layer index to ignore")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarSensorConfiguration::IsIgnoredLayerConfigurationVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarSensorConfiguration::m_excludedEntities,
                        "Excluded Entities",
                        "List of entities excluded from raycasting.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, true)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarSensorConfiguration::IsEntityExclusionVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarSensorConfiguration::m_addPointsAtMax,
                        "Points at Max",
                        "If set true LiDAR will produce points at max range for free space")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarSensorConfiguration::IsMaxPointsConfigurationVisible);
            }
        }
    }

    void LidarSensorConfiguration::FetchLidarImplementationFeatures()
    {
        if (m_lidarSystem.empty())
        {
            m_lidarSystem = Details::GetDefaultLidarSystem();
        }
        const auto* lidarMetaData = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem);
        AZ_Warning("LidarSensorConfiguration", lidarMetaData, "No metadata for \"%s\"", m_lidarSystem.c_str());
        if (lidarMetaData)
        {
            m_lidarSystemFeatures = LidarRegistrarInterface::Get()->GetLidarSystemMetaData(m_lidarSystem)->m_features;
        }
    }

    bool LidarSensorConfiguration::IsConfigurationVisible() const
    {
        return m_lidarModel == LidarTemplate::LidarModel::Custom3DLidar;
    }

    bool LidarSensorConfiguration::IsIgnoredLayerConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::CollisionLayers;
    }

    bool LidarSensorConfiguration::IsEntityExclusionVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::EntityExclusion;
    }

    bool LidarSensorConfiguration::IsMaxPointsConfigurationVisible() const
    {
        return m_lidarSystemFeatures & LidarSystemFeatures::MaxRangePoints;
    }

    AZ::Crc32 LidarSensorConfiguration::OnLidarModelSelected()
    {
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
        UpdateShowNoise();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Crc32 LidarSensorConfiguration::OnLidarImplementationSelected()
    {
        FetchLidarImplementationFeatures();
        UpdateShowNoise();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZStd::vector<AZStd::string> LidarSensorConfiguration::FetchLidarSystemList()
    {
        FetchLidarImplementationFeatures();
        UpdateShowNoise();
        return LidarRegistrarInterface::Get()->GetRegisteredLidarSystems();
    }

    void LidarSensorConfiguration::UpdateShowNoise()
    {
        m_lidarParameters.m_showNoiseConfig = m_lidarSystemFeatures & LidarSystemFeatures::Noise;
    }
} // namespace ROS2
