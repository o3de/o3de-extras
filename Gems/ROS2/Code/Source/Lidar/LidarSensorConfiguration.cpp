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
                ->Field("lidarModelName", &LidarSensorConfiguration::m_lidarModelName)
                ->Field("lidarImplementation", &LidarSensorConfiguration::m_lidarSystem)
                ->Field("LidarParameters", &LidarSensorConfiguration::m_lidarParameters)
                ->Field("IgnoredLayerIndices", &LidarSensorConfiguration::m_ignoredCollisionLayers)
                ->Field("ExcludedEntities", &LidarSensorConfiguration::m_excludedEntities)
                ->Field("PointsAtMax", &LidarSensorConfiguration::m_addPointsAtMax);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<LidarSensorConfiguration>("Lidar Sensor configuration", "Lidar sensor configuration")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &LidarSensorConfiguration::m_lidarModelName, "Lidar Model", "Lidar model")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &LidarSensorConfiguration::OnLidarModelSelected)
                    ->Attribute(AZ::Edit::Attributes::StringList, &LidarSensorConfiguration::GetAvailableModels)
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
                        AZ::Edit::UIHandlers::Default,
                        &LidarSensorConfiguration::m_ignoredCollisionLayers,
                        "Ignored collision layers",
                        "Indices of collision layers to ignore")
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

    LidarSensorConfiguration::LidarSensorConfiguration(AZStd::vector<LidarTemplate::LidarModel> availableModels)
        : m_availableModels(AZStd::move(availableModels))
    {
        if (m_availableModels.empty())
        {
            AZ_Warning("LidarSensorConfiguration", false, "Lidar configuration created with an empty models list");
            return;
        }
        m_lidarModel = m_availableModels.front();
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
        m_lidarModelName = m_lidarParameters.m_name;
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

    AZStd::vector<AZStd::string> LidarSensorConfiguration::GetAvailableModels() const
    {
        AZStd::vector<AZStd::string> result;
        for (const auto model : m_availableModels)
        {
            auto templ = LidarTemplateUtils::GetTemplate(model);
            result.push_back({ templ.m_name });
        }
        return result;
    }

    void LidarSensorConfiguration::FetchLidarModelConfiguration()
    {
        for (const auto model : m_availableModels)
        {
            auto templ = LidarTemplateUtils::GetTemplate(model);
            if (m_lidarModelName == templ.m_name)
            {
                m_lidarModel = model;
                break;
            }
        }
        m_lidarParameters = LidarTemplateUtils::GetTemplate(m_lidarModel);
    }

    bool LidarSensorConfiguration::IsConfigurationVisible() const
    {
        return m_lidarModel == LidarTemplate::LidarModel::Custom3DLidar || m_lidarModel == LidarTemplate::LidarModel::Custom2DLidar;
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
        FetchLidarModelConfiguration();
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
