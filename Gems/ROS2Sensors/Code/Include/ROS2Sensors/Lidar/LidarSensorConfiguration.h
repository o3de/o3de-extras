/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/Lidar/LidarRegistrarBus.h>
#include <ROS2Sensors/Lidar/LidarTemplate.h>
#include <ROS2Sensors/Lidar/LidarTemplateUtils.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! A structure capturing configuration of a lidar sensor (to be used with LidarCore).
    class LidarSensorConfiguration
    {
    public:
        AZ_TYPE_INFO(LidarSensorConfiguration, LidarSensorConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        LidarSensorConfiguration(AZStd::vector<LidarTemplate::LidarModel> availableModels = {});

        //! Update the lidar system features based on the current lidar system selected.
        void FetchLidarImplementationFeatures();

        //! Update the lidar configuration based on the current lidar model selected.
        void FetchLidarModelConfiguration();

        LidarSystemFeatures m_lidarSystemFeatures;

        AZStd::string m_lidarSystem;
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::LidarModel::Custom2DLidar;
        LidarTemplate m_lidarParameters = LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel::Custom2DLidar);
        AZStd::string m_lidarModelName = "CustomLidar2D";

        AZStd::unordered_set<AZ::u32> m_ignoredCollisionLayers;
        AZStd::vector<AZ::EntityId> m_excludedEntities;

        bool m_isSegmentationEnabled = false;
        bool m_addPointsAtMax = false;

    private:
        bool IsConfigurationVisible() const;
        bool IsIgnoredLayerConfigurationVisible() const;
        bool IsEntityExclusionVisible() const;
        bool IsMaxPointsConfigurationVisible() const;
        bool IsSegmentationConfigurationVisible() const;

        AZ::Crc32 OnLidarModelSelected();
        AZ::Crc32 OnLidarImplementationSelected();
        void UpdateShowNoise();
        AZStd::string GetDefaultLidarSystem();

        //! Get all models this configuration can be set to (for example all 2D lidar models).
        AZStd::vector<AZStd::string> GetAvailableModels() const;
        //! Get all available lidar systems.
        AZStd::vector<AZStd::string> FetchLidarSystemList();

        AZStd::vector<LidarTemplate::LidarModel> m_availableModels;
    };
} // namespace ROS2Sensors
