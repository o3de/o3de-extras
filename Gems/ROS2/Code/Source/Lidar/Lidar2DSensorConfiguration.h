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

#include "LidarRegistrarSystemComponent.h"
#include "LidarTemplate.h"
#include "LidarTemplateUtils.h"

namespace ROS2
{
    //! A structure capturing configuration of a lidar sensor (to be used with ROS2Lidar2DSensorComponent).
    class Lidar2DSensorConfiguration
    {
    public:
        AZ_TYPE_INFO(Lidar2DSensorConfiguration, "{ed89dd3c-81ef-4636-9907-7ea641f8fbb0}");
        static void Reflect(AZ::ReflectContext* context);

        void FetchLidarImplementationFeatures();

        LidarSystemFeatures m_lidarSystemFeatures;

        AZStd::string m_lidarSystem;
        LidarTemplate::LidarModel m_lidarModel = LidarTemplate::LidarModel::Custom2DLidar;
        LidarTemplate m_lidarParameters = LidarTemplateUtils::GetTemplate(LidarTemplate::LidarModel::Custom2DLidar);

        bool m_ignoreLayer = false;
        AZ::u32 m_ignoredLayerIndex = 0;
        AZStd::vector<AZ::EntityId> m_excludedEntities;

        bool m_addPointsAtMax = false;

    private:
        bool IsConfigurationVisible() const;
        bool IsIgnoredLayerConfigurationVisible() const;
        bool IsEntityExclusionVisible() const;
        bool IsMaxPointsConfigurationVisible() const;

        AZ::Crc32 OnLidarModelSelected();
        AZ::Crc32 OnLidarImplementationSelected();
        AZStd::vector<AZStd::string> FetchLidarSystemList();

        void UpdateShowNoise();
    };
} // namespace ROS2
