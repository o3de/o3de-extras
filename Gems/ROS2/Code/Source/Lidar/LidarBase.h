/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>
#include <ROS2/Lidar/LidarSystemBus.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>

#include "LidarRaycaster.h"
#include "LidarSensorConfiguration.h"

namespace ROS2
{
    //! Lidar Base.
    //! Lidars (Light Detection and Ranging) emit laser light and measure it after reflection.
    //! Lidar Base allows for simulation of Lidars.
    class LidarBase
    {
    public:
        AZ_TYPE_INFO(LidarBase, "{e46126a2-7a86-bb65-367a-416f2cab393c}");
        static void Reflect(AZ::ReflectContext* context);

        LidarBase(const AZStd::vector<LidarTemplate::LidarModel>& availableModels = {});
        LidarBase(const LidarSensorConfiguration& lidarConfiguration);
        ~LidarBase() = default;

        void Init(AZ::EntityId entityId);
        void Deinit();

        RaycastResult PerformRaycast();
        void VisualizeResults() const;

        LidarId GetLidarRaycasterId() const;

        LidarSensorConfiguration m_lidarConfiguration;

    private:
        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        // A structure that maps each lidar implementation busId to the busId of a raycaster created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, LidarId> m_implementationToRaycasterMap;
        LidarId m_lidarRaycasterId;

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastRotations;
        RaycastResult m_lastScanResults;

        AZ::EntityId m_entityId;
    };
} // namespace ROS2
