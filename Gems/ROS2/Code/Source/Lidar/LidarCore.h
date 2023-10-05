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

#include "LidarRaycaster.h"
#include "LidarSensorConfiguration.h"

namespace ROS2
{
    //! A class for executing lidar operations, such as data acquisition and visualization.
    class LidarCore
    {
    public:
        AZ_TYPE_INFO(LidarCore, "{e46126a2-7a86-bb65-367a-416f2cab393c}");
        static void Reflect(AZ::ReflectContext* context);

        LidarCore(const AZStd::vector<LidarTemplate::LidarModel>& availableModels = {});
        LidarCore(const LidarSensorConfiguration& lidarConfiguration);
        ~LidarCore() = default;

        //! Initialize when activating the lidar.
        //! @param entityId Entity from which the rays are sent.
        void Init(AZ::EntityId entityId);
        //! Deinitialize when deactivating the lidar.
        void Deinit();

        //! Perform a raycast.
        //! @return Results of the raycast.
        RaycastResult PerformRaycast();
        //! Visualize the results of the last performed raycast.
        void VisualizeResults() const;

        //! Get the raycaster used by this lidar.
        //! @return Used raycaster's id.
        LidarId GetLidarRaycasterId() const;

        //! Configuration according to which the lidar performs its raycasts.
        LidarSensorConfiguration m_lidarConfiguration;

    private:
        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        //! An unordered map of lidar implementations to their raycasters created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, LidarId> m_implementationToRaycasterMap;
        LidarId m_lidarRaycasterId;

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastRotations;
        RaycastResult m_lastScanResults;

        AZ::EntityId m_entityId;
    };
} // namespace ROS2
