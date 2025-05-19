/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "LidarRaycaster.h"
#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2Sensors/Lidar/LidarRegistrarBus.h>
#include <ROS2Sensors/Lidar/LidarSensorConfiguration.h>
#include <ROS2Sensors/Lidar/LidarSystemBus.h>

namespace ROS2Sensors
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
        AZStd::optional<RaycastResults> PerformRaycast();
        //! Visualize the results of the last performed raycast.
        void VisualizeResults() const;

        //! Get the raycaster used by this lidar.
        //! @return Used raycaster's id.
        LidarId GetLidarRaycasterId() const;

        //! Get the result flags used by this lidar.
        //! @return Used result flags.
        RaycastResultFlags GetResultFlags() const;

        //! Configuration according to which the lidar performs its raycasts.
        //! Note: the configuration can be changed only when the lidar is not active.
        LidarSensorConfiguration m_lidarConfiguration;

    private:
        static RaycastResultFlags GetRaycastResultFlagsForConfig(const LidarSensorConfiguration& configuration);

        void ConnectToLidarRaycaster();
        void ConfigureLidarRaycaster();

        void UpdatePoints(const RaycastResults& results);

        //! An unordered map of lidar implementations to their raycasters created by this LidarSensorComponent.
        AZStd::unordered_map<AZStd::string, LidarId> m_implementationToRaycasterMap;
        LidarId m_lidarRaycasterId;

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;

        AZStd::vector<AZ::Vector3> m_lastRotations;
        AZStd::vector<AZ::Vector3> m_lastPoints;

        AZ::EntityId m_entityId;
        RaycastResultFlags m_resultFlags;
    };
} // namespace ROS2Sensors
