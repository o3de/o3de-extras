/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "ROS2/Lidar/LidarRaycasterBus.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzFramework/Physics/PhysicsScene.h>

namespace ROS2
{
    //! A simple implementation of Lidar operation in terms of raycasting.
    class LidarRaycaster : protected LidarRaycasterRequestBus::Handler
    {
    public:
        explicit LidarRaycaster(const AZ::Uuid& uuid);
        LidarRaycaster(LidarRaycaster&& lidarSystem) noexcept;
        LidarRaycaster(const LidarRaycaster& lidarSystem) = delete;
        ~LidarRaycaster() override;

        //! Set the Scene for the ray-casting.
        //! This should be the scene with the Entity that holds the sensor.
        //! @code
        //! auto sceneHandle = AZ::RPI::Scene::GetSceneForEntityId(GetEntityId());
        //! @endcode
        //! @param handle Scene that will be subject to ray-casting.
        void SetRaycasterScene(const AzPhysics::SceneHandle& handle);

    protected:
        ////////////////////////////////////////////////////////////////////////
        // LidarRaycasterRequestBus::Handler interface implementation
        void ConfigureRayOrientations(const AZStd::vector<AZ::Vector3>& orientations) override;
        void ConfigureRayRange(float range) override;
        AZStd::vector<AZ::Vector3> PerformRaycast(const AZ::Transform& lidarTransform) override;
        void ConfigureLayerIgnoring(bool ignoreLayer, unsigned int layerIndex) override;
        void ConfigureMaxRangePointAddition(bool addMaxRangePoints) override;
        ////////////////////////////////////////////////////////////////////////

    private:
        AZ::Uuid m_uuid;
        AzPhysics::SceneHandle m_sceneHandle{ AzPhysics::InvalidSceneHandle };

        float m_range{ 1.0f };
        bool m_addMaxRangePoints{ false };
        AZStd::vector<AZ::Vector3> m_rayRotations{ { AZ::Vector3::CreateZero() } };

        bool m_ignoreLayer{ false };
        unsigned int m_ignoredLayerIndex{ 0 };
    };
} // namespace ROS2
