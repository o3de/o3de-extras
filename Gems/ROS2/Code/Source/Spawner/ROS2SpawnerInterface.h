/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2SpawnPointComponent.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <ROS2/Spawner/SpawnerInterface.h>

namespace ROS2
{
    class ROS2SpawnerInterface : public SpawnerInterface::Registrar
    {
    public:
        ROS2SpawnerInterface() = default;
        const AZ::Transform GetDefaultSpawnPose() const override;

        const SpawnPointsNameAndPoseVector GetAllSpawnPoints() const override;

    private:
        void GetAllEntityDescendants(AZ::EntityId entityId, AZStd::vector<AZ::EntityId>& entityList) const;

        AZ::Transform m_defaultSpawnPose = { AZ::Vector3{ 0, 0, 0 }, AZ::Quaternion{ 0, 0, 0, 1 }, 1.0 };
    };
} // namespace ROS2
