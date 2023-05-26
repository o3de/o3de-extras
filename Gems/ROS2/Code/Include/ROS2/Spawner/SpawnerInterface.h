/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/RTTI.h>

namespace ROS2
{
    using SpawnPointsNameAndPoseVector = AZStd::vector<AZStd::pair<AZStd::string, AZStd::shared_ptr<AZ::Transform>>>;

    class ISpawner
    {
    public:
        AZ_RTTI(ISpawner, "{9334c9fa-f642-11ed-b67e-0242ac120002}");
        ISpawner() = default;
        virtual ~ISpawner() = default;

        //! Default spawn pose getter
        //! @return default spawn point coordinates set by user in Editor (by default: translation: {0, 0, 0}, rotation: {0, 0, 0, 1},
        //! scale: 1.0)
        virtual const AZ::Transform GetDefaultSpawnPose() const = 0;

        //! All available spawn pose getter
        //! @return vector of all available spawn positions from all entities having a ROS2 spawn point component.
        //! This vector consists of pairs with the entity name and its pose.
        virtual const SpawnPointsNameAndPoseVector GetAllSpawnPoints() const = 0;
    };

    using SpawnerInterface = AZ::Interface<ISpawner>;
} // namespace ROS2