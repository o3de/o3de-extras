/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{
    //!  Interface class allowing requesting Spawner interface from other components.
    class SpawnerRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(SpawnerRequests, "{3C42A3A1-1B8E-4800-9473-E4441315D7C8}");

        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;

        virtual ~SpawnerRequests() = default;

        //! Default spawn pose getter
        //! @return default spawn point coordinates set by user in Editor (by default: translation: {0, 0, 0}, rotation: {0, 0, 0, 1},
        //! scale: 1.0)
        virtual const AZ::Transform& GetDefaultSpawnPose() const = 0;

        virtual AZStd::unordered_map<AZStd::string, SpawnPointInfo> GetAllSpawnPointInfos() const = 0;
    };

    using SpawnerRequestsBus = AZ::EBus<SpawnerRequests>;
} // namespace ROS2
