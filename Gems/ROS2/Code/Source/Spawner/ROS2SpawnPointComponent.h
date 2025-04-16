/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "Spawner/ROS2SpawnPointComponentController.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <ROS2/Spawner/SpawnerInfo.h>

namespace ROS2
{

    using ROS2SpawnPointComponentBase =
        AzFramework::Components::ComponentAdapter<ROS2SpawnPointComponentController, ROS2SpawnPointComponentConfig>;

    //! SpawnPoint indicates a place which is suitable to spawn a robot.
    class ROS2SpawnPointComponent : public ROS2SpawnPointComponentBase
    {
    public:
        AZ_COMPONENT(ROS2SpawnPointComponent, "{422c0495-5bbf-4207-ac17-8e607c6d3b30}", AZ::Component);

        ROS2SpawnPointComponent() = default;
        ROS2SpawnPointComponent(const ROS2SpawnPointComponentConfig& config);
        ~ROS2SpawnPointComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // ROS2SpawnPointComponentBase overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        AZStd::pair<AZStd::string, SpawnPointInfo> GetInfo() const;
    };
} // namespace ROS2
