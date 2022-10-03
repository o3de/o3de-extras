/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>

namespace ROS2
{
    struct SpawnPointInfo
    {
        AZStd::string info;
        AZ::Transform pose;
    };

    //! SpawnPoint indicates a place which is suitable to spawn a robot.
    class ROS2SpawnPointComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2SpawnPointComponent, "{2AE1CAAE-B300-49FD-8F6D-F7AAABED1EC3}", AZ::Component);

        // AZ::Component interface implementation.
        ROS2SpawnPointComponent() = default;

        ~ROS2SpawnPointComponent() = default;

        void Activate() override;

        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);

        AZStd::pair<AZStd::string, SpawnPointInfo> GetInfo();

    private:
        AZStd::string m_name;
        AZStd::string m_info;
    };
} // namespace ROS2
