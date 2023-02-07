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

        ROS2SpawnPointComponent() = default;

        ~ROS2SpawnPointComponent() = default;
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////
        static void Reflect(AZ::ReflectContext* context);

        AZStd::pair<AZStd::string, SpawnPointInfo> GetInfo() const;

    private:
        AZStd::string m_name;
        AZStd::string m_info;
    };
} // namespace ROS2
