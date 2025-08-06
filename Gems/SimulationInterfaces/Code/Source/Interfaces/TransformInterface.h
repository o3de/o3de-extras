/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
namespace ROS2SimulationInterfaces
{
    class TFInterfaceRequests
    {
    public:
        AZ_RTTI(TFInterfaceRequests, TFInterfaceTypeId);
        //! Gets transform from source frame to target frame, at most recent time
        //! @param source - source frame name
        //! @param target - target frame name
        //! @param time - time to get the transform at, if not provided, uses most recent time
        //! @return AZ::Outcome with transform if successful, or error message if failed
        virtual AZ::Outcome<AZ::Transform, AZStd::string> GetTransform(
            const AZStd::string& source, const AZStd::string& target, const builtin_interfaces::msg::Time& time) = 0;
    };
    using TFInterface = AZ::Interface<TFInterfaceRequests>;
} // namespace ROS2SimulationInterfaces
