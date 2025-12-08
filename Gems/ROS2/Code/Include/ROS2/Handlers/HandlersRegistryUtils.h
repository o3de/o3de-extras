/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>

namespace ROS2::HandlersRegistryUtils
{
    namespace
    {
        // prefix for settings registry related to ros2 handlers names
        inline constexpr const char* ServiceNameRegistryPrefix = "/ROS2/HandlersNames";
    } // namespace

    //! Gets name of the service with defined type from settings registry
    //! @return optional string with service name. If setting registry entry exists, its value is returned.
    //!         Otherwise AZStd::nullopt is returned
    [[nodiscard]] inline AZStd::optional<AZStd::string> GetName(const AZStd::string& serviceType)
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry is not available");
        AZStd::string output = "";
        AZ::IO::Path setRegPath = AZ::IO::Path(ServiceNameRegistryPrefix) / AZ::IO::Path(serviceType);
        const auto setRegStatus = settingsRegistry->Get(output, setRegPath.String());
        if (setRegStatus) // value gathering from settings registry succeeded
        {
            return output;
        }

        return AZStd::nullopt;
    }

} // namespace ROS2::HandlersRegistryUtils
