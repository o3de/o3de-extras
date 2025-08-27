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

namespace ROS2SimulationInterfaces::RegistryUtilities
{
    namespace
    {
        // prefix for settings registry related to ros2 topics names
        inline constexpr const char* ServiceNameRegistryPrefix = "/ROS2SimulationInterfaces";
        const char* const RegistryKeySimulatorFrameId = "/SimulationInterfaces/SimulatorFrame";
        const char* const DefaultFrameId = "world";
    } // namespace

    //! Gets name of the service with defined type form settings registry
    //! @return optional string with service name. If setting registry entry exists, its value is returned.
    //!         Otherwise AZStd::nullopt is returneds
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
        else
        {
            return AZStd::nullopt;
        }
    }

    //! Get the default frame name for the simulator in ROS2
    [[nodiscard]] inline AZStd::string GetSimulatorROS2Frame()
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry is not available");
        AZStd::string frameId;
        if (!settingsRegistry->Get(frameId, RegistryKeySimulatorFrameId))
        {
            frameId = DefaultFrameId;
            AZ_Warning(
                "Simulation interfaces", false, "No simulator frame ID found in settings registry, using default: %s", frameId.c_str());
        }
        return frameId;
    }

} // namespace ROS2SimulationInterfaces::RegistryUtilities
