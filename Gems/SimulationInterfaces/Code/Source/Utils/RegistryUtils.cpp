/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RegistryUtils.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Settings/SettingsRegistry.h>

namespace ROS2SimulationInterfaces::RegistryUtilities
{
    const char* const RegistryKeySimulatorFrameId = "/SimulationInterfaces/SimulatorFrame";
    const char* const DefaultFrameId = "world";

    AZStd::optional<AZStd::string> GetName(const AZStd::string& serviceType)
    {
        AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get();
        AZ_Assert(settingsRegistry, "Settings Registry is not available");
        AZStd::string output = "";
        AZ::IO::Path setRegPath = AZ::IO::Path(RegistryPrefix) / AZ::IO::Path(serviceType);
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

    AZStd::string GetSimulatorROS2Frame()
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
