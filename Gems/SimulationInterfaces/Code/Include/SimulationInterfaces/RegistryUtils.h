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
        const char* const RegistryKeySimulatorFrameId = "/SimulationInterfaces/SimulatorFrame";
        const char* const DefaultFrameId = "world";
    } // namespace

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
