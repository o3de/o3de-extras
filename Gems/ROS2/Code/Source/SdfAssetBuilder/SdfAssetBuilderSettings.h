/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AzCore/Settings/SettingsRegistry.h>

namespace ROS2
{
    struct SdfAssetBuilderSettings
    {
    public:
        AZ_RTTI(SdfAssetBuilderSettings, "{7FE3FA92-E274-4624-8905-61E1587DDD30}");

        SdfAssetBuilderSettings() = default;
        virtual ~SdfAssetBuilderSettings() = default;

        static void Reflect(AZ::ReflectContext* context);

        //! Read in the builder settings from the settings registry.
        //! This is done as a separate step from the constructor so that serialization has
        //! the ability to create a default set of values by using the default constructor.
        //! If we read these in the constructor, serialization would see all of the current values
        //! as default values and would try to prune them from the output by default.
        void LoadSettings(AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get());

        AZStd::vector<AssetBuilderSDK::AssetBuilderPattern> m_builderPatterns;
        bool m_useArticulations = true;
    };
} // ROS2
