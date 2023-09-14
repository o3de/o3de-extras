/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/unordered_map.h>
#include <AssetBuilderSDK/AssetBuilderSDK.h>
#include <AzCore/Settings/SettingsRegistry.h>

namespace ROS2
{
    struct SdfAssetPathResolverSettings
    {
    public:
        AZ_RTTI(SdfAssetPathResolverSettings, "{51EDDB99-FE82-4783-9C91-7DF403AD4EFA}");

        SdfAssetPathResolverSettings() = default;
        virtual ~SdfAssetPathResolverSettings() = default;

        static void Reflect(AZ::ReflectContext* context);

        using UriPrefixMap = AZStd::unordered_map<AZStd::string, AZStd::vector<AZStd::string>>;

        //! When true, use the set of paths in the AMENT_PREFIX_PATH environment variable to search for files
        bool m_useAmentPrefixPath = true;
        //! When true, search ancestor paths all the way up to the root to search for files
        bool m_useAncestorPaths = true; 
        //! The map of URI prefixes to replace with paths
        UriPrefixMap m_uriPrefixMap;
    };

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
        //! By default, fixed joint in URDF files that are processed by libsdformat are preserved
        bool m_urdfPreserveFixedJoints = true;
        //! When true, .dae/.stl mesh files are imported into the project folder to allow the AP to process them
        bool m_importReferencedMeshFiles = true;
        //! When true URDF will be fixed to be compatible with SDFormat.
        bool m_fixURDF = true;

        SdfAssetPathResolverSettings m_resolverSettings;
    };
} // namespace ROS2
