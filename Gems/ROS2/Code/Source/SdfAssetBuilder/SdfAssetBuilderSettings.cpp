/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Settings/SettingsRegistryVisitorUtils.h>

namespace ROS2
{
        namespace
        {
            constexpr const char* SdfAssetBuilderSupportedFileExtensionsRegistryKey = "/O3DE/ROS2/SdfAssetBuilder/SupportedFileTypeExtensions";
            constexpr const char* SdfAssetBuilderUseArticulationsRegistryKey = "/O3DE/ROS2/SdfAssetBuilder/UseArticulations";
        }

    void SdfAssetBuilderSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SdfAssetBuilderSettings>()
                ->Version(0)
                ->Field("UseArticulations", &SdfAssetBuilderSettings::m_useArticulations)

                // m_builderPatterns aren't serialized because we only use the serialization
                // to detect when global settings changes cause us to rebuild our assets.
                // A change to the builder patterns will cause the Asset Processor to add or
                // remove the affected product assets, so we don't need to trigger any 
                // additional rebuilds beyond that.
             ;
        }
    }

    void SdfAssetBuilderSettings::LoadGlobalSettings()
    {
        auto settingsRegistry = AZ::SettingsRegistry::Get();
        if (settingsRegistry == nullptr)
        {
            AZ_Error(SdfAssetBuilderName, false, "Settings Registry not found, Sdf Asset Builder settings will use defaults.");
            return;
        }

        // Set whether or not the outputs should use PhysX articulation components for joints.
        // Default to using articulations.
        settingsRegistry->Get(m_useArticulations, SdfAssetBuilderUseArticulationsRegistryKey);

        // Visit each supported file type extension and create an asset builder wildcard pattern for it.
        auto VisitFileTypeExtensions = [&settingsRegistry, this]
            (const AZ::SettingsRegistryInterface::VisitArgs& visitArgs)
            {
                if (AZ::SettingsRegistryInterface::FixedValueString value;
                    settingsRegistry->Get(value, visitArgs.m_jsonKeyPath))
                {
                    // Ignore any entries that are either completely empty or *only* contain a '.'.
                    // These will produce excessive (and presumably incorrect) wildcard matches.
                    if (value.empty() ||
                        ((value.size() == 1) && value.starts_with('.')))
                    {
                        return AZ::SettingsRegistryInterface::VisitResponse::Continue;
                    }

                    // Support both 'sdf' and '.sdf' style entries in the setreg file for robustness.
                    // Either one will get turned into a '*.sdf' pattern.
                    AZStd::string wildcardPattern = value.starts_with('.')
                        ? AZStd::string::format("*%s", value.c_str())
                        : AZStd::string::format("*.%s", value.c_str());

                    m_builderPatterns.push_back(
                            AssetBuilderSDK::AssetBuilderPattern(
                                wildcardPattern, AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard));
                }
                return AZ::SettingsRegistryInterface::VisitResponse::Continue;
            };
        AZ::SettingsRegistryVisitorUtils::VisitArray(*settingsRegistry, VisitFileTypeExtensions, SdfAssetBuilderSupportedFileExtensionsRegistryKey);

        AZ_Warning(SdfAssetBuilderName, !m_builderPatterns.empty(), "SdfAssetBuilder disabled, no supported file type extensions found.");
    }
} // ROS2
