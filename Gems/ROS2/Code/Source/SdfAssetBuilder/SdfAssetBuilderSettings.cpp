/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
#include <SdfAssetBuilder/SdfAssetBuilder.h>

#include <AzCore/Serialization/Json/JsonUtils.h>
#include <AzCore/Settings/SettingsRegistryVisitorUtils.h>

namespace ROS2
{
    namespace
    {
        struct SDFSettingsRootKeyType
        {
            using StringType = AZStd::fixed_string<256>;

            constexpr StringType operator()(AZStd::string_view name) const
            {
                constexpr size_t MaxTotalKeySize = StringType{}.max_size();
                // The +1 is for the '/' separator
                [[maybe_unused]] const size_t maxNameSize = MaxTotalKeySize - (SettingsPrefix.size() + 1);

                AZ_Assert(name.size() <= maxNameSize,
                    R"(The size of the event logger name "%.*s" is too long. It must be <= %zu characters)",
                    AZ_STRING_ARG(name), maxNameSize);
                StringType settingsKey(SettingsPrefix);
                settingsKey += '/';
                settingsKey += name;

                return settingsKey;
            }

            constexpr operator AZStd::string_view() const
            {
                return SettingsPrefix;
            }

        private:
            AZStd::string_view SettingsPrefix = "/O3DE/ROS2/SdfAssetBuilder";
        };

        constexpr SDFSettingsRootKeyType SDFSettingsRootKey;

        constexpr auto SdfAssetBuilderSupportedFileExtensionsRegistryKey = SDFSettingsRootKey("SupportedFileTypeExtensions");
        constexpr auto SdfAssetBuilderUseArticulationsRegistryKey = SDFSettingsRootKey("UseArticulations");
        constexpr auto SdfAssetBuilderURDFPreserveFixedJointRegistryKey = SDFSettingsRootKey("URDFPreserveFixedJoint");
    }

    void SdfAssetBuilderSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SdfAssetBuilderSettings>()
                ->Version(0)
                ->Field("UseArticulations", &SdfAssetBuilderSettings::m_useArticulations)
                ->Field("URDFPreserveFixedJoint", &SdfAssetBuilderSettings::m_urdfPreserveFixedJoints)

                // m_builderPatterns aren't serialized because we only use the serialization
                // to detect when global settings changes cause us to rebuild our assets.
                // A change to the builder patterns will cause the Asset Processor to add or
                // remove the affected product assets, so we don't need to trigger any
                // additional rebuilds beyond that.
             ;
        }
    }

    void SdfAssetBuilderSettings::LoadSettings(AZ::SettingsRegistryInterface* settingsRegistry)
    {
        if (settingsRegistry == nullptr)
        {
            AZ_Assert(settingsRegistry, "No settings registry provided, Sdf Asset Builder settings will use the defaults.");
            return;
        }

        // Set whether or not the outputs should use PhysX articulation components for joints.
        // Default to using articulations.
        settingsRegistry->Get(m_useArticulations, SdfAssetBuilderUseArticulationsRegistryKey);

        // Query the option to preserve child links of fixed joints when parsing a URDF using libsdformat
        settingsRegistry->Get(m_urdfPreserveFixedJoints, SdfAssetBuilderURDFPreserveFixedJointRegistryKey);

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
                        value == ".")
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
