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
        constexpr auto SdfAssetBuilderImportMeshesJointRegistryKey = SDFSettingsRootKey("ImportMeshes");
        constexpr auto SdfAssetBuilderFixURDFRegistryKey = SDFSettingsRootKey("FixURDF");
        constexpr auto SdfAssetBuilderAssetResolverRegistryKey = SDFSettingsRootKey("AssetResolverSettings");
    }

    void SdfAssetPathResolverSettings::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SdfAssetPathResolverSettings>()
                ->Version(0)
                ->Field("UseAmentPrefixPath", &SdfAssetPathResolverSettings::m_useAmentPrefixPath)
                ->Field("UseAncestorPaths", &SdfAssetPathResolverSettings::m_useAncestorPaths)
                ->Field("URIPrefixMap", &SdfAssetPathResolverSettings::m_uriPrefixMap)
             ;

            if (auto editContext = serializeContext->GetEditContext(); editContext != nullptr)
            {
                editContext
                    ->Class<SdfAssetPathResolverSettings>(
                        "Asset Paths", "Exposes settings for resolving asset path references")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetPathResolverSettings::m_useAmentPrefixPath,
                        "Use AMENT_PREFIX_PATH",
                        "Uses the AMENT_PREFIX_PATH environment variable to try and locate asset references")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetPathResolverSettings::m_useAncestorPaths,
                        "Search parent paths",
                        "Tries to resolve partial paths by traversing parent folders to look for partial path matches")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetPathResolverSettings::m_uriPrefixMap,
                        "Prefix replacements",
                        "Map path prefixes to specific paths (ex: 'model://' -> 'Assets/models')");
            }
        }
    }

    void SdfAssetBuilderSettings::Reflect(AZ::ReflectContext* context)
    {
        SdfAssetPathResolverSettings::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SdfAssetBuilderSettings>()
                ->Version(1)
                ->Field("UseArticulations", &SdfAssetBuilderSettings::m_useArticulations)
                ->Field("URDFPreserveFixedJoint", &SdfAssetBuilderSettings::m_urdfPreserveFixedJoints)
                ->Field("ImportReferencedMeshFiles", &SdfAssetBuilderSettings::m_importReferencedMeshFiles)
                ->Field("FixURDF", &SdfAssetBuilderSettings::m_fixURDF)
                ->Field("AssetResolverSettings", &SdfAssetBuilderSettings::m_resolverSettings)

                // m_builderPatterns aren't serialized because we only use the serialization
                // to detect when global settings changes cause us to rebuild our assets.
                // A change to the builder patterns will cause the Asset Processor to add or
                // remove the affected product assets, so we don't need to trigger any
                // additional rebuilds beyond that.
             ;

            if (auto editContext = serializeContext->GetEditContext(); editContext != nullptr)
            {
                editContext
                    ->Class<SdfAssetBuilderSettings>(
                        "URDF/SDF Asset Import Settings", "Exposes settings which alters importing of URDF/XACRO/SDF files.")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                            ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetBuilderSettings::m_useArticulations,
                        "Use Articulations",
                        "Determines whether PhysX articulation components should be used for joints and rigid bodies.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetBuilderSettings::m_urdfPreserveFixedJoints,
                        "Preserve URDF fixed joints",
                        "When set, preserves any fixed joints found when importing a URDF file."
                        " This prevents the joint reduction logic in libsdformat from merging links of those joints.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetBuilderSettings::m_importReferencedMeshFiles,
                        "Import meshes",
                        "Allows importing of referenced mesh content files such as .dae or .stl files when importing the URDF/SDF.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetBuilderSettings::m_fixURDF,
                        "Fix URDF to be compatible with libsdformat",
                        "When set, fixes the URDF file before importing it. This is useful for fixing URDF files that have missing inertials or duplicate names within links and joints.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SdfAssetBuilderSettings::m_resolverSettings,
                        "Path Resolvers",
                        "Determines how to resolve any partial asset paths.")
                        ;
            }
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

        // Query the import references meshes option from the Settings Registry to determine if mesh source assets are copied
        settingsRegistry->Get(m_importReferencedMeshFiles, SdfAssetBuilderImportMeshesJointRegistryKey);

        // Query the fix URDF option from the Settings Registry to determine if the URDF file should be fixed before importing
        settingsRegistry->Get(m_fixURDF, SdfAssetBuilderFixURDFRegistryKey);

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

        // Get the Asset Resolver settings
        settingsRegistry->GetObject(m_resolverSettings, SdfAssetBuilderAssetResolverRegistryKey);

        AZ_Warning(SdfAssetBuilderName, !m_builderPatterns.empty(), "SdfAssetBuilder disabled, no supported file type extensions found.");
    }
} // ROS2
