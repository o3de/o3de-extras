/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <openxr/openxr.h>

#include <AzCore/Asset/AssetCommon.h>
#include <AzFramework/Asset/GenericAssetHandler.h>

namespace OpenXRVk
{
    class OpenXRInteractionComponentPathDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionComponentPathDescriptor, "{E2038854-929D-484F-A34E-1C7390EE2CCB}");
        virtual ~OpenXRInteractionComponentPathDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        static constexpr AZStd::string_view s_TypeBoolStr = "Boolean";
        static constexpr AZStd::string_view s_TypeFloatStr = "Float";
        static constexpr AZStd::string_view s_TypeVector2Str = "Vector2";
        static constexpr AZStd::string_view s_TypePoseStr = "Pose";
        static constexpr AZStd::string_view s_TypeVibrationStr = "Vibration";

        //! Helper method
        static XrActionType GetXrActionType(AZStd::string_view actionTypeStr);
        XrActionType GetXrActionType() const;

        //! A user friendly name.
        AZStd::string m_name;
        //! For OpenXR a Component Path string would look like:
        //! "/input/x/click", or "/input/trigger/value", etc
        AZStd::string m_path;
        //! Whether this is a boolean, float, vector2 or pose.
        //! The user will be presented with a combo box to avoid
        //! chances for error.
        AZStd::string m_actionTypeStr;

    private:
        AZStd::string GetEditorText();

    };

    class OpenXRInteractionUserPathDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionUserPathDescriptor, "{F3913A15-41FC-4EC9-A381-296C0AB6D6C6}");
        virtual ~OpenXRInteractionUserPathDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);
        const OpenXRInteractionComponentPathDescriptor* GetComponentPathDescriptor(const AZStd::string& componentPathName) const;

        //! A user friendly name.
        AZStd::string m_name;
        //! For OpenXR a User Path string would look like:
        //! "/user/hand/left", or "/user/hand/right", etc
        AZStd::string m_path;
        //! Component Paths that are only supported under this user path.
        //! This list can be empty.
        AZStd::vector<OpenXRInteractionComponentPathDescriptor> m_componentPathDescriptors;

    private:
        AZStd::string GetEditorText();

    };

    class OpenXRInteractionProfileDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionProfileDescriptor, "{BC73B4BC-4F15-4B1E-AEA9-B133FBB5AD16}");
        virtual ~OpenXRInteractionProfileDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        static constexpr char LogName[] = "OpenXRInteractionProfileDescriptor";

        //! Returns success only if the data makes sense, has proper casing, etc.
        AZ::Outcome<void, AZStd::string> Validate() const;

        const OpenXRInteractionUserPathDescriptor* GetUserPathDescriptor(const AZStd::string& userPathName) const;
        const OpenXRInteractionComponentPathDescriptor* GetCommonComponentPathDescriptor(const AZStd::string& componentPathName) const;
        AZStd::string GetComponentAbsolutePath(const OpenXRInteractionUserPathDescriptor& userPathDescriptor, const AZStd::string& componentPathName) const;

        //! Unique name across all OpenXRInteractionProfileDescriptor.
        //! It serves also as user friendly display name, and because
        //! it is unique it can be used in a dictionary.
        AZStd::string m_uniqueName;
        AZStd::string m_path;

        AZStd::vector<OpenXRInteractionUserPathDescriptor> m_userPathDescriptors;
        // ComponentsPaths that are supported by all User Paths listed in @m_userPathDescriptors
        AZStd::vector<OpenXRInteractionComponentPathDescriptor> m_commonComponentPathDescriptors;

    private:
        AZStd::string GetEditorText();
    };

    //! This asset defines a list of Interaction Profile Descriptors.
    //! The Engine only needs one of these assets, which is used to express
    //! all the different interaction profiles (aka XR Headset Devices) that
    //! are supported by OpenXR.
    //! Basically this asset contains data as listed here:
    //! https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-interaction-profiles
    class OpenXRInteractionProfilesAsset final
        : public AZ::Data::AssetData
    {
    public:
        AZ_CLASS_ALLOCATOR(OpenXRInteractionProfilesAsset, AZ::SystemAllocator);
        AZ_RTTI(OpenXRInteractionProfilesAsset, "{02555DCD-E363-42FB-935C-4E67CC3A1699}", AZ::Data::AssetData);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr char s_assetTypeName[] = "OpenXR Interaction Profiles";
        static constexpr char s_assetExtension[] = "xrprofiles";

        const OpenXRInteractionProfileDescriptor* GetInteractionProfileDescriptor(const AZStd::string& profileName) const;

        AZStd::vector<OpenXRInteractionProfileDescriptor> m_interactionProfileDescriptors;
    };

    //! Custom asset handler
    class OpenXRInteractionProfilesAssetHandler final
        : public AzFramework::GenericAssetHandler<OpenXRInteractionProfilesAsset>
    {
    public:
        AZ_RTTI(OpenXRInteractionProfilesAssetHandler, "{1C4A27E9-6768-4C59-9582-2A01A0DEC1D0}", AzFramework::GenericAssetHandler<OpenXRInteractionProfilesAsset>);
        AZ_CLASS_ALLOCATOR(OpenXRInteractionProfilesAssetHandler, AZ::SystemAllocator);

        static constexpr char LogName[] = "OpenXRInteractionProfilesAssetHandler";

        OpenXRInteractionProfilesAssetHandler();

        bool SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream) override;
    };


}// namespace OpenXRVk
