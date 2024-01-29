/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>

#include <OpenXRVk/OpenXRVkInteractionProfilesAsset.h>

namespace OpenXRVk
{
    class OpenXRActionPathDescriptor final
    {
    public:
        AZ_RTTI(OpenXRActionPathDescriptor, "{F25D6382-C9E0-414B-A542-1758F5477D03}");
        virtual ~OpenXRActionPathDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        AZStd::string GetEditorText() const;

        AZStd::string m_interactionProfileName;
        AZStd::string m_userPathName;
        AZStd::string m_componentPathName;

    private:
        AZ::Crc32 OnInteractionProfileSelected();
        AZStd::vector<AZStd::string> GetInteractionProfiles() const;

        AZ::Crc32 OnUserPathSelected();
        AZStd::vector<AZStd::string> GetUserPaths() const;

        AZ::Crc32 OnComponentPathSelected();
        AZStd::vector<AZStd::string> GetComponentPaths() const;
    };

    class OpenXRActionDescriptor final
    {
    public:
        AZ_RTTI(OpenXRActionDescriptor, "{90BBF6F6-C7D6-4F64-B784-CE03F86DC36B}");
        virtual ~OpenXRActionDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        AZStd::string GetEditorText() const;

        AZStd::string m_name; // Regular char*
        AZStd::string m_localizedName; // UTF-8 string.
        //! List of I/O action paths that will be bound to this action.
        //! The first action path in this list, determines what type of action paths
        //! can be added to the list. For example:
        //! If the first action path happens to be a boolean, then subsequent action paths
        //! can only be added if they can map to a boolean.
        //! Another important case is if the this is a haptic feedback action (Output), then
        //! subsequent action paths can only be of type haptic feedback actions.
        AZStd::vector<OpenXRActionPathDescriptor> m_actionPathDescriptors;
    };

    class OpenXRActionSetDescriptor final
    {
    public:
        AZ_RTTI(OpenXRActionSetDescriptor, "{3A08BC1F-656F-441F-89C3-829F95B9B329}");
        virtual ~OpenXRActionSetDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        AZStd::string GetEditorText() const;

        AZStd::string m_name; // Regular char*
        AZStd::string m_localizedName; // UTF-8 string.
        uint32_t m_priority = 0; // Higher values mean higher priority.
        AZStd::vector<OpenXRActionDescriptor> m_actionDescriptors;
    };

    //! This asset defines a list of  OpenXR Action Sets that an application supports
    //! regarding inputs and haptics.
    class OpenXRActionSetsAsset final
        : public AZ::Data::AssetData
    {
    public:
        AZ_CLASS_ALLOCATOR(OpenXRActionSetsAsset, AZ::SystemAllocator);
        AZ_RTTI(OpenXRActionSetsAsset, "{C2DEE370-6151-4701-AEA5-AEA3CA247CFF}", AZ::Data::AssetData);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr char s_assetTypeName[] = "OpenXR Action Sets Asset";
        static constexpr char s_assetExtension[] = "xractions";

        AZ::Data::Asset<OpenXRInteractionProfilesAsset> m_interactionProfilesAsset;
        AZStd::vector<OpenXRActionSetDescriptor> m_actionSetDescriptors;

    private:
        AZ::Crc32 OnInteractionProfilesAssetChanged();
    };

    //! We need a custom asset handler because OpenXRActionSetsAsset contains a reference to another
    //! asset of type OpenXRInteractionProfilesAsset and we need to set a static singleton of type
    //! OpenXRInteractionProfilesAsset when the user is creating/editing an OpenXRActionSetsAsset with
    //! the Asset Editor. 
    class OpenXRActionSetsAssetHandler final
        : public AzFramework::GenericAssetHandler<OpenXRActionSetsAsset>
    {
    public:
        AZ_RTTI(OpenXRActionSetsAssetHandler, "{1C4A27E9-6768-4C59-9582-2A01A0DEC1D0}", AzFramework::GenericAssetHandler<OpenXRActionSetsAsset>);
        AZ_CLASS_ALLOCATOR(OpenXRActionSetsAssetHandler, AZ::SystemAllocator);
    
        static constexpr char LogName[] = "OpenXRInteractionProfilesAssetHandler";
    
        OpenXRActionSetsAssetHandler();
    
        // Called by the asset manager to perform actual asset load.
        AZ::Data::AssetHandler::LoadResult LoadAssetData(
            const AZ::Data::Asset<AZ::Data::AssetData>& asset,
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB) override;
    };
}// namespace OpenXRVk
