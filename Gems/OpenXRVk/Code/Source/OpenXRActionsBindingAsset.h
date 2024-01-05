/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/ObjectStream.h>

namespace OpenXRVk
{
    class OpenXRActionPath final
    {
    public:
        AZ_RTTI(OpenXRActionPath, "{F25D6382-C9E0-414B-A542-1758F5477D03}");
        virtual ~OpenXRActionPath() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        AZStd::string GetEditorText() const;

        AZStd::string m_interactionProfile;
        AZStd::string m_userPath;
        AZStd::string m_componentPath;

    private:
        AZ::Crc32 OnInteractionProfileSelected();
        AZStd::vector<AZStd::string> GetInteractionProfiles() const;

        AZ::Crc32 OnUserPathSelected();
        AZStd::vector<AZStd::string> GetUserPaths() const;

        AZ::Crc32 OnComponentPathSelected();
        AZStd::vector<AZStd::string> GetComponentPaths() const;
    };

    class OpenXRAction final
    {
    public:
        AZ_RTTI(OpenXRAction, "{90BBF6F6-C7D6-4F64-B784-CE03F86DC36B}");
        virtual ~OpenXRAction() = default;

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
        AZStd::vector<OpenXRActionPath> m_actionPaths; 
    };

    class OpenXRActionSet final
    {
    public:
        AZ_RTTI(OpenXRActionSet, "{3A08BC1F-656F-441F-89C3-829F95B9B329}");
        virtual ~OpenXRActionSet() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        AZStd::string GetEditorText() const;

        AZStd::string m_name; // Regular char*
        AZStd::string m_localizedName; // UTF-8 string.
        uint32_t m_priority = 0; // Higher values mean higher priority.
        AZStd::vector<OpenXRAction> m_actions;
    };

    //! This asset defines a list of  OpenXR Action Sets that an application supports
    //! regarding inputs and haptics.
    class OpenXRActionBindingsAsset final
        : public AZ::Data::AssetData
    {
    public:
        AZ_RTTI(OpenXRActionBindingsAsset, "{C2DEE370-6151-4701-AEA5-AEA3CA247CFF}", AZ::Data::AssetData);
        AZ_CLASS_ALLOCATOR(OpenXRActionBindingsAsset, AZ::SystemAllocator);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr char s_assetTypeName[] = "OpenXR Actions Binding Asset";
        static constexpr char s_assetExtension[] = "xractions";

        AZStd::vector<OpenXRActionSet> m_actionSets;
    };
}// namespace OpenXRVk
