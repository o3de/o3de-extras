/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Component/Component.h>

#include <AzFramework/Asset/AssetCatalogBus.h>

#include <OpenXRVk/OpenXRInteractionProfilesAsset.h>
#include "OpenXRInteractionProfilesProviderInterface.h"


namespace OpenXRVk
{
    struct OpenXRPath
    {
        AZStd::string m_displayName;
        //! Although this path is relative,
        //! it should start with "/".
        //! Examples:
        //! 1- For a User Path this string would look like this:
        //!    "/user/hand/left"
        //! 2- For a Component Path this string would look like this:
        //!    "/input/select/click"
        AZStd::string m_xrRelativePath;
    };

    struct OpenXRComponentPath : public OpenXRPath
    {
        XrActionType m_actionType;
    };

    //! This system component provides data that can be used to pick xrActions
    //! that will used at runtime for any given application.
    class OpenXRInteractionProfilesProviderSystemComponent final
        : public AZ::Component
        , private AzFramework::AssetCatalogEventBus::Handler
        , private AZ::Data::AssetBus::Handler
        , public OpenXRInteractionProfilesProviderInterface::Registrar
    {
    public:
        AZ_COMPONENT(OpenXRInteractionProfilesProviderSystemComponent, "{123EDAF5-416B-4AEF-BEEC-03A8A8C71643}");

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        static void Reflect(AZ::ReflectContext* context);

        OpenXRInteractionProfilesProviderSystemComponent() = default;
        ~OpenXRInteractionProfilesProviderSystemComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // AssetCatalogEventBus
        void OnCatalogLoaded(const char* /*catalogFile*/) override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // AZ::Data::AssetBus::Handler overrides
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        //////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // OpenXRInteractionProfileBus::Handler overrides
        const AZStd::vector<AZStd::string>& GetInteractionProfileNames() const override;
        const OpenXRInteractionProfileDescriptor* GetInteractionProfileDescriptor(const AZStd::string profileName) const override;

        // AZStd::string GetName() const override;
        // AZStd::vector<AZStd::string> GetUserPaths() const override;
        // AZStd::string GetUserTopPath(const AZStd::string& userPathName) const override;
        // AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const override;
        // OpenXRInteractionProfile::ActionPathInfo GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const override;
        // AZStd::string GetInteractionProviderPath() const override;
        
        ///////////////////////////////////////////////////////////////////

    private:

        AZStd::string GetDefaultInteractionProfilesAssetPath();

        static constexpr char LogName[] = "OpenXRInteractionProfilesProviderSystemComponent";

        AZ::Data::Asset<OpenXRInteractionProfilesAsset> m_interactionProfilesAsset;

    };
}//namespace OpenXRVk