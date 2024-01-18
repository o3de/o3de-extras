/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

#include <OpenXRVk/OpenXRInteractionProfileBus.h>


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
    class KHRSimpleProfileSystemComponent final
        : public AZ::Component
        , public OpenXRInteractionProfileBus::Handler
    {
    public:
        AZ_COMPONENT(KHRSimpleProfileSystemComponent, "{123EDAF5-416B-4AEF-BEEC-03A8A8C71643}");

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        KHRSimpleProfileSystemComponent() = default;
        ~KHRSimpleProfileSystemComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // Component
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // OpenXRInteractionProfileBus::Handler overrides
        //! Create OpenXRVk::Instance object
        AZStd::string GetName() const override;
        AZStd::vector<AZStd::string> GetUserPaths() const override;
        AZStd::string GetUserTopPath(const AZStd::string& userPathName) const override;
        AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const override;
        OpenXRInteractionProfile::ActionPathInfo GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const override;
        AZStd::string GetInteractionProviderPath() const override;
        ///////////////////////////////////////////////////////////////////

    private:
        static constexpr char LogName[] = "KHRSimpleProfileSystemComponent";

        static constexpr AZStd::string_view LeftHand = "(L)";
        static constexpr AZStd::string_view RightHand = "(R)";
        OpenXRPath m_name;
        AZStd::vector<OpenXRPath> m_userPaths;
        //! The key is a user path and the value is a list of component paths that exist
        //! for said user path. 
        AZStd::unordered_map<AZStd::string, AZStd::vector<OpenXRComponentPath>> m_componentPaths;
    };
}//namespace OpenXRVk