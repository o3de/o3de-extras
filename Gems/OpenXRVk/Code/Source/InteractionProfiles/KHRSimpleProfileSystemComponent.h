/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

#include <OpenXRVk/OpenXRInteractionProviderBus.h>


namespace OpenXRVk
{
    //! This class is the component related to the vulkan backend of XR.
    class KHRSimpleProfileSystemComponent final
        : public AZ::Component
        , public OpenXRInteractionProviderBus::Handler
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
        // OpenXRInteractionProviderBus::Handler overrides
        //! Create OpenXRVk::Instance object
        AZStd::string GetName() const override;
        AZStd::vector<AZStd::string> GetUserPaths() const override;
        AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const override;
        ///////////////////////////////////////////////////////////////////

    private:
        AZStd::string m_name;
        AZStd::vector<AZStd::string> m_userPaths;
        //! The key is a user path and the value is a list of component paths that exist
        //! for said user path. 
        AZStd::unordered_map<AZStd::string, AZStd::vector<AZStd::string>> m_componentPaths;
    };
}//namespace OpenXRVk