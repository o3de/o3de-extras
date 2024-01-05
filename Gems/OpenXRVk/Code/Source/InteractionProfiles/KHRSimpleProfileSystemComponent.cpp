/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


#include <AzCore/Serialization/SerializeContext.h>

#include "KHRSimpleProfileSystemComponent.h"

namespace OpenXRVk
{
    void KHRSimpleProfileSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("InteractionProfileProviderService"));
    }

    void KHRSimpleProfileSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<KHRSimpleProfileSystemComponent, AZ::Component>()
                ->Version(1);
        }
    }

    void KHRSimpleProfileSystemComponent::Activate()
    {
        
    }

    void KHRSimpleProfileSystemComponent::Deactivate()
    {
        if (m_instance)
        {
            XR::Factory::Unregister(this);
            AZ::Interface<XR::Instance>::Unregister(m_instance.get());
            m_instance = nullptr;
        }

        m_actionsBindingAssetHandler->Unregister();
    }

    ///////////////////////////////////////////////////////////////////
    // OpenXRInteractionProviderBus::Handler overrides
    //! Create OpenXRVk::Instance object
    AZStd::string KHRSimpleProfileSystemComponent::GetName() const
    {
        return m_name;
    }

    AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetUserPaths() const
    {
        return m_userPaths;
    }

    AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetComponentPaths(const AZStd::string& userPath) const
    {

    }
    ///////////////////////////////////////////////////////////////////
}