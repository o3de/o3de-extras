/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRFactory.h>
#include <AzCore/Component/Component.h>

namespace OpenXRVk
{
    //! This class is the component related to the vulkan backend of XR.
    class SystemComponent final
    : public AZ::Component
    , public XR::Factory
    {
    public:
        AZ_COMPONENT(SystemComponent, "{C0ABD1CE-FD3C-48C3-8AE8-C098BCCFC604}");

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void Reflect(AZ::ReflectContext* context);

        SystemComponent();
        ~SystemComponent();

        //////////////////////////////////////////////////////////////////////////
        // Component
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        ///////////////////////////////////////////////////////////////////
        // XR::Factory overrides
        // Create OpenXRVk::Instance object
        virtual XR::Ptr<XR::Instance> CreateInstance();

        // Create OpenXRVk::Device object
        virtual XR::Ptr<XR::Device> CreateDevice();
        ///////////////////////////////////////////////////////////////////
    };
}