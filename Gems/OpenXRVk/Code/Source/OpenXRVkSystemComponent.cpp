/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


#include <AzCore/Serialization/SerializeContext.h>

#include <OpenXRVk/OpenXRVkDevice.h>
#include <OpenXRVk/OpenXRVkInput.h>
#include <OpenXRVk/OpenXRVkInstance.h>
#include <OpenXRVk/OpenXRVkSession.h>
#include <OpenXRVk/OpenXRVkSpace.h>
#include <OpenXRVk/OpenXRVkSwapChain.h>
#include <OpenXRVk/OpenXRVkSystemComponent.h>

namespace OpenXRVk
{
    void SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(XR::Factory::GetPlatformService());
    }

    void SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SystemComponent, AZ::Component>()
                ->Version(1);
        }

        AzFramework::InputDeviceXRController::Reflect(context);
    }

    SystemComponent::SystemComponent()
    {
        // Only have Vulkan back-end implementation for XR at the moment so register it. 
        XR::Factory::Register(this);
    }

    SystemComponent::~SystemComponent()
    {
        XR::Factory::Unregister(this);
    }

    XR::Ptr<XR::Instance> SystemComponent::CreateInstance()
    {
        return Instance::Create();
    }

    XR::Ptr<XR::Device> SystemComponent::CreateDevice()
    {
        return Device::Create();
    }

    XR::Ptr<XR::Session> SystemComponent::CreateSession()
    {
        return Session::Create();
    }
    
    XR::Ptr<XR::Input> SystemComponent::CreateInput()
    {
        return Input::Create();
    }

    XR::Ptr<XR::Space> SystemComponent::CreateSpace()
    {
        return Space::Create();
    }

    XR::Ptr<XR::SwapChain> SystemComponent::CreateSwapChain()
    {
        return SwapChain::Create();
    }

    XR::Ptr<XR::SwapChain::View> SystemComponent::CreateSwapChainView()
    {
        return SwapChain::View::Create();
    }

    XR::Ptr<XR::SwapChain::Image> SystemComponent::CreateSwapChainImage()
    {
        return SwapChain::Image::Create();
    }

    void SystemComponent::Activate()
    {
    }

    void SystemComponent::Deactivate()
    {
    }
}