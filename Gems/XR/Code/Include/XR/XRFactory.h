/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>

#include <XR/XRBase.h>
#include <XR/XRInstance.h>
#include <XR/XRDevice.h>
#include <XR/XRSession.h>
#include <XR/XRInput.h>
#include <XR/XRSwapChain.h>
#include <XR/XRGraphicsBinding.h>

namespace XR
{
    //! Interface responsible for creating all the XR objects which are
    //! internally backed by concrete objects
    class Factory
    {
    public:
        AZ_CLASS_ALLOCATOR(Factory, AZ::SystemAllocator, 0);
        AZ_RTTI(Factory, "{A3D7271A-64FD-442C-9116-DBC32224222F}");

        Factory() = default;
        virtual ~Factory() = default;

        AZ_DISABLE_COPY_MOVE(Factory);

        //! Returns the component service name CRC used by the platform RHI system component.
        static AZ::u32 GetPlatformService();

        //! Registers the global factory instance.
        static void Register(XR::Factory* instance);

        //! Unregisters the global factory instance.
        static void Unregister(XR::Factory* instance);

        //! REturns if the factory ready.
        static bool IsReady();

        //! Access the global factory instance.
        static XR::Factory& Get();

        //! Create XR::Instance object.
        virtual Ptr<XR::Instance> CreateInstance() = 0;

        //! Create XR::Device object.
        virtual Ptr<XR::Device> CreateDevice() = 0;
    };
} // namespace XR

