/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>

#include <XR/XRGraphicsBinding.h>
#include <XR/XRBase.h>

namespace XR
{
    //Todo: Pull this in when needed or remove
    /*
    class SessionDescriptor 
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(SessionDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(SessionDescriptor, "{F76B99EF-ED66-4AAA-BA35-578339CAB428}");

        SessionDescriptor() = default;
        virtual ~SessionDescriptor() = default;

        // Graphics Binding will contain renderer related data to start a xr session
        Ptr<GraphicsBinding> m_graphicsBinding;
    };

    // This class will be responsible for creating XR::Session and
    // all the code around managing the session state
    class Session
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(Session, AZ::SystemAllocator, 0);
        AZ_RTTI(Session, "{E7276FE1-94B8-423A-9C1D-1BCF1A0066BC}");

        Session() = default;
        virtual ~Session() = default;
        
        AZ::RHI::ResultCode Init();
        virtual bool IsSessionRunning() const;
        virtual bool IsSessionFocused() const;
        virtual AZ::RHI::ResultCode InitInternal();
        
    private:
        Ptr<SessionDescriptor> m_descriptor;
        bool m_sessionRunning = false;
        bool m_sessionFocused = false;
    };
    */
} // namespace XR
