/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRGraphicsBinding.h>

namespace XR
{
    class SessionDescriptor
    {
    public:
        AZ_RTTI(SessionDescriptor, "{F76B99EF-ED66-4AAA-BA35-578339CAB428}");

        SessionDescriptor() = default;
        virtual ~SessionDescriptor() = default;

        // Graphics Binding will contain renderer related data to start a xr session
        AZStd::intrusive_ptr<GraphicsBinding> m_graphicsBinding;
    };

    // This class will be responsible for creating XR::Session and
    // all the code around managing the session state
    class Session
    {
    public:
        AZ_RTTI(Session, "{E7276FE1-94B8-423A-9C1D-1BCF1A0066BC}");

        Session() = default;
        virtual ~Session() = default;

        AZ::RHI::ResultCode Init();
        virtual bool IsSessionRunning() const;
        virtual bool IsSessionFocused() const;
        virtual AZ::RHI::ResultCode InitInternal();

    private:
        AZStd::intrusive_ptr<SessionDescriptor> m_descriptor;
        bool m_sessionRunning = false;
        bool m_sessionFocused = false;
    };
} // namespace XR
