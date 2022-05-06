/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRSession.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    class SessionDescriptor final
        : public XR::SessionDescriptor 
    {
    public:
        AZ_RTTI(SessionDescriptor, "{775CCED3-9676-4F48-B419-BDADE0F7F447}", XR::SessionDescriptor);

        SessionDescriptor() = default;
        virtual ~SessionDescriptor() = default;

        //any openxr specific session descriptor data
    };

    // Class that will help manage XrSession
    class Session final
        : public XR::Session
    {
    public:
        AZ_RTTI(Session, "{6C899F0C-9A3D-4D79-8E4F-92AFB67E5EB1}", XR::Session);

        static AZStd::intrusive_ptr<Session> Create();

        void LogReferenceSpaces();
        void HandleSessionStateChangedEvent(
            const XrEventDataSessionStateChanged& stateChangedEvent,
            bool* exitRenderLoop,
            bool* requestRestart);
        XrSession GetXrSession();
        virtual bool IsSessionRunning() const override;
        virtual bool IsSessionFocused() const override;
        virtual AZ::RHI::ResultCode InitInternal();

    private:
        XrSession m_session{ XR_NULL_HANDLE };
        // Application's current lifecycle state according to the runtime
        XrSessionState m_sessionState{ XR_SESSION_STATE_UNKNOWN };
        XrFrameState m_frameState{ XR_TYPE_FRAME_STATE };
    };
}
