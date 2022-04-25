/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSession.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage XrSession
        class Session final
            : public AZ::RPI::XR::Session
        {
        public:
            static AZStd::intrusive_ptr<Session> Create();
            AZ::RPI::XR::ResultCode InitSessionInternal(AZ::RPI::XR::Session::Descriptor descriptor) override;
            void LogReferenceSpaces();
            void HandleSessionStateChangedEvent(
                const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop, bool* requestRestart);
            XrSession GetSession();
            bool IsSessionFocused() const override;
            AZ::RPI::XR::ResultCode InitInternal();

        private:
            XrSession m_session{ XR_NULL_HANDLE };
            // Application's current lifecycle state according to the runtime
            XrSessionState m_sessionState{ XR_SESSION_STATE_UNKNOWN };
            XrFrameState m_frameState{ XR_TYPE_FRAME_STATE };
        };
    } // namespace OpenXRVk
} // namespace AZ
