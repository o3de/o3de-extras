/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSession.h>

namespace AZ
{
    namespace OpenXRVk
    {
        AZStd::intrusive_ptr<Session> Session::Create();
        {
        }

        AZ::RPI::XR::ResultCode Session::InitSessionInternal(AZ::RPI::XR::Session::Descriptor descriptor) override
        {
            // AZStd::intrusive_ptr<GraphicsBinding> gBinding = static_cast<GraphicsBinding>(descriptor.m_graphicsBinding);
            // xrCreateSession(..m_session,gBinding,..)
            return AZ::RPI::XR::ResultCode::Success;
        }

        void Session::LogReferenceSpaces()
        {
            //..xrEnumerateReferenceSpaces/
        }

        void Session::HandleSessionStateChangedEvent(
            const XrEventDataSessionStateChanged& stateChangedEvent, bool* exitRenderLoop, bool* requestRestart)
        {
            // Handle Session state changes
        }

        XrSession Session::GetSession()
        {
            return m_session;
        }

        bool Session::IsSessionFocused() const override
        {
            return m_sessionState == XR_SESSION_STATE_FOCUSED;
        }

        AZ::RPI::XR::ResultCode Session::InitInternal()
        {
            // Init specific code
            return AZ::RPI::XR::ResultCode::Success;
        }
    } // namespace OpenXRVk
} // namespace AZ
