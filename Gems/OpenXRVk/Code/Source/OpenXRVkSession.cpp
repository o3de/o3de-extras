/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkSession.h>

namespace OpenXRVk
{
    AZStd::intrusive_ptr<Session> Session::Create()
    {
        return nullptr;
    }

    AZ::RHI::ResultCode Session::InitInternal()
    {
        // AZStd::intrusive_ptr<GraphicsBinding> gBinding = static_cast<GraphicsBinding>(descriptor.m_graphicsBinding);
        // xrCreateSession(..m_session,gBinding,..)
        return AZ::RHI::ResultCode::Success;
    }

    void Session::LogReferenceSpaces()
    {
        //..xrEnumerateReferenceSpaces/
    }

    void Session::HandleSessionStateChangedEvent(
        const XrEventDataSessionStateChanged& /*stateChangedEvent*/,
        bool* /*exitRenderLoop*/,
        bool* /*requestRestart*/)
    {
        // Handle Session state changes
    }

    XrSession Session::GetXrSession()
    {
        return m_session;
    }

    bool Session::IsSessionRunning() const
    {
        return true;
    }

    bool Session::IsSessionFocused() const
    {
        return m_sessionState == XR_SESSION_STATE_FOCUSED;
    }

}
