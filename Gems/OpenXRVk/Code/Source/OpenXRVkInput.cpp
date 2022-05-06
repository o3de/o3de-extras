/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkInput.h>

namespace OpenXRVk
{
    AZStd::intrusive_ptr<XR::Input> Input::Create()
    {
        return nullptr;
    }

    AZ::RHI::ResultCode Input::Init()
    {
        InitializeActions();
    }

    void Input::InitializeActions()
    {
        // Code to populate m_input
        // xrCreateActionSet
        // xrCreateAction
        // xrCreateActionSpace
        // xrAttachSessionActionSets
    }

    void Input::PollActions()
    {
        // xrSyncActions
    }

    void Input::PollEvents()
    {
        // m_session->HandleSessionStateChangedEvent
    }
}
