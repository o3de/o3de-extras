/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <XR/XRInput.h>

namespace XR
{
    AZ::RHI::ResultCode Input::Init()
    {
        //m_session = descriptor.m_session;
        return InitInternal();
    }
} // namespace XR
