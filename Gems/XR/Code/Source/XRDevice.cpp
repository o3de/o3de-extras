/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <XR/XRDevice.h>

namespace XR
{
    AZ::RHI::ResultCode Device::Init(Ptr<Instance> instance)
    {
        m_instance = instance;
        return AZ::RHI::ResultCode::Success;
    }

    Ptr<Instance> Device::GetInstance()
    {
        return m_instance;
    }

    void Device::Shutdown()
    {
        ShutdownInternal();
        m_instance = nullptr;
    }

} // namespace XR
