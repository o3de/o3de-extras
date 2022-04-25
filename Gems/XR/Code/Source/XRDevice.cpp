/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/XR/XRDevice.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            ResultCode Device::InitDeviceInternal(Device::Descriptor /* descriptor */)
            {
                return ResultCode::Success;
            }
        } // namespace XR
    } // namespace RPI
} // namespace AZ
