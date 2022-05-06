/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AZCore/Interface/Interface.h>
#include <XR/XRSystem.h>

namespace XR
{
    System* System::Get()
    {
        return AZ::Interface<System>::Get();
    }
} // namespace XR
