/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRResult.h>

namespace XR
{
    // This class will be responsible for managing XR Space
    class Space
    {
    public:
        AZ_RTTI(Space, "{A78A37F1-8861-4EB4-8FC6-0E9C11394EF1}");

        virtual AZ::RHI::ResultCode InitInternal() = 0;
    };
} // namespace XR
