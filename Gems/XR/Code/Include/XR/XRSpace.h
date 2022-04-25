/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // This class will be responsible for managing XR Space
            class Space
            {
            public:
                virtual ResultCode InitInternal() = 0;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
