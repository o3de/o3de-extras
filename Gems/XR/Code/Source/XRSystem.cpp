/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/XR/XRSystemInterface.h>
#include <AZCore/Interface/Interface.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            SystemInterface* SystemInterface::Get()
            {
                return Interface<SystemInterface>::Get();
            }
        } // namespace XR
    } // namespace RPI
} // namespace AZ
