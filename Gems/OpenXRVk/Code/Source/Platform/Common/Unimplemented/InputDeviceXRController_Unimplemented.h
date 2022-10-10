/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <OpenXRVk/InputDeviceXRController.h>

namespace AzFramework
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // static
    inline InputDeviceXRController::Implementation* InputDeviceXRController::Implementation::Create(
        InputDeviceXRController& inputDevice)
    {
        return nullptr;
    }

} // namespace AzFramework
