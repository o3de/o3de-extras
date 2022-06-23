/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/Matrix4x4.h>

namespace XR
{
    //! Creates an off-center projection matrix suitable for VR
    AZ::Matrix4x4 CreateProjectionOffset(float angleLeft, float angleRight, float angleBottom, float angleTop, float nearDist, float farDist);
}
