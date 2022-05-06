/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RHI.Reflect/Base.h>

#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    AZ::RHI::ResultCode ConvertResult(XrResult xrResult);
    bool IsSuccess(XrResult result);
}
