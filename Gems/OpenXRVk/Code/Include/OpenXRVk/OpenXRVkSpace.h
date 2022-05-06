/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRSpace.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    // Class that will help manage XrSpaces
    class Space final
        : public XR::Space
    {
    public:
        static AZStd::intrusive_ptr<Space> Create();

        //XrSpaceLocation GetSpace(XrSpace space);

    private:
        XrSpace m_baseSpace{ XR_NULL_HANDLE };
    };
}
