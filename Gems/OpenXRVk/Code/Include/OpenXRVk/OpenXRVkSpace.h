/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSpace.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage XrSpaces
        class Space final
            : public AZ::RPI::XR::Space
        {
        public:
            static AZStd::intrusive_ptr<AZ::RPI::XR::Space> Create();

            XrSpaceLocation GetSpace(XrSpace space);

        private:
            XrSpace m_baseSpace{ XR_NULL_HANDLE };
        };
    } // namespace OpenXRVk
} // namespace AZ
