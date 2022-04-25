/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <Atom/RPI.Public/XR/XRGraphicsBinding.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // This class will be responsible for creating XR::Session and
            // all the code around managing the session state
            class Session
            {
            public:
                class Descriptor
                {
                public:
                    // Graphics Binding will contain renderer related data to start a xr session
                    GraphicsBinding* m_graphicsBinding;
                };

                ResultCode Init(Session::Descriptor sessionDesc);
                bool IsSessionRunning() const;
                bool IsSessionFocused() const;
                virtual ResultCode InitSessionInternal(Session::Descriptor descriptor);

            private:
                Session::Descriptor m_descriptor;
                bool m_sessionRunning = false;
                bool m_sessionFocused = false;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
