/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRResult.h>
#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <Atom/RPI.Public/XR/XRSession.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            // This class will be responsible for creating XR::Input
            // which manage event queue or poll actions
            class Input
            {
            public:
                Input() = default;
                virtual ~Input() = default;

                class Descriptor
                {
                public:
                    Descriptor() = default;
                    ~Descriptor() = default;

                    Session* m_session;
                };

                ResultCode Init(Input::Descriptor descriptor);

                virtual void InitializeActions() = 0;
                virtual void PollActions() = 0;
                virtual void PollEvents() = 0;
                virtual ResultCode InitInternal() = 0;

            private:
                AZStd::intrusive_ptr<Session> m_session;
            };
        } // namespace XR
    } // namespace RPI
} // namespace AZ
