/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/smart_ptr/intrusive_ptr.h>
#include <XR/XRSession.h>

namespace XR
{
    class InputDescriptor
    {
    public:
        AZ_RTTI(InputDescriptor, "{C690ABBF-D8A9-4348-98E6-45BBF432D673}");

        InputDescriptor() = default;
        virtual ~InputDescriptor() = default;

        //any extra info for a generic xr InputDescriptor
        AZStd::intrusive_ptr<Session> m_session;
    };

    // This class will be responsible for creating XR::Input
    // which manage event queue or poll actions
    class Input
    {
    public:
        AZ_RTTI(Input, "{DCDFC6A7-B457-414B-BC24-0831C2AC628B}");

        Input() = default;
        virtual ~Input() = default;

        AZ::RHI::ResultCode Init();

        virtual void InitializeActions() = 0;
        virtual void PollActions() = 0;
        virtual void PollEvents() = 0;
        virtual AZ::RHI::ResultCode InitInternal() = 0;

    private:
        AZStd::intrusive_ptr<InputDescriptor> m_descriptor;
    };
} // namespace XR
