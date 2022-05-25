/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <XR/XRBase.h>
#include <XR/XRSession.h>

namespace XR
{
    //Todo: Pull this in when needed or remove
    /*
    class InputDescriptor 
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(InputDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(InputDescriptor, "{C690ABBF-D8A9-4348-98E6-45BBF432D673}");

        InputDescriptor() = default;
        virtual ~InputDescriptor() = default;

        //any extra info for a generic xr InputDescriptor
        Ptr<Session> m_session;
    };

    // This class will be responsible for creating XR::Input
    // which manage event queue or poll actions
    class Input
        : public AZStd::intrusive_base
    {
    public:
        AZ_CLASS_ALLOCATOR(Input, AZ::SystemAllocator, 0);
        AZ_RTTI(Input, "{DCDFC6A7-B457-414B-BC24-0831C2AC628B}");

        Input() = default;
        virtual ~Input() = default;

        virtual AZ::RHI::ResultCode Init();

        virtual void InitializeActions() = 0;
        virtual void PollActions() = 0;
        virtual void PollEvents() = 0;
        virtual AZ::RHI::ResultCode InitInternal() = 0;

    private:
        Ptr<InputDescriptor> m_descriptor;
    };
    */
} // namespace XR
