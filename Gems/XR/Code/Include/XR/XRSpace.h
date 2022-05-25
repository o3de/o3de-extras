/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/smart_ptr/intrusive_base.h>

namespace XR
{
    // This class will be responsible for managing XR Space
    class Space
        : public AZStd::intrusive_base
    {
	    //Todo: Pull this in when needed or remove
        /*
    public:
        AZ_CLASS_ALLOCATOR(Space, AZ::SystemAllocator, 0);
        AZ_RTTI(Space, "{A78A37F1-8861-4EB4-8FC6-0E9C11394EF1}");

        virtual AZ::RHI::ResultCode InitInternal() = 0;
        */
    };
} // namespace XR
