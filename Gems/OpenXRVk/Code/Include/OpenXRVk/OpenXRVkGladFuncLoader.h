/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <OpenXRVk/OpenXRVkFunctionLoader.h>
#include <AzCore/Memory/SystemAllocator.h>

namespace OpenXRVk
{
    class GladFunctionLoader:
        public FunctionLoader
    {
    public:
        AZ_CLASS_ALLOCATOR(GladFunctionLoader, AZ::SystemAllocator, 0);

        GladFunctionLoader() = default;
        virtual ~GladFunctionLoader() = default;

    private:
        //////////////////////////////////////////////////////////////////////////
        // FunctionLoader overrrides
        //! Load the the function pointers from the vulkan dll.
        bool InitInternal() override;
        //! Unload appropriate data.
        void ShutdownInternal() override;
    };
}
