/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/Module/DynamicModuleHandle.h>

namespace OpenXRVk
{
    class FunctionLoader
    {
    public:
        AZ_CLASS_ALLOCATOR(FunctionLoader, AZ::SystemAllocator, 0);

        static AZStd::unique_ptr<FunctionLoader> Create();

        //! Load vulkan specific dlls.
        bool Init();

        //! Unload the dlls.
        void Shutdown();

        virtual ~FunctionLoader();
    protected:

        virtual bool InitInternal() = 0;
        virtual void ShutdownInternal() = 0;

        AZStd::unique_ptr<AZ::DynamicModuleHandle> m_moduleHandle;
    };
}
