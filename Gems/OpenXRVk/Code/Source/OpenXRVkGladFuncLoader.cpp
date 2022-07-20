/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/PlatformIncl.h>
#define GLAD_VULKAN_IMPLEMENTATION
#include <OpenXRVk_Platform.h>

#include <OpenXRVk/OpenXRVkGladFuncLoader.h>
#include <AzCore/Module/DynamicModuleHandle.h>

namespace
{
    GLADapiproc LoadFunctionFromLibrary(void* userPtr, const char *name)
    {
        AZ::DynamicModuleHandle* moduleHandle = reinterpret_cast<AZ::DynamicModuleHandle*>(userPtr);
        AZ_Assert(moduleHandle, "Invalid module handle");
        return moduleHandle->GetFunction<GLADapiproc>(name);
    }
}

namespace OpenXRVk
{
    AZStd::unique_ptr<FunctionLoader> FunctionLoader::Create()
    {
        return AZStd::make_unique<GladFunctionLoader>();
    }

    bool GladFunctionLoader::InitInternal()
    {
        // Since we don't have the vulkan instance or device yet, we just load the function pointers from the loader
        // using dlsym or something similar.
        return gladLoadVulkanUserPtr(VK_NULL_HANDLE, &LoadFunctionFromLibrary, m_moduleHandle.get()) != 0;
    }

    void GladFunctionLoader::ShutdownInternal()
    {
        gladLoaderUnloadVulkan();
    }
}
