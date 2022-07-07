/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/containers/vector.h>
#include <OpenXRVk/OpenXRVkFunctionLoader.h>
#include <OpenXRVk_Platform.h>
#include <OpenXRVk_Traits_Platform.h>

namespace OpenXRVk
{
    FunctionLoader::~FunctionLoader()
    {
        AZ_Assert(!m_moduleHandle, "Shutdown was not called before destroying this FunctionLoader");
    }

    bool FunctionLoader::Init()
    {
        const AZStd::vector<const char*> libs = {
            VULKAN_DLL,
            VULKAN_1_DLL
        };

        for (const char* libName : libs)
        {
            m_moduleHandle = AZ::DynamicModuleHandle::Create(libName);
            if (m_moduleHandle->Load(false))
            {
                break;
            }
            else
            {
                m_moduleHandle = nullptr;
            }
        }

        if (!m_moduleHandle)
        {
            AZ_Warning("Vulkan", false, "Could not find Vulkan library.");
            return false;
        }
            
        return InitInternal();
    }

    void FunctionLoader::Shutdown()
    {
        ShutdownInternal();
        if (m_moduleHandle)
        {
            m_moduleHandle->Unload();
        }
        m_moduleHandle = nullptr;
    }
}
