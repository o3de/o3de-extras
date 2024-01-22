/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>

#include "OpenXRActionSetDescriptor.h"

namespace OpenXRVk
{
    //! This asset defines a list of  OpenXR Action Sets that an application supports
    //! regarding inputs and haptics.
    class OpenXRActionSetsAsset final
        : public AZ::Data::AssetData
    {
    public:
        AZ_RTTI(OpenXRActionSetsAsset, "{C2DEE370-6151-4701-AEA5-AEA3CA247CFF}", AZ::Data::AssetData);
        AZ_CLASS_ALLOCATOR(OpenXRActionSetsAsset, AZ::SystemAllocator);
        static void Reflect(AZ::ReflectContext* context);

        static constexpr char s_assetTypeName[] = "OpenXR Action Sets Asset";
        static constexpr char s_assetExtension[] = "xractions";

        AZStd::vector<OpenXRActionSetDescriptor> m_actionSetDescriptors;
    };
}// namespace OpenXRVk
