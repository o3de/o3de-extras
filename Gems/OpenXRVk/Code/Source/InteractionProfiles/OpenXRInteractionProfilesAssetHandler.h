/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/ObjectStream.h>
#include <AzCore/Asset/AssetCommon.h>

#include <AzFramework/Asset/GenericAssetHandler.h>

#include "OpenXRInteractionProfilesAsset.h"


namespace OpenXRVk
{
    //! This asset defines a list of Interaction Profile Descriptors.
    //! The Engine only needs one of these assets, which is used to express
    //! all the different interaction profiles (aka XR Headset Devices) that
    //! are supported by OpenXR.
    //! Basically this asset contains data as listed here:
    //! https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-interaction-profiles
    class OpenXRInteractionProfilesAssetHandler final
        : public AzFramework::GenericAssetHandler<OpenXRInteractionProfilesAsset>
    {
    public:
        AZ_RTTI(OpenXRInteractionProfilesAssetHandler, "{1C4A27E9-6768-4C59-9582-2A01A0DEC1D0}", AzFramework::GenericAssetHandler<OpenXRInteractionProfilesAsset>);
        AZ_CLASS_ALLOCATOR(OpenXRInteractionProfilesAssetHandler, AZ::SystemAllocator);
        static void Reflect(AZ::ReflectContext* context);

    };
}// namespace OpenXRVk
