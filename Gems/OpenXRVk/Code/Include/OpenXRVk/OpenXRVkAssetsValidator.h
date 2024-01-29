/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>

#include <OpenXRVk/OpenXRVkInteractionProfilesAsset.h>
#include <OpenXRVk/OpenXRVkActionSetsAsset.h>

namespace OpenXRVkAssetsValidator
{
    AZ::Outcome<void, AZStd::string> ValidateInteractionProfilesAsset(
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset);

    AZ::Outcome<void, AZStd::string> ValidateActionSetsAsset(const OpenXRVk::OpenXRActionSetsAsset& actionSetsAsset,
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset);

}// namespace OpenXRVkAssetsValidator
