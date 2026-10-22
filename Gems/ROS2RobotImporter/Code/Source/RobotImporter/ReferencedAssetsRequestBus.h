/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>
#include <RobotImporter/ImportSessionId.h>

namespace ROS2RobotImporter
{
    class ReferencedAssetsRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = ImportSessionId;

        //! Return referenced asset from RefferencedAssetMap
        //! @param modelUri URI of the model the asset belongs to, empty for assets outside of any model
        //! @param assetUri unresolved URI of the asset within the model from the source file
        virtual AZStd::optional<Assets::ReferencedAsset> FindReferencedAssets(
            const AZStd::string& modelUri, const AZ::IO::Path& assetUri) const = 0;
    };

    using ReferencedAssetsRequestBus = AZ::EBus<ReferencedAssetsRequests>;

} // namespace ROS2RobotImporter
