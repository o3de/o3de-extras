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
#include <AzCore/std/string/string.h>
#include <RobotImporter/Assets/AssetTypes.h>

namespace ROS2RobotImporter
{
    using ImportSessionId = AZ::u32;

    class RobotImporterAssetsBus : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = ImportSessionId;

        //! Return referenced asset from RefferencedAssetMap
        //! @param modelUri URI path for the refferenced asset from the source file
        virtual AZStd::optional<Assets::ReferencedAsset> FindRefferencedAssets(AZ::IO::Path modelUri) const = 0;
    };

    using RobotImporterAssetsRequestBus = AZ::EBus<RobotImporterAssetsBus>;

} // namespace ROS2RobotImporter
