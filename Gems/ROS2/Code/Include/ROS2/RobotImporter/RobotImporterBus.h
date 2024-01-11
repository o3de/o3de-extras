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
#include <ROS2/RobotImporter/SDFormatModelPluginImporterHook.h>
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>

namespace ROS2
{
    class RobotImporterRequest : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        //! Generate prefab from the urdf file.
        //! @param filePath The path of the urdf file
        //! @param importAssetWithUrdf If true, the assets referenced in the urdf file will be imported
        //! @param useArticulation If true, the prefab will be generated with articulation
        virtual bool GeneratePrefabFromFile(const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation) = 0;

        //! Return the reference to the list of sensor importer hooks
        virtual const SDFormat::SensorImporterHooksStorage& GetSensorHooks() const = 0;

        //! Return the reference to the list of model plugin importer hooks
        virtual const SDFormat::ModelPluginImporterHooksStorage& GetModelPluginHooks() const = 0;
    };

    using RobotImporterRequestBus = AZ::EBus<RobotImporterRequest>;

} // namespace ROS2
