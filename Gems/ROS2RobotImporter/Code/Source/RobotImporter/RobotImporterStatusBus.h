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
    enum class ImportStatusMessageType
    {
        Model,
        Link,
        Joint,
        Sensor,
        SensorPlugin,
        ModelPlugin
    };

    class RobotImporterStatusBus : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = ImportSessionId;

        virtual void OnImportStatusMessage(ImportStatusMessageType, const AZStd::string&)
        {
        }
        virtual void OnArticulatedLinkCreated()
        {
        }
    };

    using RobotImporterStatusRequestBus = AZ::EBus<RobotImporterStatusBus>;

} // namespace ROS2RobotImporter
