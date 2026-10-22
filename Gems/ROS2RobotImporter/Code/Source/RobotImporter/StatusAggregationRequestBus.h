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
#include <RobotImporter/ImportSessionId.h>

namespace ROS2RobotImporter
{
    enum class ImportStatusMessageType
    {
        Model,
        Link,
        Joint,
        Sensor,
        SensorPlugin,
        ModelPlugin
    };

    //! Interface used by the makers to report what they imported to the owner of the import,
    //! which aggregates the messages and builds a single report of the import.
    class StatusAggregationRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = ImportSessionId;

        //! Add a message about an element of the source file to the report of the import.
        //! @param messageType type of the element the message refers to, used to group the messages in the report.
        //! @param message text of the message, a single entry of the report.
        virtual void ReportImportStatusMessage(ImportStatusMessageType messageType, const AZStd::string& message)
        {
        }

        //! Report that a link was created as an articulation link.
        //! The number of such links is compared with the limit of the physics engine when the report is built.
        virtual void ReportArticulatedLinkCreated()
        {
        }
    };

    using StatusAggregationRequestBus = AZ::EBus<StatusAggregationRequests>;

} // namespace ROS2RobotImporter
