/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <ROS2/Sensor/SensorConfiguration.h>
namespace ROS2
{
    //! Interface that allows to get sensor configuration and switch publish state.
    class SensorConfigurationRequest : public AZ::EBusTraits
    {
    public:
        AZ_RTTI(SensorConfigurationRequest, "{01904eab-fa33-7487-b634-e3f8361eb5fb}");
        using BusIdType = AZ::EntityComponentIdPair;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Get the configuration of the sensor.
        virtual SensorConfiguration GetSensorConfiguration() const = 0;

        //! Enable or disable the sensor, completely stopping the sensor from running.
        virtual void SetSensorEnabled(bool enable) = 0;

        //! Enable or disable publishing of the sensor to ROS 2.
        //! The sensor implementation will still be running, but the data will not be published.
        virtual void SetPublishingEnabled(bool publishingEnabled) = 0;

        //! Enable or disable visualization of the sensor.
        //! The sensor implementation will still be running, but the data will not be visualized in the viewport.
        virtual void SetVisualizeEnabled(bool visualizeEnabled) = 0;

        //! Get the the current frequency of the sensor in hertz.
        virtual float GetEffectiveFrequency() const = 0;

        //! Sets desired frequency of the sensor in hertz.
        virtual void SetDesiredFrequency(float frequency) = 0;
    };

    using SensorConfigurationRequestBus = AZ::EBus<SensorConfigurationRequest>;
} // namespace ROS2
