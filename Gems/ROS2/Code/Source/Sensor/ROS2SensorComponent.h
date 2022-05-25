/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include "SensorConfiguration.h"

namespace ROS2
{
    //! Captures common behavior of ROS2 sensor Components.
    //! Sensors acquire data from the simulation engine and publish it to ROS2 ecosystem.
    //! Derive this Component to implement a new ROS2 sensor. Each sensor Component requires ROS2FrameComponent.
    class ROS2SensorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler // TODO - high resolution tick source?
    {
    public:
        ROS2SensorComponent() = default;
        virtual ~ROS2SensorComponent() = default;
        AZ_COMPONENT(ROS2SensorComponent, "{91BCC1E9-6D93-4466-9CDB-E73D497C6B5E}");

        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        AZStd::string GetNamespace() const; //!< Get a complete namespace for this sensor topics and frame ids.
        AZStd::string GetFrameID() const; //!< Already includes namespace.

        SensorConfiguration m_sensorConfiguration;

    private:
        //! Executes the sensor action (acquire data -> publish) according to frequency.
        //! Override to implement a specific sensor behavior.
        virtual void FrequencyTick() { };

        //! Visualise sensor operation.
        //! For example, draw points or rays for a lidar, viewport for a camera, etc.
        //! Visualisation can be turned on or off in SensorConfiguration.
        virtual void Visualise() { };
    };
}  // namespace ROS2
