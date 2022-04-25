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
    /// Captures common behavior of ROS2 sensor components. Derive to implement ROS2 sensors
    class ROS2SensorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler // TODO - high resolution tick source?
    {
    public:
        ROS2SensorComponent() = default;
        virtual ~ROS2SensorComponent() = default;
        AZ_COMPONENT(ROS2SensorComponent, "{91BCC1E9-6D93-4466-9CDB-E73D497C6B5E}", AZ::Component);

        // AZ::Component interface implementation.
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // Required Reflect function.
        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        AZStd::string GetNamespace() const;
        AZStd::string GetFrameID() const; // includes namespace
        const SensorConfiguration& GetConfiguration() const;

    private:
        virtual SensorConfiguration DefaultConfiguration() const; // Override to provide default sensor config
        virtual void FrequencyTick() { }; // Override to implement sensor behavior

        // TODO - Editor component: validation of fields, constraints between values and so on
        SensorConfiguration m_sensorConfiguration;
    };
}  // namespace ROS2
