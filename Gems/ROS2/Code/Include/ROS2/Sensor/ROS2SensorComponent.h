/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "SensorConfiguration.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ImGuiBus.h>
#include <ImGui/ImGuiPass.h>
#include <fstream>
#include <imgui/imgui.h>
#include <LYImGuiUtils/HistogramContainer.h>

namespace ROS2
{
    //! Captures common behavior of ROS2 sensor Components.
    //! Sensors acquire data from the simulation engine and publish it to ROS2 ecosystem.
    //! Derive this Component to implement a new ROS2 sensor. Each sensor Component requires ROS2FrameComponent.
    class ROS2SensorComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , public ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        ROS2SensorComponent() = default;
        virtual ~ROS2SensorComponent() = default;
        AZ_COMPONENT(ROS2SensorComponent, "{91BCC1E9-6D93-4466-9CDB-E73D497C6B5E}");

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

    protected:
        AZStd::string GetNamespace() const; //!< Get a complete namespace for this sensor topics and frame ids.
        AZStd::string GetFrameID() const; //!< Returns this sensor frame ID. The ID contains namespace.

        SensorConfiguration m_sensorConfiguration;

        // ImGui::ImGuiUpdateListenerBus::Handler
        void OnImGuiUpdate() override;

    private:
        //! Executes the sensor action (acquire data -> publish) according to frequency.
        //! Override to implement a specific sensor behavior.
        virtual void FrequencyTick(){};

        //! Visualise sensor operation.
        //! For example, draw points or rays for a lidar, viewport for a camera, etc.
        //! Visualisation can be turned on or off in SensorConfiguration.
        virtual void Visualise(){};

        float m_timeElapsedSinceLastTick = 0.0f;
        double m_timeElapsedSinceLastFrequencyTick = 0.0f;
        float m_effectiveFps = 0.0f;
        ImGui::LYImGuiUtils::HistogramContainer m_deltaTimeHistogram;
        AZStd::vector<float> m_aggregateFps;
        bool m_benchmarkGoing = false;
        float m_benchmarkElapsedChrono = 0;
        AZStd::chrono::time_point<AZStd::chrono::system_clock> m_start;

    };
} // namespace ROS2
