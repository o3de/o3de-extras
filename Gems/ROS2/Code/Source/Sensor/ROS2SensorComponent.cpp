/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    template<typename T>
    T ComputeMean(const AZStd::vector<T>& vec)
    {
        const size_t size = vec.size();
        if (size == 0 )return 0;
        return std::accumulate(vec.begin(), vec.end(), 0.0) / size;
    }

    template<typename T>
    T ComputeVariance(const AZStd::vector<T>& vec, T mean)
    {
        const size_t size = vec.size();
        if (size == 0 )return 0;
        auto variance_computer = [&mean, &size](T acc, const T& val)
        {
            return acc + ((val - mean) * (val - mean) / (size - 1));
        };

        return std::accumulate(vec.begin(), vec.end(), 0.0, variance_computer);
    }


    void ROS2SensorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        AZStd::string s = AZStd::string::format("Sensor %s [%llu] onFrequencyTick FPS ", GetEntity()->GetName().c_str(), GetId());
        m_deltaTimeHistogram.Init(s.c_str(), 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 80.0f);
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

    }

    void ROS2SensorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void ROS2SensorComponent::Reflect(AZ::ReflectContext* context)
    {
        SensorConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SensorComponent, AZ::Component>()->Version(1)->Field(
                "SensorConfiguration", &ROS2SensorComponent::m_sensorConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SensorComponent>("ROS2 Sensor", "Base component for sensors")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SensorComponent::m_sensorConfiguration,
                        "Sensor configuration",
                        "Sensor configuration");
            }
        }
    }

    AZStd::string ROS2SensorComponent::GetNamespace() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetNamespace();
    };

    AZStd::string ROS2SensorComponent::GetFrameID() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameID();
    }

    void ROS2SensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2SensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (m_benchmarkGoing)
        {
            auto now = AZStd::chrono::system_clock::now();
            AZStd::chrono::duration<float> elapsed = now - m_start;
            m_benchmarkElapsedChrono = elapsed.count();
            if (m_benchmarkElapsedChrono > 10.0f){
                m_benchmarkGoing = false;
            }
        }
        Visualise(); // each frame
        if (!m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        auto frequency = m_sensorConfiguration.m_frequency;

        auto frameTime = frequency == 0 ? 1 : 1 / frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }
        // Note that sensor frequency can be limited by simulation tick rate (if higher sensor Hz is desired).
        double currentTime =  TimeMsToSecondsDouble(AZ::Interface<AZ::ITime>::Get()->GetElapsedTimeMs());
        const float deltaTimeOnFreqTick = 1.0f*(currentTime - m_timeElapsedSinceLastFrequencyTick);
        m_effectiveFps  = 1.f / deltaTimeOnFreqTick;
        m_deltaTimeHistogram.PushValue(1.0f/deltaTimeOnFreqTick);
        m_timeElapsedSinceLastFrequencyTick = currentTime;
        if (m_benchmarkGoing){
            m_aggregateFps.push_back(m_effectiveFps);
        }
        FrequencyTick();

    }

    void ROS2SensorComponent::OnImGuiUpdate(){
        AZStd::string s = AZStd::string::format("Benchmark##%s %llu ",GetEntity()->GetName().c_str(),GetId());
        ImGui::Text("======== Sensor %s [%llu] ========================", GetEntity()->GetName().c_str(), GetId());
        ImGui::BulletText("FrameRate : %f / %f", m_effectiveFps, m_sensorConfiguration.m_frequency);
        if (m_benchmarkGoing){
            ImGui::BulletText("Benchmarking %f", m_benchmarkElapsedChrono);
        }else{
            if (ImGui::Button(s.c_str()))
            {
                m_benchmarkGoing = true;
                m_aggregateFps.clear();
                m_start = AZStd::chrono::system_clock::now();
            }
            if(m_aggregateFps.size() > 2)
            {
                auto minmax = AZStd::minmax_element(m_aggregateFps.begin(),m_aggregateFps.end());
                ImGui::BulletText("Benchmark result : ");
                ImGui::BulletText("Max / Min : %f / %f", *minmax.first, *minmax.second);
                float mean = ComputeMean(m_aggregateFps);
                float var = ComputeVariance(m_aggregateFps, mean);
                float stddev = AZStd::sqrt(var);
                ImGui::BulletText("Mean (stddev) : %f ( %f )", mean, stddev);
            }
        }
        m_deltaTimeHistogram.Draw(ImGui::GetColumnWidth(), 100.0f);
        ImGui::End();

    }

} // namespace ROS2
