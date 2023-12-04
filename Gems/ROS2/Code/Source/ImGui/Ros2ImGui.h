#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <unordered_map>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <imgui/imgui.h>

#include "../Imu/ROS2ImuSensorComponent.h"
#include "../Lidar/ROS2LidarSensorComponent.h"
#include "../Camera/ROS2CameraSensorComponent.h"
#include "../ContactSensor/ROS2ContactSensorComponent.h"
#include "../GNSS/ROS2GNSSSensorComponent.h"
#include "../Odometry/ROS2OdometrySensorComponent.h"

#pragma once

namespace ROS2
{   
    class Ros2ImGui
    {
    public:
        Ros2ImGui();
        ~Ros2ImGui();

        void Draw(AZ::Entity* entity);

    private:
        std::unordered_map<AZ::ComponentId, bool> componentActiveState;

        AZStd::vector<AZ::Component*> GetComponents(AZ::EntityId entityId);
        void FilterChildComponents(AZStd::vector<AZ::Component*>& components);
        bool EndsWith(const std::string& value, const std::string& ending) const;
        bool IsSensorComponent(const AZ::Component* component) const;
        bool IsImGuiComponent(const AZ::Component* component) const;
        bool IsTickBasedSensorComponent(const AZ::Component* component) const;
        bool IsPhysicsBasedSensorComponent(const AZ::Component* component) const;
        void ToggleFixLoss(ROS2GNSSSensorComponent* gnssSensor);
        void DrawGNSSFix(ROS2GNSSSensorComponent* gnssSensor, const std::string& uniqueId);


        template<typename SensorSourceType>
        void DrawSensorComponentControls(ROS2SensorComponentBase<SensorSourceType>* sensorComponent, const std::string& uniqueId);

        template<typename SensorSourceType>
        void ToggleComponentState(ROS2SensorComponentBase<SensorSourceType>* sensorComponent, const std::string& label, bool& isActive);

        template<typename SensorSourceType>
        void DrawTopicName(ROS2SensorComponentBase<SensorSourceType>* sensorComponent);
    };

    template<typename SensorSourceType>
    void Ros2ImGui::DrawSensorComponentControls(ROS2SensorComponentBase<SensorSourceType>* sensorComponent, const std::string& uniqueId)
        {
            const AZ::ComponentId componentId = sensorComponent->GetId();
            if (componentActiveState.find(componentId) == componentActiveState.end()) {
                componentActiveState[componentId] = true;
            }

            bool& isActive = componentActiveState[componentId]; 

            std::string label = "Component##" + uniqueId;
            ToggleComponentState(sensorComponent, label, isActive);
        }

    template<typename SensorSourceType>
    void Ros2ImGui::ToggleComponentState(ROS2SensorComponentBase<SensorSourceType>* sensorComponent, const std::string& label, bool& isActive)
    {
        if (isActive && ImGui::Button(("Disable " + label).c_str()))
        {
            sensorComponent->Deactivate();
            isActive = false;
        }
        else if (!isActive && ImGui::Button(("Enable " + label).c_str()))
        {
            sensorComponent->Activate();
            isActive = true;
        }
    }

    template<typename SensorSourceType>
    void Ros2ImGui::DrawTopicName(ROS2SensorComponentBase<SensorSourceType>* sensorComponent)
    {
        const auto map = sensorComponent->GetSensorConfiguration().m_publishersConfigurations;

        for (const auto& [key, value] : map)
        {
            ImGui::Text("Topic: %s", value.m_topic.c_str());
        }
    }
} // namespace ROS2
