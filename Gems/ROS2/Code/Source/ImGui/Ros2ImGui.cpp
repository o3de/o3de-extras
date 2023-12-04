#include "Ros2ImGui.h"

#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzFramework/Components/TransformComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/Events/TickBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <ROS2/Sensor/SensorConfiguration.h>

#include <algorithm>
#include <string>

namespace ROS2
{
    Ros2ImGui::Ros2ImGui()
    {
    }

    Ros2ImGui::~Ros2ImGui()
    {
    }

    void Ros2ImGui::Draw(AZ::Entity* entity)
    {
        if (!entity)
        {
            AZ_Error("Ros2ImGui", false, "Invalid entity provided to Draw method.");
            return;
        }

        auto childComponents = GetComponents(entity->GetId());
        if (childComponents.empty())
        {
            AZ_Warning("Ros2ImGui", false, "No child components found for the entity.");
            return;
        }

        FilterChildComponents(childComponents);
        ImGui::Begin("Sensors");

        for (auto* component : childComponents)
        {
            ImGui::Text("Component: %s", component->RTTI_GetTypeName());

            if (auto* gnssSensor = azrtti_cast<ROS2GNSSSensorComponent*>(component))
            {
                DrawTopicName(gnssSensor);
                std::string uniqueId = std::to_string(reinterpret_cast<uintptr_t>(component));
                DrawSensorComponentControls(gnssSensor, uniqueId);
                // TEMP disabled until the GNSS toggle fix loss is merged (https://github.com/o3de/o3de-extras/pull/624)
                // DrawGNSSFix(gnssSensor, uniqueId);
            }
            else if (IsTickBasedSensorComponent(component))
            {
                auto* sensorComponent = azrtti_cast<ROS2SensorComponentBase<TickBasedSource>*>(component);
                if (sensorComponent)
                {
                    DrawTopicName(sensorComponent);
                    std::string uniqueId = std::to_string(reinterpret_cast<uintptr_t>(component));
                    DrawSensorComponentControls(sensorComponent, uniqueId);
                }
            }
            else if (IsPhysicsBasedSensorComponent(component))
            {
                auto* sensorComponent = azrtti_cast<ROS2SensorComponentBase<PhysicsBasedSource>*>(component);
                if (sensorComponent)
                {
                    DrawTopicName(sensorComponent);
                    std::string uniqueId = std::to_string(reinterpret_cast<uintptr_t>(component));
                    DrawSensorComponentControls(sensorComponent, uniqueId);
                }
            }

            ImGui::Text("----");
        }

        ImGui::End();
    }

    // Private methods implementation

    AZStd::vector<AZ::Component*> Ros2ImGui::GetComponents(AZ::EntityId entityId)
    {
        AZStd::vector<AZ::Component*> components;
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        if (entity)
        {
            components = entity->GetComponents();
        }
        else
        {
            AZ_Error("Ros2ImGui", false, "Could not find entity with ID: %llu", static_cast<unsigned long long>(entityId));
        }
        return components;
    }

    void Ros2ImGui::FilterChildComponents(AZStd::vector<AZ::Component*>& components)
    {
        components.erase(
            std::remove_if(
                components.begin(),
                components.end(),
                [this](AZ::Component* component)
                {
                    return !IsSensorComponent(component) || IsImGuiComponent(component);
                }),
            components.end());
    }

    bool Ros2ImGui::EndsWith(const std::string& value, const std::string& ending) const
    {
        if (ending.size() > value.size())
            return false;
        return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
    }

    bool Ros2ImGui::IsSensorComponent(const AZ::Component* component) const
    {
        std::string typeName = component->RTTI_GetTypeName();
        return EndsWith(typeName, "SensorComponent");
    }

    bool Ros2ImGui::IsImGuiComponent(const AZ::Component* component) const
    {
        std::string typeName = component->RTTI_GetTypeName();
        return EndsWith(typeName, "ImGuiComponent");
    }

    bool Ros2ImGui::IsTickBasedSensorComponent(const AZ::Component* component) const
    {
        const auto sensorComponent = azrtti_cast<const ROS2SensorComponentBase<TickBasedSource>*>(component);
        if (sensorComponent)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Ros2ImGui::IsPhysicsBasedSensorComponent(const AZ::Component* component) const
    {
        const auto sensorComponent = azrtti_cast<const ROS2SensorComponentBase<PhysicsBasedSource>*>(component);
        if (sensorComponent)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void Ros2ImGui::ToggleFixLoss(ROS2GNSSSensorComponent* gnssSensor)
    {
        // TEMP disabled until the GNSS toggle fix loss is merged (https://github.com/o3de/o3de-extras/pull/624)
        // gnssSensor->ToggleFixLoss();
    }

    void Ros2ImGui::DrawGNSSFix(ROS2GNSSSensorComponent* gnssSensor, const std::string& uniqueId)
    {
        const AZ::ComponentId componentId = gnssSensor->GetId();
        if (componentActiveState.find(componentId) == componentActiveState.end())
        {
            componentActiveState[componentId] = true;
        }

        const bool& isActive = componentActiveState[componentId];

        // TEMP disabled until the GNSS toggle fix loss is merged (https://github.com/o3de/o3de-extras/pull/624)
        const bool isFix = true;
        // const bool isFix = gnssSensor->GetFixState();
        std::string buttonLabel = isFix ? "Lose Fix##" : "Recover Fix##";
        buttonLabel += uniqueId;

        if (!isActive)
        {
            // Change the button's appearance to look disabled
            ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);

            // When the button is clicked, it won't do anything
            ImGui::Button(buttonLabel.c_str());

            // Restore the button's appearance
            ImGui::PopStyleVar();
        }
        else
        {
            // If the component is active, button works normally
            if (ImGui::Button(buttonLabel.c_str()))
            {
                ToggleFixLoss(gnssSensor);
            }
        }
    }

} // namespace ROS2