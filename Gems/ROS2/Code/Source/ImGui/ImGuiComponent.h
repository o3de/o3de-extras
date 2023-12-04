
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/EditContext.h>

#include <ImGuiBus.h>
#include "Ros2ImGui.h"

namespace ROS2
{
    class ImGuiComponent
        : public AZ::Component
        , public ImGui::ImGuiUpdateListenerBus::Handler
    {
    public:
        AZ_COMPONENT(ImGuiComponent, "{248ADB4E-9FCE-4068-BB21-D47F575AFC99}", AZ::Component);

        ImGuiComponent();
        ~ImGuiComponent() = default;

        void Activate() override;
        void Deactivate() override;

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);


    private:
        void OnImGuiUpdate() override;
        void OnImGuiMainMenuUpdate() override;
        Ros2ImGui *imGui;
        bool m_showRos2ImGui{ false }; 
    };

} // namespace ROS2
