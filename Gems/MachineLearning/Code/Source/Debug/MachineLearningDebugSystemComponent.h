/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Interface/Interface.h>
#include <MachineLearning/IMachineLearning.h>
#include <Debug/MachineLearningDebugTrainingWindow.h>

#ifdef IMGUI_ENABLED
#   include <imgui/imgui.h>
#   include <ImGuiBus.h>
#endif

namespace MachineLearning
{
    class MachineLearningDebugSystemComponent final
        : public AZ::Component
#ifdef IMGUI_ENABLED
        , public ImGui::ImGuiUpdateListenerBus::Handler
#endif
    {
    public:

        AZ_COMPONENT(MachineLearningDebugSystemComponent, "{44A3FACE-9808-4BAD-BC9C-6DB6AE0A9707}");

        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ~MachineLearningDebugSystemComponent() override = default;

        //! AZ::Component overrides
        //! @{
        void Activate() override;
        void Deactivate() override;
        //! @}

#ifdef IMGUI_ENABLED
        void OnModelRegistryDisplay();
        void OnModelTrainingDisplay();

        //! ImGui::ImGuiUpdateListenerBus overrides
        //! @{
        void OnImGuiMainMenuUpdate() override;
        void OnImGuiUpdate() override;
        //! @}
    private:

        bool m_displayModelRegistry = false;
        bool m_displayTrainingWindow = false;
        MachineLearningDebugTrainingWindow m_trainingWindow;
#endif
    };
}
