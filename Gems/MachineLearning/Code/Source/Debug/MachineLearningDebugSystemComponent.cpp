/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/Debug/MachineLearningDebugSystemComponent.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Interface/Interface.h>
#include <Atom/Feature/ImGui/SystemBus.h>
#include <ImGuiContextScope.h>
#include <ImGui/ImGuiPass.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

namespace MachineLearning
{
    void MachineLearningDebugSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MachineLearningDebugSystemComponent, AZ::Component>()
                ->Version(1);
        }
    }

    void MachineLearningDebugSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("MachineLearningDebugSystemComponent"));
    }

    void MachineLearningDebugSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        ;
    }

    void MachineLearningDebugSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatbile)
    {
        incompatbile.push_back(AZ_CRC_CE("MachineLearningDebugSystemComponent"));
    }

    void MachineLearningDebugSystemComponent::Activate()
    {
#ifdef IMGUI_ENABLED
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();
#endif
    }

    void MachineLearningDebugSystemComponent::Deactivate()
    {
#ifdef IMGUI_ENABLED
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
#endif
    }

#ifdef IMGUI_ENABLED
    void MachineLearningDebugSystemComponent::OnModelRegistryDisplay()
    {
        const float TEXT_BASE_WIDTH = ImGui::CalcTextSize("A").x;

        const ImGuiTableFlags flags = ImGuiTableFlags_BordersV
            | ImGuiTableFlags_BordersOuterH
            | ImGuiTableFlags_Resizable
            | ImGuiTableFlags_RowBg
            | ImGuiTableFlags_NoBordersInBody;

        const ImGuiTreeNodeFlags nodeFlags = (ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanFullWidth);

        IMachineLearning* machineLearning = MachineLearningInterface::Get();
        const ModelSet& modelSet = machineLearning->GetModelSet();

        ImGui::Text("Total registered models: %u", static_cast<uint32_t>(modelSet.size()));
        ImGui::NewLine();

        if (ImGui::BeginTable("Model Details", 6, flags))
        {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthFixed, TEXT_BASE_WIDTH * 32.0f);
            ImGui::TableSetupColumn("File", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableSetupColumn("Input Neurons", ImGuiTableColumnFlags_WidthFixed, TEXT_BASE_WIDTH * 12.0f);
            ImGui::TableSetupColumn("Output Neurons", ImGuiTableColumnFlags_WidthFixed, TEXT_BASE_WIDTH * 12.0f);
            ImGui::TableSetupColumn("Layers", ImGuiTableColumnFlags_WidthFixed, TEXT_BASE_WIDTH * 12.0f);
            ImGui::TableSetupColumn("Parameters", ImGuiTableColumnFlags_WidthFixed, TEXT_BASE_WIDTH * 12.0f);
            ImGui::TableHeadersRow();

            AZStd::size_t index = 0;
            for (auto& neuralNetwork : modelSet)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text(neuralNetwork->GetName().c_str());
                ImGui::TableNextColumn();
                ImGui::Text(neuralNetwork->GetAssetFile(AssetTypes::Model).c_str());
                ImGui::TableNextColumn();
                ImGui::Text("%lld", aznumeric_cast<AZ::s64>(neuralNetwork->GetInputDimensionality()));
                ImGui::TableNextColumn();
                ImGui::Text("%lld", aznumeric_cast<AZ::s64>(neuralNetwork->GetOutputDimensionality()));
                ImGui::TableNextColumn();
                ImGui::Text("%lld", aznumeric_cast<AZ::s64>(neuralNetwork->GetLayerCount()));
                ImGui::TableNextColumn();
                ImGui::Text("%llu", neuralNetwork->GetParameterCount());
                ++index;
            }
            ImGui::EndTable();
            ImGui::NewLine();
        }

        ImGui::End();
    }

    void MachineLearningDebugSystemComponent::OnModelTrainingDisplay()
    {
        m_trainingWindow.OnImGuiUpdate();
    }

    void MachineLearningDebugSystemComponent::OnImGuiMainMenuUpdate()
    {
        if (ImGui::BeginMenu("MachineLearning"))
        {
            ImGui::Checkbox("Model Registry", &m_displayModelRegistry);
            ImGui::Checkbox("Model Training", &m_displayTrainingWindow);
            ImGui::EndMenu();
        }
    }

    void MachineLearningDebugSystemComponent::OnImGuiUpdate()
    {
        if (m_displayModelRegistry)
        {
            if (ImGui::Begin("Model Registry", &m_displayModelRegistry, ImGuiWindowFlags_None))
            {
                OnModelRegistryDisplay();
            }
            ImGui::End();
        }

        if (m_displayTrainingWindow)
        {
            if (ImGui::Begin("Model Training", &m_displayTrainingWindow, ImGuiWindowFlags_None))
            {
                OnModelTrainingDisplay();
            }
            ImGui::End();
        }
    }
#endif
}
