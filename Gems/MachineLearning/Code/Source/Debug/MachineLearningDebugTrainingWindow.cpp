/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/Debug/MachineLearningDebugTrainingWindow.h>
#include <Source/Assets/MnistDataLoader.h>
#include <Source/Algorithms/Activations.h>
#include <ImGuiContextScope.h>
#include <ImGui/ImGuiPass.h>
#include <imgui/imgui.h>
#include <imgui/imgui_internal.h>

namespace MachineLearning
{
#ifdef IMGUI_ENABLED
    int32_t AZStdStringResizeCallback(ImGuiInputTextCallbackData* data)
    {
        if (data->EventFlag == ImGuiInputTextFlags_CallbackResize)
        {
            AZStd::string* azString = (AZStd::string*)data->UserData;
            AZ_Assert(azString->begin() == data->Buf, "Invalid type");
            azString->resize(data->BufSize);
            data->Buf = azString->begin();
        }
        return 0;
    }

    void TextInputHelper(const char* label, AZStd::string& data)
    {
        ImGui::InputText(label, data.begin(), data.size(), ImGuiInputTextFlags_CallbackResize, AZStdStringResizeCallback, (void*)(&data));
    }

    MachineLearningDebugTrainingWindow::~MachineLearningDebugTrainingWindow()
    {
        for (auto iter : m_trainingInstances)
        {
            delete iter.second;
        }
    }

    TrainingInstance* MachineLearningDebugTrainingWindow::RetrieveTrainingInstance(INeuralNetworkPtr modelPtr)
    {
        TrainingInstance* trainingInstance = m_trainingInstances[m_selectedModel];
        if (trainingInstance == nullptr)
        {
            m_trainingInstances[m_selectedModel] = new TrainingInstance();
            trainingInstance = m_trainingInstances[m_selectedModel];
            trainingInstance->m_trainingCycle.m_model = m_selectedModel;
            trainingInstance->m_testHistogram.Init("Test Cost", 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 0.2f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
            trainingInstance->m_testHistogram.SetMoveDirection(ImGui::LYImGuiUtils::HistogramContainer::PushRightMoveLeft);
            trainingInstance->m_trainHistogram.Init("Train Cost", 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 0.2f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
            trainingInstance->m_trainHistogram.SetMoveDirection(ImGui::LYImGuiUtils::HistogramContainer::PushRightMoveLeft);
            trainingInstance->m_testDataName = m_selectedModel->GetAssetFile(AssetTypes::TestData);
            trainingInstance->m_testLabelName = m_selectedModel->GetAssetFile(AssetTypes::TestLabels);
            trainingInstance->m_trainDataName = m_selectedModel->GetAssetFile(AssetTypes::TrainingData);
            trainingInstance->m_trainLabelName = m_selectedModel->GetAssetFile(AssetTypes::TrainingLabels);
        }
        return trainingInstance;
    }

    void MachineLearningDebugTrainingWindow::LoadTestTrainData(TrainingInstance* trainingInstance)
    {
        if (trainingInstance->m_trainingCycle.m_trainingData == nullptr)
        {
            trainingInstance->m_trainingCycle.m_trainingData = AZStd::make_shared<MnistDataLoader>();
            trainingInstance->m_trainingCycle.m_trainingData->LoadArchive(trainingInstance->m_trainDataName, trainingInstance->m_trainLabelName);
        }
        if (trainingInstance->m_trainingCycle.m_testData == nullptr)
        {
            trainingInstance->m_trainingCycle.m_testData = AZStd::make_shared<MnistDataLoader>();
            trainingInstance->m_trainingCycle.m_testData->LoadArchive(trainingInstance->m_testDataName, trainingInstance->m_testLabelName);
        }
    }

    void MachineLearningDebugTrainingWindow::RecalculateAccuracy(TrainingInstance* trainingInstance, ILabeledTrainingDataPtr data)
    {
        trainingInstance->m_trainingCycle.InitializeContexts();
        trainingInstance->m_totalSamples = static_cast<int32_t>(data->GetSampleCount());
        trainingInstance->m_correctPredictions = 0;
        trainingInstance->m_incorrectPredictions = 0;
        for (int32_t iter = 0; iter < trainingInstance->m_totalSamples; ++iter)
        {
            const AZ::VectorN& activations = data->GetDataByIndex(iter);
            const AZStd::size_t label = data->GetLabelAsValueByIndex(iter);
            const AZ::VectorN* output = m_selectedModel->Forward(trainingInstance->m_trainingCycle.m_inferenceContext.get(), activations);
            AZStd::size_t prediction = ArgMaxDecode(*output);
            if (label == prediction)
            {
                ++trainingInstance->m_correctPredictions;
            }
            else
            {
                ++trainingInstance->m_incorrectPredictions;
            }
        }
    }

    void MachineLearningDebugTrainingWindow::OnImGuiUpdate()
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

        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(0.0f, 0.0f));
        ImGui::BeginChild("LeftPanel", ImVec2(m_trainingSplitWidth, -1.0f), true);

        if (ImGui::BeginTable("Models", 1, flags))
        {
            ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
            ImGui::TableHeadersRow();

            AZStd::size_t index = 0;
            for (auto& neuralNetwork : modelSet)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                const bool isSelected = (m_selectedModelIndex == index);
                if (ImGui::Selectable(neuralNetwork->GetName().c_str(), isSelected))
                {
                    m_selectedModel = neuralNetwork;
                    m_selectedModelIndex = index;
                }
                ++index;
            }
            ImGui::EndTable();
            ImGui::NewLine();
        }

        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::InvisibleButton("vsplitter", ImVec2(8.0f, -1.0f));
        if (ImGui::IsItemActive())
        {
            m_trainingSplitWidth += ImGui::GetIO().MouseDelta.x;
        }
        ImGui::SameLine();
        ImGui::BeginChild("RightPanel", ImVec2(0.0f, -1.0f), true);

        if (m_selectedModel != nullptr)
        {
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8.0f, 4.0f));
            TrainingInstance* trainingInstance = RetrieveTrainingInstance(m_selectedModel);

            float currentTestCost = 0.0f;
            float currentTrainCost = 0.0f;
            if (trainingInstance->m_trainingCycle.m_testData != nullptr && trainingInstance->m_trainingCycle.m_trainingData != nullptr)
            {
                currentTestCost = trainingInstance->m_trainingCycle.ComputeCurrentCost(trainingInstance->m_trainingCycle.m_testData, trainingInstance->m_trainingCycle.m_costFunction, m_costSampleSize);
                currentTrainCost = trainingInstance->m_trainingCycle.ComputeCurrentCost(trainingInstance->m_trainingCycle.m_trainingData, trainingInstance->m_trainingCycle.m_costFunction, m_costSampleSize);
            }

            if (!trainingInstance->m_trainingCycle.m_trainingComplete)
            {
                trainingInstance->m_testHistogram.PushValue(currentTestCost);
                trainingInstance->m_trainHistogram.PushValue(currentTrainCost);
                if (ImGui::Button("Stop training"))
                {
                    trainingInstance->m_trainingCycle.StopTraining();
                }
                ImGui::SameLine();
                int32_t epoch = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_currentEpoch);
                ImGui::Text("Epoch: %d", epoch);
            }
            else
            {
                if (ImGui::Button("Start training"))
                {
                    LoadTestTrainData(trainingInstance);
                    trainingInstance->m_trainingCycle.StartTraining();
                }
                ImGui::SameLine();
                if (ImGui::Button("Save"))
                {
                    m_selectedModel->SaveModel();
                }
                ImGui::SameLine();
                if (ImGui::Button("Load"))
                {
                    m_selectedModel->LoadModel();
                }
                ImGui::SameLine();
                if (ImGui::Button("Recalculate accuracy on test data"))
                {
                    LoadTestTrainData(trainingInstance);
                    RecalculateAccuracy(trainingInstance, trainingInstance->m_trainingCycle.m_testData);
                }
                ImGui::SameLine();
                if (ImGui::Button("Recalculate accuracy on training data"))
                {
                    LoadTestTrainData(trainingInstance);
                    RecalculateAccuracy(trainingInstance, trainingInstance->m_trainingCycle.m_trainingData);
                }
            }
 
            ImGui::Text("Model Name: %s", m_selectedModel->GetName().c_str());
            ImGui::NewLine();
            ImGui::Text("Asset location: %s", m_selectedModel->GetAssetFile(AssetTypes::Model).c_str());
            ImGui::NewLine();

            ImGui::Text("Total samples: %d", trainingInstance->m_totalSamples);
            ImGui::Text("Correct predictions: %d", trainingInstance->m_correctPredictions);
            ImGui::Text("Incorrect predictions: %d", trainingInstance->m_incorrectPredictions);

            const float accuracy = (static_cast<float>(trainingInstance->m_correctPredictions) * 100.0f) / static_cast<float>(trainingInstance->m_totalSamples);
            ImGui::Text("Accuracy: %f", accuracy);

            ImGui::Text("Test score: %f", currentTestCost);
            trainingInstance->m_testHistogram.Draw(ImGui::GetColumnWidth(), 200.0f);
            ImGui::Text("Train score: %f", currentTrainCost);
            trainingInstance->m_trainHistogram.Draw(ImGui::GetColumnWidth(), 200.0f);
            ImGui::SliderInt("Cost evaluation sample size", &m_costSampleSize, 10, 10000);
            ImGui::NewLine();

            TextInputHelper("Test data asset file", trainingInstance->m_testDataName);
            TextInputHelper("Test data label file", trainingInstance->m_testLabelName);
            ImGui::NewLine();
            TextInputHelper("Train data asset file", trainingInstance->m_trainDataName);
            TextInputHelper("Train data label file", trainingInstance->m_trainLabelName);
            ImGui::NewLine();

            //AZStd::fixed_string<64> valueString;
            //valueString = AZStd::fixed_string<64>::format("%0.3f", trainingInstance->m_trainingCycle.m_learningRate);
            //float logValue = log(trainingInstance->m_trainingCycle.m_learningRate);
            //ImGui::SliderFloat("LearningRate", &logValue, log(0.0001f), log(1.0f), valueString.c_str());
            //trainingInstance->m_trainingCycle.m_learningRate = exp(logValue);

            ImGui::SliderFloat("LearningRate", &trainingInstance->m_trainingCycle.m_learningRate, 0.0f, 0.1f);
            ImGui::SliderFloat("LearningRateDecay", &trainingInstance->m_trainingCycle.m_learningRateDecay, 0.0f, 1.0f);
            ImGui::NewLine();

            int32_t batchSize = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_batchSize);
            ImGui::SliderInt("Batch size", &batchSize, 1, 1000);
            trainingInstance->m_trainingCycle.m_batchSize = batchSize;
            int32_t totalIterations = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_totalIterations);
            ImGui::SliderInt("Number of iterations", &totalIterations, 1, 1000);
            trainingInstance->m_trainingCycle.m_totalIterations = totalIterations;

            int32_t costMetric = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_costFunction);
            ImGui::Combo("Cost metric", &costMetric, "MeanSquaredError\0");
            trainingInstance->m_trainingCycle.m_costFunction = static_cast<LossFunctions>(costMetric);

            ImGui::PopStyleVar();
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();
    }
#endif
}
