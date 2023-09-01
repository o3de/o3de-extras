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
#include <AzCore/Console/IConsole.h>
#include <AzCore/Console/ILogger.h>

#ifdef IMGUI_ENABLED
#   include <ImGuiContextScope.h>
#   include <ImGui/ImGuiPass.h>
#   include <imgui/imgui.h>
#   include <imgui/imgui_internal.h>
#endif

namespace MachineLearning
{
    AZ_CVAR(bool, ml_logAccuracyValues, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Dumps the actual and expected labels during accuracy calculations");

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
        if (!trainingInstance->m_trainingCycle.m_trainData.IsValid())
        {
            ILabeledTrainingDataPtr dataPtr = AZStd::make_shared<MnistDataLoader>();
            trainingInstance->m_trainingCycle.m_trainData.SetSourceData(dataPtr);
            trainingInstance->m_trainingCycle.m_trainData.LoadArchive(trainingInstance->m_trainDataName, trainingInstance->m_trainLabelName);
        }

        if (!trainingInstance->m_trainingCycle.m_testData.IsValid())
        {
            ILabeledTrainingDataPtr dataPtr = AZStd::make_shared<MnistDataLoader>();
            trainingInstance->m_trainingCycle.m_testData.SetSourceData(dataPtr);
            trainingInstance->m_trainingCycle.m_testData.LoadArchive(trainingInstance->m_testDataName, trainingInstance->m_testLabelName);
        }
    }

    void LogVectorN(const AZ::VectorN& value, const char* label)
    {
        AZStd::string vectorString(label);
        for (AZStd::size_t iter = 0; iter < value.GetDimensionality(); ++iter)
        {
            vectorString += AZStd::string::format(" %.02f", value.GetElement(iter));
        }
        AZLOG_INFO(vectorString.c_str());
    }

    void MachineLearningDebugTrainingWindow::RecalculateAccuracy(TrainingInstance* trainingInstance, ILabeledTrainingData& data)
    {
        trainingInstance->m_trainingCycle.InitializeContexts();
        trainingInstance->m_totalSamples = static_cast<int32_t>(data.GetSampleCount());
        trainingInstance->m_correctPredictions = 0;
        trainingInstance->m_incorrectPredictions = 0;
        for (int32_t iter = 0; iter < trainingInstance->m_totalSamples; ++iter)
        {
            const AZ::VectorN& activations = data.GetDataByIndex(iter);
            const AZ::VectorN& label = data.GetLabelByIndex(iter);
            const AZ::VectorN* output = m_selectedModel->Forward(trainingInstance->m_trainingCycle.m_inferenceContext.get(), activations);
            AZStd::size_t prediction = ArgMaxDecode(*output);
            AZStd::size_t actual = ArgMaxDecode(label);
            if (prediction == actual)
            {
                ++trainingInstance->m_correctPredictions;
            }
            else
            {
                ++trainingInstance->m_incorrectPredictions;
            }

            if (ml_logAccuracyValues)
            {
                LogVectorN(label, "Actual");
                LogVectorN(*output, "Output");
            }
        }
    }

    void MachineLearningDebugTrainingWindow::DrawLayerParameters(TrainingInstance* trainingInstance, AZStd::size_t layerIndex)
    {
        if (trainingInstance->m_layerWeights.size() < layerIndex)
        {
            trainingInstance->m_layerWeights.resize(layerIndex + 1);
            trainingInstance->m_layerWeights[layerIndex].Init("Weights", 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 1.0f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
        }

        if (trainingInstance->m_layerBiases.size() < layerIndex)
        {
            trainingInstance->m_layerBiases.resize(layerIndex + 1);
            trainingInstance->m_layerBiases[layerIndex].Init("Biases", 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 1.0f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
        }

        //m_selectedModel->GetLayerWeights(layerIter), m_selectedModel->GetLayerBiases(layerIter)

        //trainingInstance->m_layerWeights[layerIndex].Draw(ImGui::GetColumnWidth(), 200.0f);
        //trainingInstance->m_layerBiases[layerIndex].Draw(ImGui::GetColumnWidth(), 200.0f);
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

            {
                AZStd::deque<float> costs;
                trainingInstance->m_trainingCycle.m_testCosts.Swap(costs);
                for (float item : costs)
                {
                    trainingInstance->m_testHistogram.PushValue(item);
                }
            }

            {
                AZStd::deque<float> costs;
                trainingInstance->m_trainingCycle.m_trainCosts.Swap(costs);
                for (float item : costs)
                {
                    trainingInstance->m_trainHistogram.PushValue(item);
                }
            }

            int32_t batchSize = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_batchSize);
            int32_t totalIterations = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_totalIterations);

            if (!trainingInstance->m_trainingCycle.m_trainingComplete)
            {
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
                    trainingInstance->m_testHistogram.Init("Test Cost", totalIterations, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 0.2f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
                    trainingInstance->m_trainHistogram.Init("Train Cost", totalIterations, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 0.2f, ImGui::LYImGuiUtils::HistogramContainer::AutoExpand);
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
                    RecalculateAccuracy(trainingInstance, trainingInstance->m_trainingCycle.m_trainData);
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

            ImGui::Text("Test score: %f", trainingInstance->m_testHistogram.GetLastValue());
            trainingInstance->m_testHistogram.Draw(ImGui::GetColumnWidth(), 200.0f);
            ImGui::Text("Train score: %f", trainingInstance->m_trainHistogram.GetLastValue());
            trainingInstance->m_trainHistogram.Draw(ImGui::GetColumnWidth(), 200.0f);
            ImGui::Checkbox("Shuffle data", &trainingInstance->m_trainingCycle.m_shuffleTrainingData);
            ImGui::NewLine();

            ImGui::SliderFloat("LearningRate", &trainingInstance->m_trainingCycle.m_learningRate, 0.0f, 0.1f);
            ImGui::SliderFloat("LearningRateDecay", &trainingInstance->m_trainingCycle.m_learningRateDecay, 0.0f, 1.0f);
            ImGui::SliderFloat("EarlyStop", &trainingInstance->m_trainingCycle.m_earlyStopCost, 0.0f, 1.0f);
            ImGui::NewLine();

            ImGui::SliderInt("Batch size", &batchSize, 1, 1000);
            trainingInstance->m_trainingCycle.m_batchSize = batchSize;
            ImGui::SliderInt("Number of iterations", &totalIterations, 1, 1000);
            trainingInstance->m_trainingCycle.m_totalIterations = totalIterations;

            int32_t costMetric = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_costFunction);
            ImGui::Combo("Cost metric", &costMetric, "MeanSquaredError\0");
            trainingInstance->m_trainingCycle.m_costFunction = static_cast<LossFunctions>(costMetric);
            ImGui::NewLine();

            ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.4f);

            if (ImGui::CollapsingHeader("Test data", ImGuiTreeNodeFlags_Framed))
            {
                ImGui::PushID(&trainingInstance->m_trainingCycle.m_testData);
                int32_t firstElement = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_testData.m_first);
                int32_t lastElement = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_testData.m_last);
                TextInputHelper("Asset file", trainingInstance->m_testDataName);
                TextInputHelper("Label file", trainingInstance->m_testLabelName);
                ImGui::SliderInt("First", &firstElement, 0, lastElement);
                ImGui::SameLine();
                ImGui::SliderInt("Last", &lastElement, firstElement, static_cast<int32_t>(trainingInstance->m_trainingCycle.m_testData.GetOriginalSize()));
                trainingInstance->m_trainingCycle.m_testData.m_first = firstElement;
                trainingInstance->m_trainingCycle.m_testData.m_last = lastElement;
                ImGui::PopID();
            }
            ImGui::NewLine();

            if (ImGui::CollapsingHeader("Train data", ImGuiTreeNodeFlags_Framed))
            {
                ImGui::PushID(&trainingInstance->m_trainingCycle.m_trainData);
                int32_t firstElement = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_trainData.m_first);
                int32_t lastElement = static_cast<int32_t>(trainingInstance->m_trainingCycle.m_trainData.m_last);
                TextInputHelper("Asset file", trainingInstance->m_trainDataName);
                TextInputHelper("Label file", trainingInstance->m_trainLabelName);
                ImGui::SliderInt("First", &firstElement, 0, lastElement);
                ImGui::SameLine();
                ImGui::SliderInt("Last", &lastElement, firstElement, static_cast<int32_t>(trainingInstance->m_trainingCycle.m_trainData.GetOriginalSize()));
                trainingInstance->m_trainingCycle.m_trainData.m_first = firstElement;
                trainingInstance->m_trainingCycle.m_trainData.m_last = lastElement;
                ImGui::PopID();
            }

            //for (AZStd::size_t layerIter = 0; layerIter < m_selectedModel->GetLayerCount(); ++layerIter)
            //{
            //    AZStd::fixed_string<64> name;
            //    name = AZStd::string::format("Layer %u Parameters", static_cast<uint32_t>(layerIter));
            //    if (ImGui::CollapsingHeader(name.c_str()))
            //    {
            //        DrawLayerParameters(trainingInstance, layerIter);
            //    }
            //}

            ImGui::PopItemWidth();
            ImGui::PopStyleVar();
        }

        ImGui::EndChild();
        ImGui::PopStyleVar();
    }
#endif
}
