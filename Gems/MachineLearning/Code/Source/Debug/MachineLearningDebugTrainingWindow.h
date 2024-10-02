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
#include <AzCore/std/containers/map.h>
#include <MachineLearning/IMachineLearning.h>
#include <Algorithms/Training.h>

#ifdef IMGUI_ENABLED
#   include <imgui/imgui.h>
#   include <ImGuiBus.h>
#   include <LYImGuiUtils/HistogramContainer.h>
#endif

namespace MachineLearning
{
    struct TrainingInstance
    {
        SupervisedLearningCycle m_trainingCycle;

        AZStd::string m_testDataName;
        AZStd::string m_testLabelName;

        AZStd::string m_trainDataName;
        AZStd::string m_trainLabelName;

        int32_t m_totalSamples = 0;
        int32_t m_correctPredictions = 0;
        int32_t m_incorrectPredictions = 0;

#ifdef IMGUI_ENABLED
        ImGui::LYImGuiUtils::HistogramContainer m_testHistogram;
        ImGui::LYImGuiUtils::HistogramContainer m_trainHistogram;
#endif
    };

    class MachineLearningDebugTrainingWindow
    {
    public:
        ~MachineLearningDebugTrainingWindow();

        TrainingInstance* RetrieveTrainingInstance(INeuralNetworkPtr modelPtr);
        void LoadTestTrainData(TrainingInstance* trainingInstance);
        void RecalculateAccuracy(TrainingInstance* trainingInstance, ILabeledTrainingData& data);

#ifdef IMGUI_ENABLED
        void OnImGuiUpdate();
#endif

        AZStd::size_t m_selectedModelIndex = 0;
        INeuralNetworkPtr m_selectedModel = nullptr;
        float m_trainingSplitWidth = 400.0f;

        AZStd::map<INeuralNetworkPtr, TrainingInstance*> m_trainingInstances;
    };
}
