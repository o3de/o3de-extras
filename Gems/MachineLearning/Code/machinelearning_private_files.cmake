#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

set(FILES
    Source/MachineLearningModuleInterface.cpp
    Source/MachineLearningModuleInterface.h
    Source/MachineLearningSystemComponent.cpp
    Source/MachineLearningSystemComponent.h
    Source/Algorithms/Activations.h
    Source/Algorithms/Activations.inl
    Source/Algorithms/LossFunctions.h
    Source/Algorithms/LossFunctions.inl
    Source/Models/Layer.cpp
    Source/Models/Layer.h
    Source/Models/MultilayerPerceptron.cpp
    Source/Models/MultilayerPerceptron.h
    Source/Nodes/AccumulateTrainingGradients.ScriptCanvasNodeable.xml
    Source/Nodes/AccumulateTrainingGradients.cpp
    Source/Nodes/AccumulateTrainingGradients.h
    Source/Nodes/ComputeCost.ScriptCanvasNodeable.xml
    Source/Nodes/ComputeCost.cpp
    Source/Nodes/ComputeCost.h
    Source/Nodes/CreateModel.ScriptCanvasNodeable.xml
    Source/Nodes/CreateModel.cpp
    Source/Nodes/CreateModel.h
    Source/Nodes/FeedForward.ScriptCanvasNodeable.xml
    Source/Nodes/FeedForward.cpp
    Source/Nodes/FeedForward.h
    Source/Nodes/GradientDescent.ScriptCanvasNodeable.xml
    Source/Nodes/GradientDescent.cpp
    Source/Nodes/GradientDescent.h
    Source/Nodes/LoadTrainingData.ScriptCanvasNodeable.xml
    Source/Nodes/LoadTrainingData.cpp
    Source/Nodes/LoadTrainingData.h
)
