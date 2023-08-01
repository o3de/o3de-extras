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
    Source/Nodes/CreateModel.ScriptCanvasNodeable.xml
    Source/Nodes/CreateModel.cpp
    Source/Nodes/CreateModel.h
    Source/Nodes/FeedForward.ScriptCanvasNodeable.xml
    Source/Nodes/FeedForward.cpp
    Source/Nodes/FeedForward.h
)
