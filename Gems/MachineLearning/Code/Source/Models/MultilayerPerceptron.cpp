/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Models/MultilayerPerceptron.h>
#include <Algorithms/LossFunctions.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/FileReader.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Console/ILogger.h>
#include <AzNetworking/Serialization/NetworkInputSerializer.h>
#include <AzNetworking/Serialization/NetworkOutputSerializer.h>

namespace MachineLearning
{
    void MultilayerPerceptron::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultilayerPerceptron>()
                ->Version(1)
                ->Field("ModelAsset", &MultilayerPerceptron::m_asset)
                ->Field("Name", &MultilayerPerceptron::m_name)
                ->Field("ModelFile", &MultilayerPerceptron::m_modelFile)
                ->Field("TestDataFile", &MultilayerPerceptron::m_testDataFile)
                ->Field("TestLabelFile", &MultilayerPerceptron::m_testLabelFile)
                ->Field("TrainDataFile", &MultilayerPerceptron::m_trainDataFile)
                ->Field("TrainLabelFile", &MultilayerPerceptron::m_trainLabelFile)
                ->Field("ActivationCount", &MultilayerPerceptron::m_activationCount)
                ->Field("Layers", &MultilayerPerceptron::m_layers)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<MultilayerPerceptron>("A basic multilayer perceptron class", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_asset, "ModelAsset", "The model asset")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_name, "Name", "The name for this model")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_modelFile, "ModelFile", "The file this model is saved to and loaded from")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_testDataFile, "TestDataFile", "The file test data should be loaded from")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_testLabelFile, "TestLabelFile", "The file test labels should be loaded from")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_trainDataFile, "TrainDataFile", "The file training data should be loaded from")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_trainLabelFile, "TrainLabelFile", "The file training labels should be loaded from")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_activationCount, "Activation Count", "The number of neurons in the activation layer")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &MultilayerPerceptron::OnActivationCountChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptron::m_layers, "Layers", "The layers of the neural network")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &MultilayerPerceptron::OnActivationCountChanged)
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<MultilayerPerceptron>("Multilayer perceptron")->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<AZStd::size_t>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)->
                Method("GetName", &MultilayerPerceptron::GetName)->
                Method("GetLayerCount", &MultilayerPerceptron::GetLayerCount)->
                Property("ActivationCount", BehaviorValueProperty(&MultilayerPerceptron::m_activationCount))->
                Property("Layers", BehaviorValueProperty(&MultilayerPerceptron::m_layers))
                ;
        }
    }

    MultilayerPerceptron::MultilayerPerceptron()
    {
    }

    MultilayerPerceptron::MultilayerPerceptron(const MultilayerPerceptron& rhs)
        : m_name(rhs.m_name)
        , m_modelFile(rhs.m_modelFile)
        , m_testDataFile(rhs.m_testDataFile)
        , m_testLabelFile(rhs.m_testLabelFile)
        , m_trainDataFile(rhs.m_trainDataFile)
        , m_trainLabelFile(rhs.m_trainLabelFile)
        , m_activationCount(rhs.m_activationCount)
        , m_layers(rhs.m_layers)
    {
    }

    MultilayerPerceptron::MultilayerPerceptron(AZStd::size_t activationCount)
        : m_activationCount(activationCount)
    {
    }

    MultilayerPerceptron::~MultilayerPerceptron()
    {
    }

    MultilayerPerceptron& MultilayerPerceptron::operator=(const MultilayerPerceptron& rhs)
    {
        m_name = rhs.m_name;
        m_modelFile = rhs.m_modelFile;
        m_testDataFile = rhs.m_testDataFile;
        m_testLabelFile = rhs.m_testLabelFile;
        m_trainDataFile = rhs.m_trainDataFile;
        m_trainLabelFile = rhs.m_trainLabelFile;
        m_activationCount = rhs.m_activationCount;
        m_layers = rhs.m_layers;
        return *this;
    }

    AZStd::string MultilayerPerceptron::GetName() const
    {
        return m_name;
    }

    AZStd::string MultilayerPerceptron::GetAssetFile(AssetTypes assetType) const
    {
        switch (assetType)
        {
        case AssetTypes::Model:
            return m_modelFile;
        case AssetTypes::TestData:
            return m_testDataFile;
        case AssetTypes::TestLabels:
            return m_testLabelFile;
        case AssetTypes::TrainingData:
            return m_trainDataFile;
        case AssetTypes::TrainingLabels:
            return m_trainLabelFile;
        }
        return "";
    }

    AZStd::size_t MultilayerPerceptron::GetInputDimensionality() const
    {
        return m_activationCount;
    }

    AZStd::size_t MultilayerPerceptron::GetOutputDimensionality() const
    {
        if (!m_layers.empty())
        {
            return m_layers.back().m_biases.GetDimensionality();
        }
        return m_activationCount;
    }

    AZStd::size_t MultilayerPerceptron::GetLayerCount() const
    {
        return m_layers.size();
    }

    AZ::MatrixMxN MultilayerPerceptron::GetLayerWeights(AZStd::size_t layerIndex) const
    {
        return m_layers[layerIndex].m_weights;
    }

    AZ::VectorN MultilayerPerceptron::GetLayerBiases(AZStd::size_t layerIndex) const
    {
        return m_layers[layerIndex].m_biases;
    }

    AZStd::size_t MultilayerPerceptron::GetParameterCount() const
    {
        AZStd::size_t parameterCount = 0;
        for (const Layer& layer : m_layers)
        {
            parameterCount += layer.m_inputSize * layer.m_outputSize + layer.m_outputSize;
        }
        return parameterCount;
    }

    IInferenceContextPtr MultilayerPerceptron::CreateInferenceContext()
    {
        return new MlpInferenceContext();
    }

    ITrainingContextPtr MultilayerPerceptron::CreateTrainingContext()
    {
        return new MlpTrainingContext();
    }

    const AZ::VectorN* MultilayerPerceptron::Forward(IInferenceContextPtr context, const AZ::VectorN& activations)
    {
        MlpInferenceContext* forwardContext = static_cast<MlpInferenceContext*>(context);
        forwardContext->m_layerData.resize(m_layers.size());

        const AZ::VectorN* lastLayerOutput = &activations;
        for (AZStd::size_t iter = 0; iter < m_layers.size(); ++iter)
        {
            m_layers[iter].Forward(forwardContext->m_layerData[iter], *lastLayerOutput);
            lastLayerOutput = &forwardContext->m_layerData[iter].m_output;
        }
        return lastLayerOutput;
    }

    void MultilayerPerceptron::Reverse(ITrainingContextPtr context, LossFunctions lossFunction, const AZ::VectorN& activations, const AZ::VectorN& expected)
    {
        MlpTrainingContext* reverseContext = static_cast<MlpTrainingContext*>(context);
        MlpInferenceContext* forwardContext = &reverseContext->m_forward;
        reverseContext->m_layerData.resize(m_layers.size());
        forwardContext->m_layerData.resize(m_layers.size());

        ++reverseContext->m_trainingSampleSize;

        // First feed-forward the activations to get our current model predictions
        // We do additional book-keeping over a standard forward pass to make gradient calculations easier
        const AZ::VectorN* lastLayerOutput = &activations;
        for (AZStd::size_t iter = 0; iter < m_layers.size(); ++iter)
        {
            reverseContext->m_layerData[iter].m_lastInput = lastLayerOutput;
            m_layers[iter].Forward(forwardContext->m_layerData[iter], *lastLayerOutput);
            lastLayerOutput = &forwardContext->m_layerData[iter].m_output;
        }

        // Compute the partial derivatives of the loss function with respect to the final layer output
        AZ::VectorN costGradients;
        ComputeLoss_Derivative(lossFunction, *lastLayerOutput, expected, costGradients);

        AZ::VectorN* lossGradient = &costGradients;
        for (int64_t iter = static_cast<int64_t>(m_layers.size()) - 1; iter >= 0; --iter)
        {
            m_layers[iter].AccumulateGradients(reverseContext->m_trainingSampleSize, reverseContext->m_layerData[iter], forwardContext->m_layerData[iter], *lossGradient);
            lossGradient = &reverseContext->m_layerData[iter].m_backpropagationGradients;
        }
    }

    void MultilayerPerceptron::GradientDescent(ITrainingContextPtr context, float learningRate)
    {
        MlpTrainingContext* reverseContext = static_cast<MlpTrainingContext*>(context);
        if (reverseContext->m_trainingSampleSize > 0)
        {
            for (AZStd::size_t iter = 0; iter < m_layers.size(); ++iter)
            {
                m_layers[iter].ApplyGradients(reverseContext->m_layerData[iter], learningRate);
            }
        }
        reverseContext->m_trainingSampleSize = 0;
    }

    void MultilayerPerceptron::OnActivationCountChanged()
    {
        AZStd::size_t lastLayerDimensionality = m_activationCount;
        for (Layer& layer : m_layers)
        {
            layer.m_inputSize = lastLayerDimensionality;
            layer.OnSizesChanged();
            lastLayerDimensionality = layer.m_outputSize;
        }
    }

    bool MultilayerPerceptron::LoadModel()
    {
        AZ::IO::SystemFile modelFile;
        AZ::IO::FixedMaxPath filePathFixed = m_modelFile.c_str();
        if (AZ::IO::FileIOBase* fileIOBase = AZ::IO::FileIOBase::GetInstance())
        {
            fileIOBase->ResolvePath(filePathFixed, m_modelFile.c_str());
        }

        if (!modelFile.Open(filePathFixed.c_str(), AZ::IO::SystemFile::SF_OPEN_READ_ONLY))
        {
            AZLOG_ERROR("Failed to load '%s'. File could not be opened.", filePathFixed.c_str());
            return false;
        }

        const AZ::IO::SizeType length = modelFile.Length();
        if (length == 0)
        {
            AZLOG_ERROR("Failed to load '%s'. File is empty.", filePathFixed.c_str());
            return false;
        }

        AZStd::vector<uint8_t> serializeBuffer;
        serializeBuffer.resize(length);
        modelFile.Seek(0, AZ::IO::SystemFile::SF_SEEK_BEGIN);
        modelFile.Read(serializeBuffer.size(), serializeBuffer.data());
        AzNetworking::NetworkOutputSerializer serializer(serializeBuffer.data(), static_cast<uint32_t>(serializeBuffer.size()));
        return Serialize(serializer);
    }

    bool MultilayerPerceptron::SaveModel()
    {
        AZ::IO::SystemFile modelFile;
        AZ::IO::FixedMaxPath filePathFixed = m_modelFile.c_str();
        if (AZ::IO::FileIOBase* fileIOBase = AZ::IO::FileIOBase::GetInstance())
        {
            fileIOBase->ResolvePath(filePathFixed, m_modelFile.c_str());
        }

        if (!modelFile.Open(filePathFixed.c_str(), AZ::IO::SystemFile::SF_OPEN_READ_WRITE | AZ::IO::SystemFile::SF_OPEN_CREATE))
        {
            AZLOG_ERROR("Failed to save to '%s'. File could not be opened for writing.", filePathFixed.c_str());
            return false;
        }
        modelFile.Seek(0, AZ::IO::SystemFile::SF_SEEK_BEGIN);

        AZStd::vector<uint8_t> serializeBuffer;
        serializeBuffer.resize(EstimateSerializeSize());
        AzNetworking::NetworkInputSerializer serializer(serializeBuffer.data(), static_cast<uint32_t>(serializeBuffer.size()));
        if (Serialize(serializer))
        {
            modelFile.Write(serializeBuffer.data(), serializeBuffer.size());
            return true;
        }

        return false;
    }

    void MultilayerPerceptron::AddLayer(AZStd::size_t layerDimensionality, ActivationFunctions activationFunction)
    {
        // This is not thread safe, this should only be used during model configuration
        const AZStd::size_t lastLayerDimensionality = GetOutputDimensionality();
        m_layers.push_back(AZStd::move(Layer(activationFunction, lastLayerDimensionality, layerDimensionality)));
    }

    Layer* MultilayerPerceptron::GetLayer(AZStd::size_t layerIndex)
    {
        // This is not thread safe, this method should only be used by unit testing to inspect layer weights and biases for correctness
        return &m_layers[layerIndex];
    }

    bool MultilayerPerceptron::Serialize(AzNetworking::ISerializer& serializer)
    {
        return serializer.Serialize(m_name, "Name")
            && serializer.Serialize(m_activationCount, "activationCount")
            && serializer.Serialize(m_layers, "layers");
    }

    AZStd::size_t MultilayerPerceptron::EstimateSerializeSize() const
    {
        const AZStd::size_t padding = 64; // 64 bytes of extra padding just in case
        AZStd::size_t estimatedSize = padding 
            + sizeof(AZStd::size_t)
            + m_name.size()
            + sizeof(m_activationCount)
            + sizeof(AZStd::size_t);
        for (const Layer& layer : m_layers)
        {
            estimatedSize += layer.EstimateSerializeSize();
        }
        return estimatedSize;
    }
}
