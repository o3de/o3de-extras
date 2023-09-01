/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/MatrixMxN.h>
#include <AzNetworking/Serialization/ISerializer.h>
#include <MachineLearning/INeuralNetwork.h>
#include <Models/Layer.h>
#include <Assets/ModelAsset.h>

namespace MachineLearning
{
    //! This is a basic multilayer perceptron neural network capable of basic training and feed forward operations.
    class MultilayerPerceptron
        : public INeuralNetwork
    {
    public:

        AZ_RTTI(MultilayerPerceptron, "{E12EF761-41A5-48C3-BF55-7179B280D45F}", INeuralNetwork);

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        MultilayerPerceptron();
        MultilayerPerceptron(const MultilayerPerceptron&);
        MultilayerPerceptron(AZStd::size_t activationCount);
        virtual ~MultilayerPerceptron();

        MultilayerPerceptron& operator=(const MultilayerPerceptron&);

        //! INeuralNetwork interface
        //! @{
        AZStd::string GetName() const override;
        AZStd::string GetAssetFile(AssetTypes assetType) const override;
        AZStd::size_t GetInputDimensionality() const override;
        AZStd::size_t GetOutputDimensionality() const override;
        AZStd::size_t GetLayerCount() const override;
        AZ::MatrixMxN GetLayerWeights(AZStd::size_t layerIndex) const override;
        AZ::VectorN GetLayerBiases(AZStd::size_t layerIndex) const override;
        AZStd::size_t GetParameterCount() const override;
        IInferenceContextPtr CreateInferenceContext() override;
        ITrainingContextPtr CreateTrainingContext() override;
        const AZ::VectorN* Forward(IInferenceContextPtr context, const AZ::VectorN& activations) override;
        void Reverse(ITrainingContextPtr context, LossFunctions lossFunction, const AZ::VectorN& activations, const AZ::VectorN& expected) override;
        void GradientDescent(ITrainingContextPtr context, float learningRate) override;
        bool LoadModel() override;
        bool SaveModel() override;
        //! @}

        //! Adds a new layer to the model.
        void AddLayer(AZStd::size_t layerDimensionality, ActivationFunctions activationFunction = ActivationFunctions::ReLU);

        //! Retrieves a specific layer from the model, this is not thread safe and should only be used during unit testing to validate model parameters.
        Layer* GetLayer(AZStd::size_t layerIndex);

        //! Base serialize method for all serializable structures or classes to implement.
        //! @param serializer ISerializer instance to use for serialization
        //! @return boolean true for success, false for serialization failure
        bool Serialize(AzNetworking::ISerializer& serializer);

        //! Returns the estimated size required to serialize this model.
        AZStd::size_t EstimateSerializeSize() const;

    private:

        void OnActivationCountChanged();

        //! The model asset.
        AZ::Data::Asset<ModelAsset> m_asset;

        //! The model name.
        AZStd::string m_name;

        //! The model asset file.
        AZStd::string m_modelFile;

        //! Optional test and train asset data files.
        AZStd::string m_testDataFile;
        AZStd::string m_testLabelFile;
        AZStd::string m_trainDataFile;
        AZStd::string m_trainLabelFile;

        //! The number of neurons in the activation layer.
        AZStd::size_t m_activationCount = 0;

        //! The set of layers in the network.
        AZStd::vector<Layer> m_layers;
    };

    struct MlpInferenceContext
        : public IInferenceContext
    {
        AZStd::vector<LayerInferenceData> m_layerData;
    };

    struct MlpTrainingContext
        : public ITrainingContext
    {
        //! Used during the forward pass when calculating loss gradients.
        MlpInferenceContext m_forward;

        //! The number of accumulated training samples.
        AZStd::size_t m_trainingSampleSize = 0;

        //! The set of layer training data.
        AZStd::vector<LayerTrainingData> m_layerData;
    };
}
