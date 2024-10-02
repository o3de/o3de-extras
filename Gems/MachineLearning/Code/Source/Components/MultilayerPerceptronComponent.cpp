/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Components/MultilayerPerceptronComponent.h>
#include <MachineLearning/IMachineLearning.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Console/ILogger.h>

namespace MachineLearning
{
    void MultilayerPerceptronComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultilayerPerceptronComponent>()
                ->Version(0)
                ->Field("Asset", &MultilayerPerceptronComponent::m_asset)
                ->Field("Model", &MultilayerPerceptronComponent::m_model)
                ;
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<MultilayerPerceptronComponent>("MultilayerPerceptron Component")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "machineLearning")
                ->Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)
                ->Constructor<>()
                ->Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)
                ->Property("Model", BehaviorValueProperty(&MultilayerPerceptronComponent::m_model))
                ;

            behaviorContext->EBus<MultilayerPerceptronComponentRequestBus>("Multilayer perceptron requests")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "machinelearning")
                ->Attribute(AZ::Script::Attributes::Category, "MachineLearning")
                ->Event("Get model", &MachineLearning::MultilayerPerceptronComponentRequestBus::Events::GetModel)
                ;
        }
    }

    void MultilayerPerceptronComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("MultilayerPerceptronService"));
    }

    MultilayerPerceptronComponent::MultilayerPerceptronComponent()
    {
        m_handle.reset(&m_model);
        MachineLearningInterface::Get()->RegisterModel(m_handle);
    }

    MultilayerPerceptronComponent::~MultilayerPerceptronComponent()
    {
        MachineLearningInterface::Get()->UnregisterModel(m_handle);
    }

    void MultilayerPerceptronComponent::Activate()
    {
        MultilayerPerceptronComponentRequestBus::Handler::BusConnect(GetEntityId());
        AssetChanged();
    }

    void MultilayerPerceptronComponent::Deactivate()
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        MultilayerPerceptronComponentRequestBus::Handler::BusDisconnect();
    }

    INeuralNetworkPtr MultilayerPerceptronComponent::GetModel()
    {
        return m_handle;
    }

    void MultilayerPerceptronComponent::AssetChanged()
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        if (m_asset.GetStatus() == AZ::Data::AssetData::AssetStatus::Error ||
            m_asset.GetStatus() == AZ::Data::AssetData::AssetStatus::NotLoaded)
        {
            m_asset.QueueLoad();
        }
        AZ::Data::AssetBus::Handler::BusConnect(m_asset.GetId());
    }

    void MultilayerPerceptronComponent::AssetCleared()
    {
        ;
    }

    void MultilayerPerceptronComponent::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        ModelAsset* modelAsset = asset.GetAs<ModelAsset>();
        if ((asset == m_asset) && (modelAsset != nullptr))
        {
            m_model = *modelAsset;
        }
    }

    void MultilayerPerceptronComponent::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        OnAssetReady(asset);
    }

    void MultilayerPerceptronComponent::OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_asset)
        {
            AZLOG_WARN("OnAssetError: %s", asset.GetHint().c_str());
        }
    }

    void MultilayerPerceptronComponent::OnAssetReloadError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_asset)
        {
            AZLOG_WARN("OnAssetReloadError: %s", asset.GetHint().c_str());
        }
    }
}
