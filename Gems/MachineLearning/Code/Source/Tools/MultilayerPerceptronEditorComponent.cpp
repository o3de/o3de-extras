/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Tools/MultilayerPerceptronEditorComponent.h>
#include <Components/MultilayerPerceptronComponent.h>
#include <MachineLearning/IMachineLearning.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistryMergeUtils.h>
#include <AzCore/Console/ILogger.h>
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/API/EditorAssetSystemAPI.h>
#include <AzToolsFramework/UI/UICore/WidgetHelpers.h>
#include <AzQtComponents/Components/Widgets/FileDialog.h>
#include <QMessageBox>

namespace MachineLearning
{
    void MultilayerPerceptronEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MultilayerPerceptronEditorComponent>()
                ->Version(0)
                ->Field("Asset", &MultilayerPerceptronEditorComponent::m_asset)
                ->Field("Model", &MultilayerPerceptronEditorComponent::m_model)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext
                    ->Class<MultilayerPerceptronEditorComponent>("Multilayer Perceptron", "")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                            ->Attribute(AZ::Edit::Attributes::Category, "MachineLearning")
                            ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/NeuralNetwork.svg")
                            ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/NeuralNetwork.svg")
                            ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptronEditorComponent::m_asset, "Asset", "This is the asset file the model is persisted to")
                            ->Attribute(AZ::Edit::Attributes::ChangeNotify, &MultilayerPerceptronEditorComponent::AssetChanged)
                            ->Attribute(AZ::Edit::Attributes::ClearNotify, &MultilayerPerceptronEditorComponent::AssetCleared)
                        ->DataElement(AZ::Edit::UIHandlers::Default, &MultilayerPerceptronEditorComponent::m_model, "Model", "This is the machine-learning model provided by this component");
            }
        }
    }

    void MultilayerPerceptronEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("MultilayerPerceptronService"));
    }

    MultilayerPerceptronEditorComponent::MultilayerPerceptronEditorComponent()
    {
        m_model.m_proxy = this;
        m_handle.reset(&m_model);
        MachineLearningInterface::Get()->RegisterModel(m_handle);
    }

    MultilayerPerceptronEditorComponent::~MultilayerPerceptronEditorComponent()
    {
        MachineLearningInterface::Get()->UnregisterModel(m_handle);
    }

    void MultilayerPerceptronEditorComponent::Activate()
    {
        AssetChanged();
    }

    void MultilayerPerceptronEditorComponent::Deactivate()
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
    }

    void MultilayerPerceptronEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        MultilayerPerceptronComponent* component = gameEntity->CreateComponent<MultilayerPerceptronComponent>();
        component->m_asset = m_asset;
    }

    bool MultilayerPerceptronEditorComponent::SaveAsset()
    {
        return SaveAsAsset();
    }

    bool MultilayerPerceptronEditorComponent::LoadAsset()
    {
        m_asset.QueueLoad();
        return true;
    }

    void MultilayerPerceptronEditorComponent::AssetChanged()
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        if (m_asset.GetStatus() == AZ::Data::AssetData::AssetStatus::Error ||
            m_asset.GetStatus() == AZ::Data::AssetData::AssetStatus::NotLoaded)
        {
            m_asset.QueueLoad();
        }
        AZ::Data::AssetBus::Handler::BusConnect(m_asset.GetId());
    }

    void MultilayerPerceptronEditorComponent::AssetCleared()
    {
        ;
    }

    void MultilayerPerceptronEditorComponent::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        ModelAsset* modelAsset = asset.GetAs<ModelAsset>();
        if ((asset == m_asset) && (modelAsset != nullptr))
        {
            m_model = *modelAsset;
            AzToolsFramework::ToolsApplicationNotificationBus::Broadcast
            (
                &AzToolsFramework::ToolsApplicationNotificationBus::Events::InvalidatePropertyDisplay, 
                AzToolsFramework::Refresh_EntireTree
            );
        }
    }

    void MultilayerPerceptronEditorComponent::OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        OnAssetReady(asset);
    }

    void MultilayerPerceptronEditorComponent::OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_asset)
        {
            AZLOG_WARN("OnAssetError: %s", asset.GetHint().c_str());
        }
    }

    void MultilayerPerceptronEditorComponent::OnAssetReloadError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        if (asset == m_asset)
        {
            AZLOG_WARN("OnAssetReloadError: %s", asset.GetHint().c_str());
        }
    }

    static AZStd::string PathAtProjectRoot(const AZStd::string_view name, const AZStd::string_view extension)
    {
        AZ::IO::Path projectPath;
        if (auto settingsRegistry = AZ::SettingsRegistry::Get(); settingsRegistry != nullptr)
        {
            settingsRegistry->Get(projectPath.Native(), AZ::SettingsRegistryMergeUtils::FilePathKey_ProjectPath);
        }
        projectPath /= AZ::IO::FixedMaxPathString::format("%.*s.%.*s", AZ_STRING_ARG(name), AZ_STRING_ARG(extension));
        return projectPath.Native();
    }

    template <typename T>
    AZ::Data::Asset<T> CreateOrFindAsset(const AZStd::string& assetPath, AZ::Data::AssetLoadBehavior loadBehavior)
    {
        AZ::Data::AssetId generatedAssetId;
        AZ::Data::AssetCatalogRequestBus::BroadcastResult
        (
            generatedAssetId, 
            &AZ::Data::AssetCatalogRequests::GenerateAssetIdTEMP, 
            assetPath.c_str()
        );
        return AZ::Data::AssetManager::Instance().FindOrCreateAsset(generatedAssetId, azrtti_typeid<T>(), loadBehavior);
    }

    bool MultilayerPerceptronEditorComponent::SaveAsAsset()
    {
        if (m_asset.Get() != nullptr)
        {
            m_asset->m_name = m_model.m_name;
            m_asset->m_activationCount = m_model.m_activationCount;
            m_asset->m_layers = m_model.m_layers;
            return m_asset.Save();
        }

        const AZStd::string initialAbsolutePathToSave = PathAtProjectRoot(m_model.GetName().c_str(), ModelAsset::Extension);
        const QString fileFilter = AZStd::string::format("Model (*.%s)", ModelAsset::Extension).c_str();
        const QString absolutePathQt = AzQtComponents::FileDialog::GetSaveFileName(nullptr, "Save As Asset...", QString(initialAbsolutePathToSave.c_str()), fileFilter);
        const AZStd::string absolutePath = AZStd::string(absolutePathQt.toUtf8());

        // User cancelled
        if (absolutePathQt.isEmpty())
        {
            return false;
        }

        // Copy m_model to m_asset so we can save latest data
        m_asset = CreateOrFindAsset<ModelAsset>(absolutePath, m_asset.GetAutoLoadBehavior());
        m_asset->m_name = m_model.m_name;
        m_asset->m_activationCount = m_model.m_activationCount;
        m_asset->m_layers = m_model.m_layers;

        AZ::Data::AssetBus::Handler::BusDisconnect();
        AZ::Data::AssetBus::Handler::BusConnect(m_asset.GetId());

        bool result = false;
        const auto assetType = AZ::AzTypeInfo<ModelAsset>::Uuid();
        if (auto assetHandler = AZ::Data::AssetManager::Instance().GetHandler(assetType))
        {
            if (AZ::IO::FileIOStream fileStream(absolutePath.c_str(), AZ::IO::OpenMode::ModeWrite); fileStream.IsOpen())
            {
                result = assetHandler->SaveAssetData(m_asset, &fileStream);
                AZLOG_INFO("Save %s. Location: %s", result ? "succeeded" : "failed", absolutePath.c_str());
            }
        }

        AzToolsFramework::ToolsApplicationNotificationBus::Broadcast
        (
            &AzToolsFramework::ToolsApplicationNotificationBus::Events::InvalidatePropertyDisplay,
            AzToolsFramework::Refresh_EntireTree
        );

        return result;
    }
}
