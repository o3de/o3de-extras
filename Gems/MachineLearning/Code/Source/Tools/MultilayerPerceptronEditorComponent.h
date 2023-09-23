/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>
#include <AzCore/Asset/AssetCommon.h>
#include <Models/MultilayerPerceptron.h>
#include <Assets/ModelAsset.h>
#include <MachineLearning/Types.h>

namespace MachineLearning
{
    class MultilayerPerceptronEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AZ::Data::AssetBus::Handler
        , public IAssetPersistenceProxy
    {
    public:

        AZ_COMPONENT(MultilayerPerceptronEditorComponent, "{E33802A1-18E8-4CBE-A45B-7D7C979B1027}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        MultilayerPerceptronEditorComponent();
        ~MultilayerPerceptronEditorComponent();

        //! AZ::Component overrides
        //! @{
        void Activate() override;
        void Deactivate() override;
        //! @}

        //! EditorComponentBase
        //! @{
        void BuildGameEntity(AZ::Entity* gameEntity) override;
        //! @}

        //! IAssetPersistenceProxy overrides
        //! @{
        bool SaveAsset() override;
        bool LoadAsset() override;
        //! @}

    private:

        //! Edit context callbacks
        //! @{
        void AssetChanged();
        void AssetCleared();
        //! @}

        //! AZ::Data::AssetBus overrides
        //! @{
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloadError(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        //! @}

        bool SaveAsAsset();

        //! The model asset.
        AZ::Data::Asset<ModelAsset> m_asset;

        MultilayerPerceptron m_model;
        INeuralNetworkPtr m_handle;
    };
}
