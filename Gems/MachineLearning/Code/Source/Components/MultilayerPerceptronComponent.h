/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Asset/AssetCommon.h>
#include <Models/MultilayerPerceptron.h>
#include <Assets/ModelAsset.h>

namespace MachineLearning
{
    //! Scriptbind
    class MultilayerPerceptronComponentRequests
        : public AZ::ComponentBus
    {
    public:
        //! Returns the underlying machine learning model.
        virtual INeuralNetworkPtr GetModel() = 0;
    };
    using MultilayerPerceptronComponentRequestBus = AZ::EBus<MultilayerPerceptronComponentRequests>;

    class MultilayerPerceptronComponent
        : public AZ::Component
        , private AZ::Data::AssetBus::Handler
        , public MultilayerPerceptronComponentRequestBus::Handler
    {
    public:

        AZ_COMPONENT(MultilayerPerceptronComponent, "{022E7841-1DB9-4AE5-9984-79B00A92DE58}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);

        MultilayerPerceptronComponent();
        ~MultilayerPerceptronComponent();

        //! AZ::Component overrides
        //! @{
        void Activate() override;
        void Deactivate() override;
        //! @}

        //! MultilayerPerceptronComponentBus
        //! @{
        INeuralNetworkPtr GetModel() override;
        //! @}

    private:

        // Edit context callbacks
        void AssetChanged();
        void AssetCleared();

        // AZ::Data::AssetBus ...
        void OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloaded(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset) override;
        void OnAssetReloadError(AZ::Data::Asset<AZ::Data::AssetData> asset) override;

        //! The model asset.
        AZ::Data::Asset<ModelAsset> m_asset;

        MultilayerPerceptron m_model;
        INeuralNetworkPtr m_handle;

        friend class MultilayerPerceptronEditorComponent;
    };
}
