/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzNetworking/Serialization/ISerializer.h>
#include <AzFramework/Asset/GenericAssetHandler.h>
#include <Models/Layer.h>

namespace MachineLearning
{
    class ModelAsset final
        : public AZ::Data::AssetData
    {
    public:
        static constexpr inline const char* DisplayName = "ModelAsset";
        static constexpr inline const char* Extension = "mlmodel";
        static constexpr inline const char* Group = "MachineLearning";

        AZ_RTTI(ModelAsset, "{4D8D3782-DC3A-499A-A59D-542B85F5EDE9}", AZ::Data::AssetData);
        AZ_CLASS_ALLOCATOR(ModelAsset, AZ::SystemAllocator);

        ~ModelAsset() = default;

        //! Base serialize method for all serializable structures or classes to implement.
        //! @param serializer ISerializer instance to use for serialization
        //! @return boolean true for success, false for serialization failure
        bool Serialize(AzNetworking::ISerializer& serializer);

        //! Returns the estimated size required to serialize this model.
        AZStd::size_t EstimateSerializeSize() const;

        //! The model name.
        AZStd::string m_name;

        //! The number of neurons in the activation layer.
        AZStd::size_t m_activationCount = 0;

        //! The set of layers in the network.
        AZStd::vector<Layer> m_layers;
    };

    class ModelAssetHandler final
        : public AzFramework::GenericAssetHandler<ModelAsset>
    {
    public:
        ModelAssetHandler();

    private:
        AZ::Data::AssetHandler::LoadResult LoadAssetData
        (
            const AZ::Data::Asset<AZ::Data::AssetData>& asset, 
            AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
            const AZ::Data::AssetFilterCB& assetLoadFilterCB
        ) override;

        bool SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream) override;
    };
}
