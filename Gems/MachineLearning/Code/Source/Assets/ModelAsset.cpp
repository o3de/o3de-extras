/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Assets/ModelAsset.h>
#include <AzNetworking/Serialization/NetworkInputSerializer.h>
#include <AzNetworking/Serialization/NetworkOutputSerializer.h>

namespace MachineLearning
{
    void ModelAsset::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ModelAsset>()
                ->Version(1);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ModelAsset>("ML Model Asset", "ML Model Asset")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "");
            }
        }
    }

    bool ModelAsset::Serialize(AzNetworking::ISerializer& serializer)
    {
        return serializer.Serialize(m_name, "Name")
            && serializer.Serialize(m_activationCount, "activationCount")
            && serializer.Serialize(m_layers, "layers");
    }

    AZStd::size_t ModelAsset::EstimateSerializeSize() const
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

    ModelAssetHandler::ModelAssetHandler()
        : AzFramework::GenericAssetHandler<ModelAsset>(ModelAsset::DisplayName, ModelAsset::Group, ModelAsset::Extension)
    {
    }

    AZ::Data::AssetHandler::LoadResult ModelAssetHandler::LoadAssetData
    (
        const AZ::Data::Asset<AZ::Data::AssetData>& asset, 
        AZStd::shared_ptr<AZ::Data::AssetDataStream> stream,
        [[maybe_unused]]const AZ::Data::AssetFilterCB& assetLoadFilterCB
    )
    {
        ModelAsset* assetData = asset.GetAs<ModelAsset>();
        AZ_Assert(assetData, "Asset is of the wrong type.");

        const AZ::IO::SizeType length = stream->GetLength();

        AZStd::vector<uint8_t> serializeBuffer;
        serializeBuffer.resize(length);
        stream->Read(length, serializeBuffer.data());
        AzNetworking::NetworkOutputSerializer serializer(serializeBuffer.data(), static_cast<uint32_t>(serializeBuffer.size()));
        if (assetData->Serialize(serializer))
        {
            return AZ::Data::AssetHandler::LoadResult::LoadComplete;
        }

        return AZ::Data::AssetHandler::LoadResult::Error;
    }

    bool ModelAssetHandler::SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream)
    {
        ModelAsset* assetData = asset.GetAs<ModelAsset>();
        AZ_Assert(assetData, "Asset is of the wrong type.");

        AZStd::vector<uint8_t> serializeBuffer;
        serializeBuffer.resize(assetData->EstimateSerializeSize());
        AzNetworking::NetworkInputSerializer serializer(serializeBuffer.data(), static_cast<uint32_t>(serializeBuffer.size()));
        if (assetData->Serialize(serializer))
        {
            stream->Write(serializer.GetSize(), serializeBuffer.data());
            return true;
        }

        return false;
    }
}
