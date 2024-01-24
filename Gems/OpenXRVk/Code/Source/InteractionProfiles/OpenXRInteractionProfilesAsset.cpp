/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRInteractionProfilesAsset.h>

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRInteractionProfilesAsset
    /*static*/ AZStd::string OpenXRInteractionProfilesAsset::GetInteractionProfilesAssetPath()
    {
        //FIXME! GALIB
        return "system.xrprofiles";
    }

    void OpenXRInteractionProfilesAsset::Reflect(AZ::ReflectContext* context)
    {
        OpenXRInteractionProfileDescriptor::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRInteractionProfilesAsset, AZ::Data::AssetData>()
                ->Version(1)
                ->Attribute(AZ::Edit::Attributes::EnableForAssetEditor, true)
                ->Field("InteractionProfiles", &OpenXRInteractionProfilesAsset::m_interactionProfileDescriptors)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRInteractionProfilesAsset>(
                    s_assetTypeName, "Defines the OpenXR Interaction Profiles supported by O3DE.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionProfilesAsset::m_interactionProfileDescriptors, "Interaction Profiles", "List of interaction profile descriptors.")
                    ;
            }
        }
    }

    const OpenXRInteractionProfileDescriptor* OpenXRInteractionProfilesAsset::GetInteractionProfileDescriptor(const AZStd::string& profileName) const
    {
        for (const auto& profileDescriptor : m_interactionProfileDescriptors)
        {
            if (profileName == profileDescriptor.m_uniqueName)
            {
                return &profileDescriptor;
            }
        }
        return nullptr;
    }

    /// OpenXRInteractionProfilesAsset
    ///////////////////////////////////////////////////////////

    OpenXRInteractionProfilesAssetHandler::OpenXRInteractionProfilesAssetHandler()
        : AzFramework::GenericAssetHandler<OpenXRInteractionProfilesAsset>(
            OpenXRInteractionProfilesAsset::s_assetTypeName,
            "Other",
            OpenXRInteractionProfilesAsset::s_assetExtension)
    {
    }

    bool OpenXRInteractionProfilesAssetHandler::SaveAssetData(const AZ::Data::Asset<AZ::Data::AssetData>& asset, AZ::IO::GenericStream* stream)
    {
        auto profileAsset = asset.GetAs<OpenXRInteractionProfilesAsset>();
        if (!profileAsset)
        {
            AZ_Error(LogName, false, "This should be an OpenXR Interaction Profile Asset, as this is the only type this handler can process.");
            return false;
        }
        const auto& descriptorsList = profileAsset->m_interactionProfileDescriptors;
        if (descriptorsList.empty())
        {
            AZ_Error(LogName, false, "The list of Interaction Profile Descriptors is empty.");
            return false;
        }

        if (!m_serializeContext)
        {
            AZ_Error(LogName, false, "Can't save the OpenXR Interaction Profile Asset without a serialize context.");
            return false;
        }

        for (const auto& profileDescriptor : descriptorsList)
        {
            if (!profileDescriptor.Validate())
            {
                return false;
            }
        }
        return AZ::Utils::SaveObjectToStream(*stream, AZ::ObjectStream::ST_JSON, profileAsset,
            asset->RTTI_GetType(), m_serializeContext);
    }

} // namespace OpenXRVk
