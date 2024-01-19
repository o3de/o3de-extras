/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "OpenXRInteractionProfilesAsset.h"

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRActionBindingsAsset
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
    /// OpenXRActionBindingsAsset
    ///////////////////////////////////////////////////////////

} // namespace OpenXRVk
