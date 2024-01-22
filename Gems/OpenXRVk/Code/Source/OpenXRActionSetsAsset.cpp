/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/ObjectStream.h>

#include "OpenXRActionSetsAsset.h"

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRActionBindingsAsset
    void OpenXRActionSetsAsset::Reflect(AZ::ReflectContext* context)
    {
        OpenXRActionSetDescriptor::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionSetsAsset, AZ::Data::AssetData>()
                ->Version(1)
                ->Attribute(AZ::Edit::Attributes::EnableForAssetEditor, true)
                ->Field("ActionSetDescriptors", &OpenXRActionSetsAsset::m_actionSetDescriptors)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionSetsAsset>(
                    s_assetTypeName, "Defines the OpenXR Actions an application cares about.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSetsAsset::m_actionSetDescriptors, "Action Sets", "List of action sets.")
                    ;
            }
        }
    }
    /// OpenXRActionBindingsAsset
    ///////////////////////////////////////////////////////////

} // namespace OpenXRVk
