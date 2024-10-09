/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "GeoreferenceLevelEditorComponent.h"

namespace ROS2
{

    GeoReferenceLevelEditorComponent::GeoReferenceLevelEditorComponent(const GeoReferenceLevelConfig& configuration)
        : GeoReferenceLevelEditorComponentBase(configuration)
    {
    }

    void GeoReferenceLevelEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        GeoReferenceLevelEditorComponentBase::Reflect(context);

        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<GeoReferenceLevelEditorComponent, GeoReferenceLevelEditorComponentBase>()->Version(1);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext
                    ->Class<GeoReferenceLevelEditorComponent>(
                        "GeoReference Level Editor Component", "Component allows to provide georeference level for the level")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Component allows to provide georeference level for the level")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Level"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2GNSSSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void GeoReferenceLevelEditorComponent::Activate()
    {
        GeoReferenceLevelEditorComponentBase::Activate();
    }

    void GeoReferenceLevelEditorComponent::Deactivate()
    {
        GeoReferenceLevelEditorComponentBase::Deactivate();
    }
    bool GeoReferenceLevelEditorComponent::ShouldActivateController() const
    {
        return true;
    };

} // namespace ROS2
