/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */
#pragma once

#include <AzCore/Component/Component.h>

#include "GeoreferenceLevelComponent.h"
#include <AzToolsFramework/ToolsComponents/EditorComponentAdapter.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>


namespace ROS2
{
    using GeoReferenceLevelEditorComponentBase = AzToolsFramework::Components::
        EditorComponentAdapter<GeoReferenceLevelController, GeoReferenceLevelComponent, GeoReferenceLevelConfig>;

    class GeoReferenceLevelEditorComponent
        : public GeoReferenceLevelEditorComponentBase
    {
    public:
        GeoReferenceLevelEditorComponent() = default;
        explicit GeoReferenceLevelEditorComponent(const GeoReferenceLevelConfig& configuration);

        AZ_EDITOR_COMPONENT(GeoReferenceLevelEditorComponent, "{b2c22304-d414-455c-808b-290142e59af0}", GeoReferenceLevelEditorComponentBase);
        static void Reflect(AZ::ReflectContext* context);

        // GeoReferenceLevelEditorComponentBase interface overrides...
        void Activate() override;
        void Deactivate() override;
        bool ShouldActivateController() const override;


    };
} // namespace ROS2
